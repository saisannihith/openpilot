#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
from collections import defaultdict, deque
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader, ReadMode

TRACK_MIN = 0xC4100
TRACK_MAX = 0xC41FF
DT_MDL = 0.05
HISTORY_FRAMES = 20
MIN_FRAMES = 8
MAX_RAW_OBS_RESIDUAL = 3.0
HOLD_FRAMES = 4
DIRECT_BLEND_MIN_FRAMES = 5
DIRECT_BLEND_MAX_RESIDUAL = 1.0
BLEND_WEIGHT = 0.35
MIN_MODEL_DELTA = 2.0
MAX_MODEL_CORRECTION = 4.0


def finite(value, default=0.0):
  try:
    value = float(value)
  except Exception:
    return default
  return value if math.isfinite(value) else default


def percentile(values: list[float], pct: float):
  if not values:
    return None
  ordered = sorted(values)
  return round(ordered[round((len(ordered) - 1) * pct / 100.0)], 4)


def summary(values: list[float]):
  return {
    "count": len(values),
    "p50": percentile(values, 50),
    "p95": percentile(values, 95),
    "max": round(max(values), 4) if values else None,
  }


def expand(patterns: list[str]):
  paths = []
  for pattern in patterns:
    matches = glob.glob(pattern)
    paths.extend(matches if matches else [pattern])
  return sorted({Path(path).resolve() for path in paths if Path(path).is_file()})


def main() -> int:
  parser = argparse.ArgumentParser(description="Replay the exact Carnival confirmation closing-rate fallback.")
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out")
  args = parser.parse_args()

  paths = expand(args.logs)
  candidates = []
  direct_samples = 0
  confirmation_samples = 0

  for path in paths:
    histories = defaultdict(lambda: deque(maxlen=HISTORY_FRAMES))
    ages = defaultdict(int)
    closing_rates = {}
    miss_frames = defaultdict(int)
    latest = {}
    observations = defaultdict(list)

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      now = int(msg.logMonoTime)
      which = msg.which()
      if which == "liveTracks":
        latest = {}
        for point in msg.liveTracks.points:
          track_id = int(point.trackId)
          if not TRACK_MIN <= track_id <= TRACK_MAX:
            continue
          d_rel = finite(point.dRel)
          raw_vrel = finite(point.vRel)
          ages[track_id] += 1
          histories[track_id].append(d_rel)
          observations[track_id].append((now, d_rel))
          observed_vrel = None
          if len(histories[track_id]) >= MIN_FRAMES:
            observed_vrel = (histories[track_id][-1] - histories[track_id][0]) / ((len(histories[track_id]) - 1) * DT_MDL)
          if observed_vrel is not None and math.isfinite(observed_vrel) and abs(raw_vrel - observed_vrel) <= MAX_RAW_OBS_RESIDUAL:
            closing_rates[track_id] = max(raw_vrel, observed_vrel)
            miss_frames[track_id] = 0
          elif track_id in closing_rates:
            miss_frames[track_id] += 1
            if miss_frames[track_id] >= HOLD_FRAMES:
              closing_rates.pop(track_id, None)
              miss_frames[track_id] = 0
          latest[track_id] = (now, d_rel, raw_vrel, observed_vrel, closing_rates.get(track_id), ages[track_id])

      elif which == "radarState":
        for lead_name in ("leadOne", "leadTwo"):
          lead = getattr(msg.radarState, lead_name)
          if not lead.status or not lead.radar:
            continue
          track_id = int(lead.radarTrackId)
          sample = latest.get(track_id)
          if sample is None:
            continue
          confirmation_samples += 1
          live_time, d_rel, raw_vrel, observed_vrel, closing_rate, age = sample
          state_vrel = finite(lead.vRel)
          direct_reachable = age >= DIRECT_BLEND_MIN_FRAMES and abs(raw_vrel - state_vrel) <= ((1.0 - BLEND_WEIGHT) * DIRECT_BLEND_MAX_RESIDUAL + 1e-3)
          model_vrel = (state_vrel - BLEND_WEIGHT * raw_vrel) / (1.0 - BLEND_WEIGHT) if direct_reachable else state_vrel
          if age >= DIRECT_BLEND_MIN_FRAMES and abs(raw_vrel - model_vrel) <= DIRECT_BLEND_MAX_RESIDUAL:
            direct_samples += 1
            continue
          if closing_rate is None or closing_rate >= model_vrel - MIN_MODEL_DELTA:
            continue

          corrected_vrel = max(model_vrel + BLEND_WEIGHT * (closing_rate - model_vrel), model_vrel - MAX_MODEL_CORRECTION)
          candidates.append({
            "path": str(path), "time": now, "trackId": track_id,
            "dRel": d_rel, "rawVRel": raw_vrel, "observedVRel": observed_vrel,
            "closingRate": closing_rate, "modelVRel": model_vrel,
            "correctedVRel": corrected_vrel, "correction": model_vrel - corrected_vrel,
            "observations": observations[track_id], "liveTime": live_time,
          })

  model_errors = []
  corrected_errors = []
  improvements = []
  harmful = []
  correction_sizes = []
  for candidate in candidates:
    correction_sizes.append(candidate["correction"])
    future = next(((t, d) for t, d in candidate["observations"] if 0.25 <= (t - candidate["liveTime"]) / 1e9 <= 0.60), None)
    if future is None:
      continue
    dt = (future[0] - candidate["liveTime"]) / 1e9
    future_vrel = (future[1] - candidate["dRel"]) / dt
    model_error = abs(candidate["modelVRel"] - future_vrel)
    corrected_error = abs(candidate["correctedVRel"] - future_vrel)
    improvement = model_error - corrected_error
    model_errors.append(model_error)
    corrected_errors.append(corrected_error)
    improvements.append(improvement)
    if improvement < -0.5 and len(harmful) < 20:
      harmful.append({
        key: round(value, 4) if isinstance(value, float) else value
        for key, value in candidate.items() if key not in ("observations", "path")
      } | {"futureVRel": round(future_vrel, 4), "errorDelta": round(improvement, 4), "file": Path(candidate["path"]).parent.name})

  worsened = sum(value < -0.5 for value in improvements)
  report = {
    "files": len(paths),
    "confirmationLeadSamples": confirmation_samples,
    "directBlendSamples": direct_samples,
    "fallbackCandidateSamples": len(candidates),
    "futureValidatedSamples": len(improvements),
    "correctionMagnitude": summary(correction_sizes),
    "modelFutureError": summary(model_errors),
    "correctedFutureError": summary(corrected_errors),
    "improvement": summary(improvements),
    "worsenedByOver0_5": worsened,
    "improvedByOver0_5": sum(value > 0.5 for value in improvements),
    "harmfulExamples": harmful,
    "actuationReady": bool(len(improvements) >= 100 and percentile(corrected_errors, 95) is not None
                           and percentile(corrected_errors, 95) < percentile(model_errors, 95)
                           and worsened / max(len(improvements), 1) < 0.01),
  }
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
