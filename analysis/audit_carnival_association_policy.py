#!/usr/bin/env python3
"""Compare recorded Carnival R0100 association under old and current policies."""

from __future__ import annotations

import argparse
import glob
import json
import math
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

from openpilot.tools.lib.logreader import LogReader, ReadMode


TRACK_MIN = 0xC4100
TRACK_MAX = 0xC41FF
RADAR_TO_CAMERA = 1.52
DIST_SCALE = 0.22
DIST_FLOOR = 4.0
Y_STD_SCALE = 1.5
Y_FLOOR = 1.2
V_STD_SCALE = 3.0
V_FLOOR = 4.0
NIS_GATE = 11.345
PREFERRED_NIS_GATE = 14.156
SWITCH_MARGIN = 3.841
RADAR_D_STD = 0.25
RADAR_Y_STD = 0.25
RADAR_V_STD = 0.35
MAX_CONTEXT_AGE_NS = 150_000_000


def finite(value, default=0.0):
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if math.isfinite(result) else default


def expand(patterns):
  paths = []
  for pattern in patterns:
    matches = glob.glob(pattern)
    paths.extend(matches if matches else [pattern])
  return sorted({Path(path).resolve() for path in paths if Path(path).is_file()})


def clipped(value, lower, upper):
  return min(max(value, lower), upper)


def track_matches(track, lead, v_ego, model_v_ego, moving_policy):
  try:
    distance_residual = track.dRel - (float(lead.x[0]) - RADAR_TO_CAMERA)
    lateral_residual = track.yRel + float(lead.y[0])
    velocity_residual = track.vRel - (float(lead.v[0]) - model_v_ego)
    distance_sane = abs(distance_residual) < max(DIST_FLOOR, DIST_SCALE * max(float(lead.x[0]), 0.0))
    lateral_sane = abs(lateral_residual) < max(Y_FLOOR, Y_STD_SCALE * max(float(lead.yStd[0]), 0.2))
    moving_target = track.vRel + v_ego > 3.0
    velocity_sane = (moving_policy and moving_target) or abs(velocity_residual) < max(
      V_FLOOR, V_STD_SCALE * max(float(lead.vStd[0]), 0.5),
    )
    return distance_sane and lateral_sane and velocity_sane
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


def innovation_score(track, lead, model_v_ego, moving_policy):
  try:
    moving_target = track.vRel + model_v_ego > 3.0
    residuals = (
      track.dRel - (float(lead.x[0]) - RADAR_TO_CAMERA),
      track.yRel + float(lead.y[0]),
      0.0 if moving_policy and moving_target else track.vRel - (float(lead.v[0]) - model_v_ego),
    )
    model_stds = (
      clipped(float(lead.xStd[0]), 0.75, 6.0),
      clipped(float(lead.yStd[0]), 0.25, 1.5),
      clipped(float(lead.vStd[0]), 0.5, 3.0),
    )
    radar_stds = (RADAR_D_STD, RADAR_Y_STD, RADAR_V_STD)
    return sum(residual ** 2 / (model_std ** 2 + radar_std ** 2)
               for residual, model_std, radar_std in zip(residuals, model_stds, radar_stds, strict=True))
  except (AttributeError, IndexError, TypeError, ValueError):
    return math.inf


def choose_track(tracks, lead, v_ego, model_v_ego, preferred_id, moving_policy):
  candidates = []
  for track_id, track in tracks.items():
    if not track_matches(track, lead, v_ego, model_v_ego, moving_policy):
      continue
    score = innovation_score(track, lead, model_v_ego, moving_policy)
    gate = PREFERRED_NIS_GATE if track_id == preferred_id and track.cnt >= 3 else NIS_GATE
    if score <= gate and track.vRel + v_ego > -2.0:
      candidates.append((score, track_id, track))
  if not candidates:
    return None
  best = min(candidates, key=lambda candidate: candidate[0])
  preferred = next((candidate for candidate in candidates if candidate[1] == preferred_id), None)
  return preferred[2] if preferred is not None and preferred[0] <= best[0] + SWITCH_MARGIN else best[2]


def audit_path(path):
  latest_tracks = {}
  track_counts = Counter()
  track_time = 0
  v_ego = 0.0
  car_state_time = 0
  preferred = {False: [-1, -1], True: [-1, -1]}
  metrics = Counter()
  examples = []

  for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
    now = int(msg.logMonoTime)
    which = msg.which()
    if which == "carState":
      v_ego = finite(msg.carState.vEgo)
      car_state_time = now
    elif which == "liveTracks":
      current = {}
      for point in msg.liveTracks.points:
        track_id = int(point.trackId)
        if not TRACK_MIN <= track_id <= TRACK_MAX:
          continue
        track_counts[track_id] += 1
        current[track_id] = SimpleNamespace(
          identifier=track_id,
          cnt=track_counts[track_id],
          dRel=finite(point.dRel),
          yRel=finite(point.yRel),
          vRel=finite(point.vRel),
        )
      latest_tracks = current
      track_time = now
    elif which != "modelV2" or now - track_time > MAX_CONTEXT_AGE_NS or now - car_state_time > MAX_CONTEXT_AGE_NS:
      continue

    try:
      model = msg.modelV2
    except Exception:
      metrics["malformedModelFrames"] += 1
      continue
    model_v_ego = finite(model.velocity.x[0], v_ego) if len(model.velocity.x) else v_ego
    for lead_index, lead in enumerate(list(model.leadsV3)[:2]):
      if finite(lead.prob) <= 0.35:
        continue
      selected = {}
      for moving_policy in (False, True):
        selected[moving_policy] = choose_track(
          latest_tracks, lead, v_ego, model_v_ego, preferred[moving_policy][lead_index], moving_policy,
        )
        track = selected[moving_policy]
        if track is not None:
          old_id = preferred[moving_policy][lead_index]
          if old_id >= 0 and old_id != track.identifier:
            metrics[f"{'current' if moving_policy else 'old'}Switches"] += 1
          preferred[moving_policy][lead_index] = track.identifier
        elif preferred[moving_policy][lead_index] not in latest_tracks:
          preferred[moving_policy][lead_index] = -1

      old_track = selected[False]
      current_track = selected[True]
      metrics["modelLeadFrames"] += 1
      if old_track is not None:
        metrics["oldAssociatedFrames"] += 1
      if current_track is not None:
        metrics["currentAssociatedFrames"] += 1
      if old_track is None and current_track is not None:
        metrics["recoveredFrames"] += 1
        model_v_rel = finite(lead.v[0]) - model_v_ego if len(lead.v) else 0.0
        closing_underestimate = model_v_rel - current_track.vRel
        if current_track.dRel <= 120.0 and finite(lead.prob) >= 0.75 and closing_underestimate >= 3.0:
          metrics["recoveredSevereClosingFrames"] += 1
          if len(examples) < 30:
            examples.append({
              "time": now,
              "leadIndex": lead_index,
              "trackId": current_track.identifier,
              "dRel": round(current_track.dRel, 3),
              "yRel": round(current_track.yRel, 3),
              "radarVRel": round(current_track.vRel, 3),
              "modelVRel": round(model_v_rel, 3),
              "modelProb": round(finite(lead.prob), 3),
            })
      elif old_track is not None and current_track is None:
        metrics["lostFrames"] += 1

  return {"path": str(path), "metrics": dict(metrics), "examples": examples}


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()
  results = [audit_path(path) for path in expand(args.logs)]
  totals = Counter()
  for result in results:
    totals.update(result["metrics"])
  report = {
    "status": "pass" if totals["lostFrames"] == 0 and totals["currentAssociatedFrames"] >= totals["oldAssociatedFrames"] else "fail",
    "files": len(results),
    "totals": dict(totals),
    "results": [result for result in results if result["metrics"].get("recoveredFrames") or result["metrics"].get("lostFrames")],
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  print(output)
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
