#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
from collections import defaultdict, deque
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from openpilot.tools.lib.logreader import LogReader, ReadMode


TRACK_MIN = 0xC4100
TRACK_MAX = 0xC41FF
HISTORY_SECONDS = 1.0
FUTURE_MIN_SECONDS = 0.25
FUTURE_MAX_SECONDS = 0.60


@dataclass(frozen=True)
class Policy:
  name: str
  min_track_frames: int
  min_selected_frames: int
  max_consensus_residual: float
  min_model_delta: float
  max_distance: float
  max_ttc: float
  blend_weight: float
  max_correction: float
  min_distance: float = 0.0
  bidirectional: bool = False
  max_model_residual: float = math.inf
  direct_radar: bool = False
  relative_accel_forecast: bool = False


POLICIES = (
  Policy("broad", 8, 3, 2.0, 1.0, 110.0, 7.0, 0.35, 4.0),
  Policy("balanced", 12, 6, 1.5, 1.0, 90.0, 6.0, 0.35, 3.0),
  Policy("strict", 16, 8, 1.0, 1.5, 75.0, 5.0, 0.30, 2.5),
  Policy("very_strict", 20, 12, 0.75, 2.0, 60.0, 4.0, 0.25, 2.0),
)


def finite(value, default=0.0):
  try:
    value = float(value)
  except Exception:
    return default
  return value if math.isfinite(value) else default


def percentile(values, pct):
  if not values:
    return None
  return round(float(np.percentile(values, pct)), 4)


def summarize(values):
  return {
    "count": len(values),
    "p50": percentile(values, 50),
    "p90": percentile(values, 90),
    "p95": percentile(values, 95),
    "max": round(max(values), 4) if values else None,
  }


def expand(patterns):
  paths = []
  for pattern in patterns:
    matches = glob.glob(pattern)
    paths.extend(matches if matches else [pattern])
  return sorted({Path(path).resolve() for path in paths if Path(path).is_file()})


def regression_rate(history):
  if len(history) < 8:
    return None
  times = np.asarray([(t - history[-1][0]) / 1e9 for t, _d in history], dtype=float)
  distances = np.asarray([d for _t, d in history], dtype=float)
  if times[-1] - times[0] < 0.30 or not np.isfinite(distances).all():
    return None
  slope = np.polyfit(times, distances, 1)[0]
  return float(slope) if math.isfinite(slope) else None


def future_rate(observations, now, distance):
  future = next(((t, d) for t, d in observations
                 if FUTURE_MIN_SECONDS <= (t - now) / 1e9 <= FUTURE_MAX_SECONDS), None)
  if future is None:
    return None
  dt = (future[0] - now) / 1e9
  return (future[1] - distance) / dt, dt


def extract(raw, start, size, signed=False):
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


def policy_output(policy, sample):
  if sample["trackFrames"] < policy.min_track_frames or sample["selectedFrames"] < policy.min_selected_frames:
    return None
  if sample["observedVRel"] is None or abs(sample["rawVRel"] - sample["observedVRel"]) > policy.max_consensus_residual:
    return None
  if not policy.min_distance < sample["dRel"] <= policy.max_distance:
    return None

  model_vrel = sample["modelVRel"]
  if policy.direct_radar:
    return float(sample["rawVRel"])
  if policy.bidirectional:
    if abs(sample["rawVRel"] - model_vrel) > policy.max_model_residual:
      return None
    correction = float(np.clip(policy.blend_weight * (sample["rawVRel"] - model_vrel),
                               -policy.max_correction, policy.max_correction))
    return float(model_vrel + correction)

  # Less-negative of two independent radar estimates is deliberately conservative.
  consensus_vrel = max(sample["rawVRel"], sample["observedVRel"])
  if consensus_vrel >= model_vrel - policy.min_model_delta:
    return None
  ttc = sample["dRel"] / max(-consensus_vrel, 0.1)
  if ttc > policy.max_ttc:
    return None

  correction = min(policy.max_correction, policy.blend_weight * (model_vrel - consensus_vrel))
  return model_vrel - correction


def main():
  parser = argparse.ArgumentParser(description="Evaluate conservative R0100 velocity fusion policies")
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out")
  args = parser.parse_args()

  paths = expand(args.logs)
  samples = []
  observations = defaultdict(list)

  for path in paths:
    route_segment = path.parent.name
    histories = defaultdict(lambda: deque())
    track_frames = defaultdict(int)
    selected_track = {"leadOne": None, "leadTwo": None}
    selected_frames = defaultdict(int)
    latest = {}
    latest_dynamics = {}
    latest_model_vrel: list[float] = []

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      now = int(msg.logMonoTime)
      which = msg.which()
      if which == "can":
        for can in msg.can:
          if int(can.src) != 1 or not 0x180 <= int(can.address) <= 0x184 or len(can.dat) != 32:
            continue
          packed = int.from_bytes(bytes(can.dat), "little", signed=False)
          for offset in (0, 128):
            raw = (packed >> offset) & ((1 << 128) - 1)
            raw_track_id = extract(raw, 42, 8)
            quality = extract(raw, 32, 8)
            state = extract(raw, 55, 3)
            if raw_track_id == 0 or quality == 0 or state == 0:
              continue
            latest_dynamics[raw_track_id] = {
              "time": now,
              "rawARel": extract(raw, 116, 8, True) * 0.1,
              "rawYVRel": extract(raw, 106, 8) * 0.2 - 25.0,
            }
      elif which == "liveTracks":
        seen = set()
        latest = {}
        for point in msg.liveTracks.points:
          track_id = int(point.trackId)
          if not TRACK_MIN <= track_id <= TRACK_MAX:
            continue
          seen.add(track_id)
          d_rel = finite(point.dRel)
          raw_vrel = finite(point.vRel)
          track_frames[track_id] += 1
          history = histories[track_id]
          history.append((now, d_rel))
          while history and (now - history[0][0]) / 1e9 > HISTORY_SECONDS:
            history.popleft()
          observations[(route_segment, track_id)].append((now, d_rel))
          latest[track_id] = {
            "time": now,
            "dRel": d_rel,
            "rawVRel": raw_vrel,
            "observedVRel": regression_rate(history),
            "trackFrames": track_frames[track_id],
          }
          dynamics = latest_dynamics.get(track_id & 0xFF)
          if dynamics is not None and 0 <= now - dynamics["time"] <= int(0.12e9):
            latest[track_id].update(dynamics)
        for track_id in list(track_frames):
          if track_id not in seen:
            track_frames[track_id] = 0
            histories[track_id].clear()
      elif which == "modelV2":
        model_v_ego = finite(msg.modelV2.velocity.x[0]) if len(msg.modelV2.velocity.x) else 0.0
        latest_model_vrel = [finite(lead.v[0]) - model_v_ego if len(lead.v) else 0.0
                             for lead in list(msg.modelV2.leadsV3)[:2]]
      elif which == "radarState":
        for lead_index, lead_name in enumerate(("leadOne", "leadTwo")):
          lead = getattr(msg.radarState, lead_name)
          track_id = int(lead.radarTrackId)
          if not lead.status or not lead.radar or track_id not in latest:
            selected_track[lead_name] = None
            continue
          if selected_track[lead_name] == track_id:
            selected_frames[(lead_name, track_id)] += 1
          else:
            selected_track[lead_name] = track_id
            selected_frames[(lead_name, track_id)] = 1
          sample = dict(latest[track_id])
          sample.update({
            "routeSegment": route_segment,
            "lead": lead_name,
            "trackId": track_id,
            "selectedFrames": selected_frames[(lead_name, track_id)],
            "modelVRel": (latest_model_vrel[lead_index]
                          if lead_index < len(latest_model_vrel) else finite(lead.vRel)),
            "productionVRel": finite(lead.vRel),
          })
          samples.append(sample)

  reports = []
  for policy in POLICIES:
    model_errors = []
    fused_errors = []
    improvements = []
    corrections = []
    examples = []
    events = defaultdict(list)
    for sample in samples:
      fused = policy_output(policy, sample)
      if fused is None:
        continue
      future_result = future_rate(
        observations[(sample["routeSegment"], sample["trackId"])], sample["time"], sample["dRel"],
      )
      if future_result is None:
        continue
      future, future_dt = future_result
      if policy.relative_accel_forecast:
        if "rawARel" not in sample:
          continue
        # future_rate is the average relative velocity across the interval.
        # Under constant relative acceleration, its matching prediction is
        # v_now + 0.5 * a_rel * dt.
        fused += 0.5 * sample["rawARel"] * future_dt
      model_error = abs(sample["modelVRel"] - future)
      fused_error = abs(fused - future)
      improvement = model_error - fused_error
      model_errors.append(model_error)
      fused_errors.append(fused_error)
      improvements.append(improvement)
      corrections.append(sample["modelVRel"] - fused)
      event_key = (sample["routeSegment"], sample["lead"], sample["trackId"], sample["time"] // int(1e9))
      events[event_key].append(improvement)
      if improvement < -0.5 and len(examples) < 8:
        examples.append({
          "routeSegment": sample["routeSegment"],
          "trackId": sample["trackId"],
          "dRel": round(sample["dRel"], 3),
          "rawVRel": round(sample["rawVRel"], 3),
          "observedVRel": round(sample["observedVRel"], 3),
          "modelVRel": round(sample["modelVRel"], 3),
          "fusedVRel": round(fused, 3),
          "futureVRel": round(future, 3),
          "rawARel": round(sample.get("rawARel", math.nan), 3),
          "futureDt": round(future_dt, 3),
          "errorDelta": round(improvement, 3),
        })
    event_improvements = [float(np.median(values)) for values in events.values()]
    harmful = sum(value < -0.5 for value in improvements)
    harmful_events = sum(value < -0.5 for value in event_improvements)
    reports.append({
      "policy": asdict(policy),
      "samples": len(improvements),
      "events": len(event_improvements),
      "correction": summarize(corrections),
      "modelFutureError": summarize(model_errors),
      "fusedFutureError": summarize(fused_errors),
      "improvement": summarize(improvements),
      "harmfulSamplesOver0_5": harmful,
      "harmfulSampleFraction": round(harmful / max(len(improvements), 1), 4),
      "harmfulEventsOver0_5": harmful_events,
      "harmfulEventFraction": round(harmful_events / max(len(event_improvements), 1), 4),
      "harmfulExamples": examples,
      "actuationReady": bool(
        len(event_improvements) >= 25 and
        percentile(fused_errors, 95) < percentile(model_errors, 95) and
        harmful_events == 0
      ),
    })

  def group_diagnostics():
    group = samples
    observed = [sample for sample in group if sample["observedVRel"] is not None]
    residuals = [abs(sample["rawVRel"] - sample["observedVRel"]) for sample in observed]
    model_deltas = [sample["modelVRel"] - max(sample["rawVRel"], sample["observedVRel"]) for sample in observed]
    return {
      "samples": len(group),
      "observedSamples": len(observed),
      "rawDistanceRateResidual": summarize(residuals),
      "modelClosingDelta": summarize(model_deltas),
      "trackFrames": summarize([sample["trackFrames"] for sample in group]),
      "selectedFrames": summarize([sample["selectedFrames"] for sample in group]),
    }

  report = {
    "files": len(paths),
    "matchedSamples": len(samples),
    "policies": reports,
    "trackGroups": {
      "r0100": group_diagnostics(),
    },
    "conclusion": "no_policy_actuation_ready" if not any(item["actuationReady"] for item in reports) else "candidate_ready",
  }
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")


if __name__ == "__main__":
  main()
