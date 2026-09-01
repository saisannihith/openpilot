#!/usr/bin/env python3
"""Replay the Carnival post-lane-change acceleration-settle eligibility."""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader, ReadMode


MAX_AGE_NS = 150_000_000


def finite(value, default=0.0):
  try:
    return float(value)
  except (TypeError, ValueError):
    return default


def segment_number(path):
  try:
    return int(path.parent.name.split("--")[2])
  except (IndexError, ValueError):
    return -1


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()
  paths = []
  for pattern in args.logs:
    paths.extend(Path(item) for item in glob.glob(pattern))
  paths = sorted({path.resolve() for path in paths if path.is_file()}, key=segment_number)

  previous_state = "off"
  car_state = radar_state = long_plan = None
  car_time = radar_time = plan_time = 0
  transitions = []
  malformed = 0
  for path in paths:
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
      now = int(msg.logMonoTime)
      which = msg.which()
      try:
        if which == "carState":
          car_state, car_time = msg.carState, now
        elif which == "radarState":
          radar_state, radar_time = msg.radarState, now
        elif which == "longitudinalPlan":
          long_plan, plan_time = msg.longitudinalPlan, now
        elif which != "modelV2":
          continue
        else:
          model = msg.modelV2
          current_state = str(model.meta.laneChangeState)
          if previous_state != "off" and current_state == "off":
            context_fresh = all(now - stamp <= MAX_AGE_NS for stamp in (car_time, radar_time, plan_time))
            lead = radar_state.leadOne if context_fresh and radar_state is not None else None
            lead_status = bool(getattr(lead, "status", False))
            v_ego = finite(getattr(car_state, "vEgo", 0.0)) if context_fresh and car_state is not None else 0.0
            max_distance = min(max(2.5 * max(v_ego, 0.0), 35.0), 85.0)
            lead_distance = finite(getattr(lead, "dRel", 0.0)) if lead_status else 0.0
            lead_v_rel = finite(getattr(lead, "vRel", 0.0)) if lead_status else 0.0
            eligible = context_fresh and lead_status and lead_distance <= max_distance and lead_v_rel <= 2.0
            transitions.append({
              "path": str(path),
              "time": now,
              "vEgo": round(v_ego, 3),
              "leadStatus": lead_status,
              "leadDistance": round(lead_distance, 3),
              "leadVRel": round(lead_v_rel, 3),
              "maxDistance": round(max_distance, 3),
              "recordedPlanAccel": round(finite(getattr(long_plan, "aTarget", 0.0)), 3) if context_fresh else None,
              "settleEligible": eligible,
              "newPositiveAccelCap": 0.0 if eligible else None,
            })
          previous_state = current_state
      except Exception:
        malformed += 1

  report = {
    "status": "pass",
    "files": len(paths),
    "laneChangeEnds": len(transitions),
    "eligibleSettles": sum(item["settleEligible"] for item in transitions),
    "positiveAccelEventsPrevented": sum(
      item["settleEligible"] and item["recordedPlanAccel"] is not None and item["recordedPlanAccel"] > 0.0
      for item in transitions
    ),
    "malformedFrames": malformed,
    "transitions": transitions,
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  print(output)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
