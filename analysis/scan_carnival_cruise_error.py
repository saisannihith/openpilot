#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from glob import glob
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode


COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0


def safe_float(obj: Any, name: str, default: float = 0.0) -> float:
  try:
    value = float(getattr(obj, name))
  except Exception:
    return default
  return value if math.isfinite(value) else default


def safe_bool(obj: Any, name: str, default: bool = False) -> bool:
  try:
    return bool(getattr(obj, name))
  except Exception:
    return default


def route_and_segment(path: str) -> tuple[str, int]:
  parts = Path(path).parent.name.split("--")
  route = "--".join(parts[:2]) if len(parts) >= 2 else Path(path).parent.name
  try:
    segment = int(parts[2])
  except Exception:
    segment = -1
  return route, segment


def expand_logs(patterns: list[str]) -> list[str]:
  paths: list[str] = []
  for pattern in patterns:
    matches = sorted(glob(pattern))
    paths.extend(matches or [pattern])
  return sorted(dict.fromkeys(paths))


def scan(paths: list[str]) -> dict[str, Any]:
  underspeed: list[dict[str, Any]] = []
  close_stopped: list[dict[str, Any]] = []

  for path in paths:
    latest: dict[str, Any] = {}
    start_ns: int | None = None
    route, segment = route_and_segment(path)

    for msg in LogReader(path, default_mode=ReadMode.QLOG, sort_by_time=True):
      which = msg.which()
      mono_time = int(msg.logMonoTime)
      if start_ns is None and which in ("carState", "longitudinalPlan", "controlsState"):
        start_ns = mono_time
      if which in ("carState", "carControl", "controlsState", "longitudinalPlan", "starpilotPlan", "radarState", "modelV2"):
        latest[which] = getattr(msg, which)
      if which != "longitudinalPlan" or start_ns is None:
        continue
      if not all(key in latest for key in ("carState", "carControl", "longitudinalPlan", "starpilotPlan", "radarState")):
        continue

      car_state = latest["carState"]
      car_control = latest["carControl"]
      long_plan = latest["longitudinalPlan"]
      starpilot_plan = latest["starpilotPlan"]
      radar_state = latest["radarState"]
      lead = radar_state.leadOne if radar_state.leadOne.status else radar_state.leadTwo
      lead_status = bool(getattr(lead, "status", False))

      v_ego = safe_float(car_state, "vEgo")
      car_cruise = safe_float(car_state.cruiseState, "speed")
      plan_cruise = safe_float(starpilot_plan, "vCruise")
      v_cruise = car_cruise if car_cruise > 0.1 else plan_cruise
      cmd_accel = safe_float(car_control.actuators, "accel")
      plan_accel = safe_float(long_plan, "aTarget")
      traj_accels = list(getattr(long_plan, "accels", []))
      traj_accel0 = float(traj_accels[0]) if len(traj_accels) > 0 else 0.0
      traj_accel1s = float(traj_accels[5]) if len(traj_accels) > 5 else traj_accel0
      model = latest.get("modelV2")
      model_action = getattr(model, "action", None)
      model_accel = safe_float(model_action, "desiredAcceleration") if model_action is not None else 0.0
      model_should_stop = safe_bool(model_action, "shouldStop") if model_action is not None else False
      d_rel = safe_float(lead, "dRel", 999.0) if lead_status else 999.0
      v_lead = safe_float(lead, "vLead") if lead_status else 0.0
      radar_track_id = int(getattr(lead, "radarTrackId", -1)) if lead_status else -1
      headway = d_rel / max(v_ego, 0.1) if lead_status else 999.0
      t_follow = safe_float(starpilot_plan, "tFollow", 1.45)
      desired_gap = (
        (v_ego ** 2) / (2.0 * COMFORT_BRAKE) +
        t_follow * v_ego +
        STOP_DISTANCE -
        (v_lead ** 2) / (2.0 * COMFORT_BRAKE)
      ) if lead_status else 0.0
      gap_error = d_rel - desired_gap if lead_status else 0.0
      recovery_candidate = (
        lead_status and 0xC4100 <= radar_track_id <= 0xC41FF and
        v_ego >= 13.0 and v_cruise - v_ego >= 4.0 and
        d_rel >= 45.0 and gap_error >= 12.0 and
        max(v_ego - v_lead, 0.0) <= 3.0 and
        max(0.0, -safe_float(lead, "aLeadK")) <= 0.25 and
        safe_float(lead, "modelProb", 1.0) >= 0.85 and
        v_lead >= 8.0 and
        not safe_bool(starpilot_plan, "redLight") and
        not safe_bool(starpilot_plan, "forcingStop") and
        not model_should_stop
      )
      stop_hold_candidate = (
        lead_status and 0xC4100 <= radar_track_id <= 0xC41FF and
        abs(safe_float(lead, "yRel")) <= 1.75 and
        v_ego <= 5.5 and d_rel <= 11.5 and
        v_lead <= 1.2 and
        v_ego - v_lead >= 0.7 and
        not safe_bool(car_state, "gasPressed")
      )

      rec = {
        "route": route,
        "segment": segment,
        "t": round((mono_time - start_ns) / 1e9, 2),
        "vEgo": round(v_ego, 2),
        "vCruise": round(v_cruise, 2),
        "cruiseError": round(v_cruise - v_ego, 2),
        "cmdAccel": round(cmd_accel, 3),
        "planAccel": round(plan_accel, 3),
        "trajAccel0": round(traj_accel0, 3),
        "trajAccel1s": round(traj_accel1s, 3),
        "modelAccel": round(model_accel, 3),
        "modelShouldStop": model_should_stop,
        "dRel": round(d_rel, 1),
        "vLead": round(v_lead, 2),
        "closing": round(v_ego - v_lead, 2),
        "headway": round(headway, 2),
        "tFollow": round(t_follow, 2),
        "desiredGap": round(desired_gap, 1),
        "gapError": round(gap_error, 1),
        "recoveryCandidate": recovery_candidate,
        "stopHoldCandidate": stop_hold_candidate,
        "radar": safe_bool(lead, "radar") if lead_status else False,
        "radarTrackId": radar_track_id,
        "carnivalConfirmation": 0xC4100 <= radar_track_id <= 0xC41FF,
        "modelProb": round(safe_float(lead, "modelProb"), 3) if lead_status else 0.0,
        "source": str(getattr(long_plan, "longitudinalPlanSource", "")),
        "redLight": safe_bool(starpilot_plan, "redLight"),
        "forcingStop": safe_bool(starpilot_plan, "forcingStop"),
        "shouldStop": safe_bool(long_plan, "shouldStop"),
      }

      long_active = safe_bool(car_control, "longActive")
      no_override = not safe_bool(car_state, "brakePressed") and not safe_bool(car_state, "gasPressed")
      stop_context = rec["redLight"] or rec["forcingStop"] or rec["shouldStop"]

      if (
        long_active and no_override and not stop_context and
        v_cruise > 10.0 and v_cruise - v_ego > 3.0 and
        d_rel > 45.0 and cmd_accel < 0.35
      ):
        underspeed.append(rec)

      if (
        long_active and no_override and lead_status and
        d_rel < 12.0 and v_ego > 1.0 and v_lead < 1.2
      ):
        close_stopped.append(rec)

  return {
    "files": len(paths),
    "underspeedCount": len(underspeed),
    "underspeedExamples": underspeed[:40],
    "underspeedWorst": sorted(underspeed, key=lambda x: (-x["cruiseError"], x["cmdAccel"]))[:30],
    "closeStoppedCount": len(close_stopped),
    "closeStoppedExamples": close_stopped[:40],
    "closeStoppedWorst": sorted(close_stopped, key=lambda x: (x["dRel"], -x["vEgo"]))[:30],
  }


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out")
  args = parser.parse_args()

  result = scan(expand_logs(args.logs))
  text = json.dumps(result, indent=2)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")


if __name__ == "__main__":
  main()
