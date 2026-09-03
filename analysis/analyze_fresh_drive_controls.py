#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from openpilot.common.constants import CV
from openpilot.tools.lib.logreader import LogReader, ReadMode


PARAM_KEYS = {
  "AccelerationProfile", "AdvancedLongitudinalTune", "ConditionalChill",
  "ConditionalExperimental", "CurveSpeedController", "DecelerationProfile",
  "ExperimentalMode", "LongitudinalTune", "SpeedLimitController",
}


def safe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def safe_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except Exception:
    return default
  return result if math.isfinite(result) else default


def route_segment(path: Path) -> tuple[str, int]:
  match = re.match(r"(.+--[0-9a-f]+)--(\d+)$", path.parent.name)
  if match is None:
    return path.parent.name, -1
  return match.group(1), int(match.group(2))


def discover(paths: list[Path]) -> dict[str, list[Path]]:
  files: list[Path] = []
  for path in paths:
    if path.is_dir():
      files.extend(path.rglob("rlog.zst"))
    elif path.is_file():
      files.append(path)
  routes: dict[str, list[Path]] = defaultdict(list)
  for path in sorted(set(files), key=lambda p: route_segment(p)):
    routes[route_segment(path)[0]].append(path)
  return routes


def stats(values: list[float]) -> dict[str, float | int | None]:
  if not values:
    return {"count": 0, "mean": None, "p05": None, "p50": None, "p95": None}
  array = np.asarray(values, dtype=float)
  return {
    "count": len(values),
    "mean": round(float(np.mean(array)), 4),
    "p05": round(float(np.percentile(array, 5)), 4),
    "p50": round(float(np.percentile(array, 50)), 4),
    "p95": round(float(np.percentile(array, 95)), 4),
  }


def percent(count: int, total: int) -> float:
  return round(100.0 * count / total, 2) if total else 0.0


def init_params(init_data: Any) -> dict[str, str]:
  values: dict[str, str] = {}
  try:
    for entry in init_data.params.entries:
      key = str(entry.key)
      if key in PARAM_KEYS:
        values[key] = bytes(entry.value).decode("utf-8", errors="replace")
  except Exception:
    pass
  return values


def analyze_route(route: str, paths: list[Path]) -> dict[str, Any]:
  latest: dict[str, Any] = {}
  startup_params: dict[str, str] = {}
  software: dict[str, str] = {}
  values: dict[str, list[float]] = defaultdict(list)
  counts: dict[str, int] = defaultdict(int)
  examples: list[dict[str, Any]] = []

  for path in paths:
    segment = route_segment(path)[1]
    for msg in LogReader(str(path), default_mode=ReadMode.RLOG, sort_by_time=True):
      which = msg.which()
      if which == "initData" and not startup_params:
        startup_params = init_params(msg.initData)
        software = {
          "branch": str(msg.initData.gitBranch),
          "commit": str(msg.initData.gitCommit),
        }
      if which in ("carState", "carControl", "selfdriveState", "radarState", "starpilotPlan", "modelV2"):
        latest[which] = getattr(msg, which)
        continue
      if which != "longitudinalPlan":
        continue
      required = ("carState", "carControl", "selfdriveState", "radarState", "starpilotPlan", "modelV2")
      if not all(name in latest for name in required):
        continue

      car_state = latest["carState"]
      car_control = latest["carControl"]
      selfdrive_state = latest["selfdriveState"]
      radar_state = latest["radarState"]
      starpilot_plan = latest["starpilotPlan"]
      model = latest["modelV2"]
      long_plan = msg.longitudinalPlan

      if not bool(safe_attr(car_control, "longActive", False)):
        continue
      counts["longActiveFrames"] += 1

      v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
      raw_v_cruise = safe_float(safe_attr(car_state, "vCruise", 0.0)) * CV.KPH_TO_MS
      effective_v_cruise = safe_float(safe_attr(starpilot_plan, "vCruise", raw_v_cruise), raw_v_cruise)
      max_accel = safe_float(safe_attr(starpilot_plan, "maxAcceleration", 0.0))
      plan_accel = safe_float(safe_attr(long_plan, "aTarget", 0.0))
      cmd_accel = safe_float(safe_attr(safe_attr(car_control, "actuators"), "accel", plan_accel), plan_accel)
      model_accel = safe_float(safe_attr(safe_attr(model, "action"), "desiredAcceleration", 0.0))
      lead = safe_attr(radar_state, "leadOne")
      lead_present = bool(safe_attr(lead, "status", False))
      stop_context = bool(
        safe_attr(starpilot_plan, "redLight", False) or
        safe_attr(starpilot_plan, "forcingStop", False) or
        safe_attr(long_plan, "shouldStop", False) or
        safe_attr(safe_attr(model, "action"), "shouldStop", False)
      )
      experimental = bool(safe_attr(selfdrive_state, "experimentalMode", False))
      speed_deficit = effective_v_cruise - v_ego
      raw_speed_deficit = raw_v_cruise - v_ego
      target_reduction = raw_v_cruise - effective_v_cruise

      values["speedDeficitMps"].append(speed_deficit)
      values["rawSpeedDeficitMps"].append(raw_speed_deficit)
      values["effectiveTargetReductionMps"].append(target_reduction)
      if experimental:
        counts["experimentalFrames"] += 1
      else:
        counts["chillFrames"] += 1

      clear_road_catchup = (
        v_ego >= 5.0 and speed_deficit >= 2.0 and not lead_present and not stop_context and
        not bool(safe_attr(car_state, "gasPressed", False)) and
        not bool(safe_attr(car_state, "brakePressed", False))
      )
      if not clear_road_catchup:
        continue

      counts["clearRoadCatchupFrames"] += 1
      if experimental:
        counts["clearRoadCatchupExperimentalFrames"] += 1
      else:
        counts["clearRoadCatchupChillFrames"] += 1
      if target_reduction > 0.5:
        counts["effectiveTargetLimitedFrames"] += 1
      if plan_accel < 0.20:
        counts["weakPlannerAccelFrames"] += 1
      if max_accel > 0.05 and plan_accel >= 0.90 * max_accel:
        counts["accelCeilingLimitedFrames"] += 1

      values["catchupSpeedDeficitMps"].append(speed_deficit)
      values["catchupPlanAccel"].append(plan_accel)
      values["catchupCommandAccel"].append(cmd_accel)
      values["catchupModelAccel"].append(model_accel)
      values["catchupMaxAccel"].append(max_accel)
      if max_accel > 0.05:
        values["catchupCeilingUtilization"].append(plan_accel / max_accel)

      if len(examples) < 25 and (plan_accel < 0.20 or target_reduction > 0.5):
        examples.append({
          "segment": segment,
          "logMonoTime": int(msg.logMonoTime),
          "experimental": experimental,
          "vEgoMps": round(v_ego, 3),
          "rawVCruiseMps": round(raw_v_cruise, 3),
          "effectiveVCruiseMps": round(effective_v_cruise, 3),
          "speedDeficitMps": round(speed_deficit, 3),
          "targetReductionMps": round(target_reduction, 3),
          "planAccel": round(plan_accel, 3),
          "commandAccel": round(cmd_accel, 3),
          "modelAccel": round(model_accel, 3),
          "maxAccel": round(max_accel, 3),
          "source": str(safe_attr(long_plan, "longitudinalPlanSource", "unknown")),
        })

  catchup = counts["clearRoadCatchupFrames"]
  return {
    "route": route,
    "segments": len(paths),
    "software": software,
    "startupParams": startup_params,
    "mode": {
      "longActiveFrames": counts["longActiveFrames"],
      "experimentalPct": percent(counts["experimentalFrames"], counts["longActiveFrames"]),
      "chillPct": percent(counts["chillFrames"], counts["longActiveFrames"]),
    },
    "clearRoadCatchup": {
      "frames": catchup,
      "experimentalPct": percent(counts["clearRoadCatchupExperimentalFrames"], catchup),
      "chillPct": percent(counts["clearRoadCatchupChillFrames"], catchup),
      "effectiveTargetLimitedPct": percent(counts["effectiveTargetLimitedFrames"], catchup),
      "weakPlannerAccelPct": percent(counts["weakPlannerAccelFrames"], catchup),
      "accelCeilingLimitedPct": percent(counts["accelCeilingLimitedFrames"], catchup),
      "speedDeficitMps": stats(values["catchupSpeedDeficitMps"]),
      "planAccel": stats(values["catchupPlanAccel"]),
      "commandAccel": stats(values["catchupCommandAccel"]),
      "modelAccel": stats(values["catchupModelAccel"]),
      "maxAccel": stats(values["catchupMaxAccel"]),
      "ceilingUtilization": stats(values["catchupCeilingUtilization"]),
      "examples": examples,
    },
    "allLongActive": {
      "speedDeficitMps": stats(values["speedDeficitMps"]),
      "rawSpeedDeficitMps": stats(values["rawSpeedDeficitMps"]),
      "effectiveTargetReductionMps": stats(values["effectiveTargetReductionMps"]),
    },
  }


def main() -> None:
  parser = argparse.ArgumentParser(description="Explain mode and acceleration ownership in recorded drives")
  parser.add_argument("paths", nargs="+", type=Path)
  parser.add_argument("--output", type=Path)
  args = parser.parse_args()

  reports = [analyze_route(route, paths) for route, paths in discover(args.paths).items()]
  rendered = json.dumps(reports, indent=2, sort_keys=True)
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
  print(rendered)


if __name__ == "__main__":
  main()
