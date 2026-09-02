#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from openpilot.selfdrive.controls.lib.lane_centering import get_raw_lane_centering_correction
from openpilot.tools.lib.logreader import LogReader, ReadMode


MIN_SPEED = 5.0
MIN_CURVE_LAT_ACCEL = 0.30
MIN_LANE_PROB = 0.70
MAX_LANE_STD = 0.25
MIN_LANE_WIDTH = 2.60
MAX_LANE_WIDTH = 4.80
MAX_LANE_WIDTH_SPREAD = 0.45
LOOKAHEAD_MIN = 10.0
LOOKAHEAD_MAX = 35.0


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


def route_and_segment(path: Path) -> tuple[str, int]:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 2:
      try:
        segment = int(parts[2].split(".", 1)[0]) if len(parts) >= 3 else 0
      except ValueError:
        segment = 0
      return "--".join(parts[:2]), segment
  return path.parent.name, 0


def discover_logs(paths: list[Path]) -> dict[str, list[Path]]:
  candidates: list[Path] = []
  for path in paths:
    if path.is_dir():
      candidates.extend(path.rglob("rlog.zst"))
    elif path.name == "rlog.zst" and path.exists():
      candidates.append(path)

  unique: dict[tuple[str, int], Path] = {}
  for path in candidates:
    unique.setdefault(route_and_segment(path), path)

  routes: dict[str, list[Path]] = defaultdict(list)
  for (route, _), path in unique.items():
    routes[route].append(path)
  for route in routes:
    routes[route].sort(key=lambda path: route_and_segment(path)[1])
  return dict(sorted(routes.items()))


def percentile(values: list[float], pct_value: float) -> float | None:
  if not values:
    return None
  return float(np.percentile(np.asarray(values, dtype=float), pct_value))


def stats(values: list[float]) -> dict[str, float | int | None]:
  if not values:
    return {"count": 0, "mean": None, "p05": None, "p50": None, "p95": None}
  return {
    "count": len(values),
    "mean": round(float(np.mean(values)), 5),
    "p05": round(percentile(values, 5) or 0.0, 5),
    "p50": round(percentile(values, 50) or 0.0, 5),
    "p95": round(percentile(values, 95) or 0.0, 5),
  }


def pct(count: int, total: int) -> float:
  return round(100.0 * count / total, 2) if total else 0.0


def valid_path(x: np.ndarray, y: np.ndarray) -> bool:
  return bool(x.size >= 2 and x.size == y.size and np.isfinite(x).all() and
              np.isfinite(y).all() and np.all(np.diff(x) > 0.0))


def extract_geometry(model: Any, v_ego: float) -> dict[str, float] | None:
  try:
    lines = model.laneLines
    probs = np.asarray(model.laneLineProbs, dtype=float)
    stds = np.asarray(model.laneLineStds, dtype=float)
    if len(lines) < 3 or probs.size < 3 or stds.size < 3:
      return None
    if (not np.isfinite(probs[[1, 2]]).all() or not np.isfinite(stds[[1, 2]]).all() or
        np.any(probs[[1, 2]] < MIN_LANE_PROB) or np.any(stds[[1, 2]] > MAX_LANE_STD)):
      return None

    left_x = np.asarray(lines[1].x, dtype=float)
    left_y = np.asarray(lines[1].y, dtype=float)
    right_x = np.asarray(lines[2].x, dtype=float)
    right_y = np.asarray(lines[2].y, dtype=float)
    pos_x = np.asarray(model.position.x, dtype=float)
    pos_y = np.asarray(model.position.y, dtype=float)
    if not all((valid_path(left_x, left_y), valid_path(right_x, right_y), valid_path(pos_x, pos_y))):
      return None

    far = float(np.clip(v_ego, LOOKAHEAD_MIN, LOOKAHEAD_MAX))
    probes = np.asarray([0.0, max(6.0, 0.45 * far), far], dtype=float)
    if any(x[0] > 0.0 or x[-1] < far for x in (left_x, right_x, pos_x)):
      return None

    left = np.interp(probes, left_x, left_y)
    right = np.interp(probes, right_x, right_y)
    path = np.interp(probes, pos_x, pos_y)
    widths = right - left
    if (np.any(widths < MIN_LANE_WIDTH) or np.any(widths > MAX_LANE_WIDTH) or
        float(np.ptp(widths)) > MAX_LANE_WIDTH_SPREAD):
      return None

    centers = 0.5 * (left + right)
    path_std = np.asarray(safe_attr(model.position, "yStd", []), dtype=float)
    far_path_std = float(np.interp(far, pos_x, path_std)) if path_std.size == pos_x.size and np.isfinite(path_std).all() else math.inf
    return {
      "nearCenter": float(centers[0]),
      "middleCenter": float(centers[1]),
      "farCenter": float(centers[2]),
      "middlePath": float(path[1]),
      "farPath": float(path[2]),
      "farDistance": far,
      "farPathStd": far_path_std,
      "laneWidth": float(widths[0]),
      "laneConfidence": float(min(probs[1], probs[2])),
      "laneStd": float(max(stds[1], stds[2])),
    }
  except (AttributeError, IndexError, TypeError, ValueError):
    return None


def analyze_route(route: str, paths: list[Path]) -> dict[str, Any]:
  latest: dict[str, Any] = {}
  values: dict[str, list[float]] = defaultdict(list)
  by_direction: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
  counts: dict[str, int] = defaultdict(int)
  top_inside: list[dict[str, Any]] = []
  previous_frame_id: int | None = None

  for path in paths:
    segment = route_and_segment(path)[1]
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
      which = msg.which()
      if which in ("carState", "carControl", "controlsState"):
        latest[which] = getattr(msg, which)
        continue
      if which != "modelV2":
        continue

      model = msg.modelV2
      frame_id = int(safe_attr(model, "frameId", 0))
      if frame_id == previous_frame_id:
        continue
      previous_frame_id = frame_id
      counts["modelFrames"] += 1

      car_state = latest.get("carState")
      car_control = latest.get("carControl")
      controls_state = latest.get("controlsState")
      if car_state is None or car_control is None or controls_state is None:
        continue

      v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
      lat_active = bool(safe_attr(car_control, "latActive", False))
      steering_pressed = bool(safe_attr(car_state, "steeringPressed", False))
      signal = bool(safe_attr(car_state, "leftBlinker", False) or safe_attr(car_state, "rightBlinker", False))
      try:
        lane_change = int(model.meta.laneChangeState)
      except Exception:
        lane_change = 0
      if not lat_active or steering_pressed or signal or lane_change != 0 or v_ego < MIN_SPEED:
        continue
      counts["autonomousFrames"] += 1

      geometry = extract_geometry(model, v_ego)
      if geometry is None:
        continue
      counts["strictGeometryFrames"] += 1

      model_curvature = safe_float(safe_attr(safe_attr(model, "action"), "desiredCurvature", 0.0))
      desired_curvature = safe_float(safe_attr(controls_state, "desiredCurvature", model_curvature))
      actual_curvature = safe_float(safe_attr(controls_state, "curvature", 0.0))
      model_lat_accel = model_curvature * v_ego * v_ego
      ego_right = -geometry["nearCenter"]
      values["allEgoRightM"].append(ego_right)
      if abs(model_lat_accel) < MIN_CURVE_LAT_ACCEL:
        values["straightEgoRightM"].append(ego_right)
        continue
      counts["curveFrames"] += 1

      direction_sign = 1.0 if model_curvature > 0.0 else -1.0
      direction = "right" if direction_sign > 0.0 else "left"
      # Positive means toward the inside of the bend. At x=0 the model path is
      # ego-relative zero, so -lane center is the ego position relative to center.
      ego_inside = -direction_sign * geometry["nearCenter"]
      model_middle_inside = direction_sign * (geometry["middlePath"] - geometry["middleCenter"])
      model_far_inside = direction_sign * (geometry["farPath"] - geometry["farCenter"])
      desired_tracking_error = direction_sign * (desired_curvature - actual_curvature) * v_ego * v_ego
      model_tracking_error = direction_sign * (model_curvature - actual_curvature) * v_ego * v_ego
      raw_valid, raw_correction = get_raw_lane_centering_correction(model, v_ego, 0.0, 1.0)
      outward_correction = -direction_sign * raw_correction * v_ego * v_ego if raw_valid else 0.0

      sample_values = {
        "egoInsideM": ego_inside,
        "modelMiddleInsideM": model_middle_inside,
        "modelFarInsideM": model_far_inside,
        "desiredTrackingErrorLatAccel": desired_tracking_error,
        "modelTrackingErrorLatAccel": model_tracking_error,
        "outwardRawCorrectionLatAccel": outward_correction,
      }
      for name, value in sample_values.items():
        values[name].append(value)
        by_direction[direction][name].append(value)

      if ego_inside > 0.10:
        counts["egoInsideOver10cm"] += 1
      if ego_inside > 0.20:
        counts["egoInsideOver20cm"] += 1
      if model_far_inside > 0.10:
        counts["modelFarInsideOver10cm"] += 1

      if ego_inside > 0.10:
        top_inside.append({
          "segment": segment,
          "frameId": frame_id,
          "logMonoTime": int(msg.logMonoTime),
          "direction": direction,
          "vEgo": round(v_ego, 3),
          "egoInsideM": round(ego_inside, 4),
          "modelMiddleInsideM": round(model_middle_inside, 4),
          "modelFarInsideM": round(model_far_inside, 4),
          "modelLatAccel": round(model_lat_accel, 4),
          "desiredTrackingErrorLatAccel": round(desired_tracking_error, 4),
          "modelTrackingErrorLatAccel": round(model_tracking_error, 4),
          "outwardRawCorrectionLatAccel": round(outward_correction, 4),
          "laneWidth": round(geometry["laneWidth"], 4),
          "laneConfidence": round(geometry["laneConfidence"], 4),
          "laneStd": round(geometry["laneStd"], 4),
        })

  curve_frames = counts["curveFrames"]
  top_inside.sort(key=lambda item: item["egoInsideM"], reverse=True)
  return {
    "route": route,
    "segments": len(paths),
    "coverage": {
      "modelFrames": counts["modelFrames"],
      "autonomousFrames": counts["autonomousFrames"],
      "strictGeometryFrames": counts["strictGeometryFrames"],
      "curveFrames": curve_frames,
    },
    "insideBias": {
      "egoInsideM": stats(values["egoInsideM"]),
      "modelMiddleInsideM": stats(values["modelMiddleInsideM"]),
      "modelFarInsideM": stats(values["modelFarInsideM"]),
      "egoInsideOver10cmPct": pct(counts["egoInsideOver10cm"], curve_frames),
      "egoInsideOver20cmPct": pct(counts["egoInsideOver20cm"], curve_frames),
      "modelFarInsideOver10cmPct": pct(counts["modelFarInsideOver10cm"], curve_frames),
    },
    "alignment": {
      "allEgoRightM": stats(values["allEgoRightM"]),
      "straightEgoRightM": stats(values["straightEgoRightM"]),
    },
    "tracking": {
      "desiredTrackingErrorLatAccel": stats(values["desiredTrackingErrorLatAccel"]),
      "modelTrackingErrorLatAccel": stats(values["modelTrackingErrorLatAccel"]),
      "outwardRawCorrectionLatAccel": stats(values["outwardRawCorrectionLatAccel"]),
    },
    "directions": {
      direction: {name: stats(samples) for name, samples in buckets.items()}
      for direction, buckets in sorted(by_direction.items())
    },
    "largestInsideSamples": top_inside[:30],
  }


def main() -> None:
  parser = argparse.ArgumentParser(description="Separate curve inside bias from lateral tracking error")
  parser.add_argument("paths", nargs="+", type=Path)
  parser.add_argument("--output", type=Path)
  parser.add_argument("--json", action="store_true")
  args = parser.parse_args()

  routes = discover_logs(args.paths)
  reports = [analyze_route(route, paths) for route, paths in routes.items()]
  rendered = json.dumps(reports, indent=2, sort_keys=True)
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
  if args.json:
    print(rendered)
  else:
    for report in reports:
      bias = report["insideBias"]
      print(
        report["route"],
        f'curveFrames={report["coverage"]["curveFrames"]}',
        f'egoInsideP50={bias["egoInsideM"]["p50"]}',
        f'egoInsideP95={bias["egoInsideM"]["p95"]}',
        f'modelFarInsideP50={bias["modelFarInsideM"]["p50"]}',
        f'modelFarInsideP95={bias["modelFarInsideM"]["p95"]}',
        f'egoInsideOver10cm={bias["egoInsideOver10cmPct"]}%',
      )


if __name__ == "__main__":
  main()
