#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
from openpilot.tools.lib.logreader import LogReader, ReadMode


MODEL_TICKS = 5
MIN_SPEED = 5.0
CURVE_LAT_ACCEL = 0.3
DRIVER_TORQUE = 50.0
DRIVER_ERROR = 0.15
LOOKAHEAD_MIN = 10.0
LOOKAHEAD_MAX = 35.0


def safe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def safe_float(value: Any, default: float = 0.0) -> float:
  try:
    value = float(value)
    return value if math.isfinite(value) else default
  except Exception:
    return default


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

  # Keep one copy of duplicate archived segments, preferring the first CLI root.
  unique: dict[tuple[str, int], Path] = {}
  for path in candidates:
    route, segment = route_and_segment(path)
    unique.setdefault((route, segment), path)

  routes: dict[str, list[Path]] = defaultdict(list)
  for (route, _), path in unique.items():
    routes[route].append(path)
  for route in routes:
    routes[route].sort(key=lambda p: route_and_segment(p)[1])
  return dict(sorted(routes.items()))


def decode_param(value: Any) -> str:
  try:
    return bytes(value).decode("utf-8", errors="replace")
  except Exception:
    return str(value)


def init_metadata(init_data: Any) -> dict[str, Any]:
  params: dict[str, str] = {}
  try:
    for entry in init_data.params.entries:
      params[str(entry.key)] = decode_param(entry.value)
  except Exception:
    pass
  return {
    "gitCommit": str(safe_attr(init_data, "gitCommit", "unknown")),
    "gitBranch": str(safe_attr(init_data, "gitBranch", "unknown")),
    "gitRemote": str(safe_attr(init_data, "gitRemote", "unknown")),
    "carModel": params.get("CarModel", "unknown"),
    "laneCentering": params.get("LaneCentering", "unknown"),
    "laneCenterOffset": params.get("LaneCenterOffset", "unknown"),
    "laneCenteringE2EAuthority": params.get("LaneCenteringE2EAuthority", "unknown"),
    "laneCenteringPauseOnSignal": params.get("LaneCenteringPauseOnSignal", "unknown"),
    "model": params.get("ModelName", params.get("Model", params.get("DrivingModel", "unknown"))),
  }


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  return float(np.percentile(np.asarray(values, dtype=float), pct))


def stats(values: list[float]) -> dict[str, float | int | None]:
  if not values:
    return {"count": 0, "p05": None, "p50": None, "p90": None, "p95": None, "max": None}
  return {
    "count": len(values),
    "p05": round(percentile(values, 5) or 0.0, 5),
    "p50": round(percentile(values, 50) or 0.0, 5),
    "p90": round(percentile(values, 90) or 0.0, 5),
    "p95": round(percentile(values, 95) or 0.0, 5),
    "max": round(max(values), 5),
  }


def pct(count: int, total: int) -> float:
  return round(100.0 * count / total, 2) if total else 0.0


def lane_geometry(model: Any, v_ego: float) -> tuple[bool, float, float, float, float, float, float, float]:
  try:
    lines = model.laneLines
    probs = np.asarray(model.laneLineProbs, dtype=float)
    stds = np.asarray(model.laneLineStds, dtype=float)
    left_x = np.asarray(lines[1].x, dtype=float)
    left_y = np.asarray(lines[1].y, dtype=float)
    right_x = np.asarray(lines[2].x, dtype=float)
    right_y = np.asarray(lines[2].y, dtype=float)
    pos_x = np.asarray(model.position.x, dtype=float)
    pos_y = np.asarray(model.position.y, dtype=float)
    lookahead = float(np.clip(v_ego, LOOKAHEAD_MIN, LOOKAHEAD_MAX))
    lookahead_near = max(6.0, lookahead * 0.45)
    lookaheads = np.linspace(lookahead_near, lookahead, 5)
    arrays = (left_x, left_y, right_x, right_y, pos_x, pos_y)
    valid = all(a.size >= 2 and np.isfinite(a).all() for a in arrays)
    valid = valid and all(x[0] <= lookahead <= x[-1] and np.all(np.diff(x) > 0) for x in (left_x, right_x, pos_x))
    if not valid:
      return False, 0.0, 0.0, 0.0, lookahead, 0.0, 0.0, 0.0
    left = np.interp(lookaheads, left_x, left_y)
    right = np.interp(lookaheads, right_x, right_y)
    model_y = np.interp(lookaheads, pos_x, pos_y)
    widths = right - left
    width = float(widths[-1])
    confidence = float(min(probs[1], probs[2]))
    lane_std = float(max(stds[1], stds[2]))
    center_errors = 0.5 * (left + right) - model_y
    errors = np.copysign(np.maximum(np.abs(center_errors) - 0.08, 0.0), center_errors)
    terms = np.linspace(0.7, 1.0, 5) * lookaheads ** 2 * errors
    term_magnitude = float(np.sum(np.abs(terms)))
    coherence = 1.0 if term_magnitude < 1e-9 else float(abs(np.sum(terms)) / term_magnitude)
    width_spread = float(np.max(widths) - np.min(widths))
    strict = bool(
      np.all((2.6 <= widths) & (widths <= 4.8))
      and width_spread <= 0.45
      and confidence >= 0.7
      and 0.0 <= lane_std <= 0.25
    )
    return strict, float(center_errors[-1]), width, confidence, lookahead, lane_std, width_spread, coherence
  except Exception:
    return False, 0.0, 0.0, 0.0, float(np.clip(v_ego, LOOKAHEAD_MIN, LOOKAHEAD_MAX)), 0.0, 0.0, 0.0


def new_buckets() -> dict[str, list[float]]:
  return defaultdict(list)


def analyze_route(route: str, paths: list[Path], road_aware: bool = False, road_edge_offset: float = 0.15) -> dict[str, Any]:
  controller = LaneCenteringController()
  centered_reference = LaneCenteringController() if road_aware else None
  latest: dict[str, Any] = {}
  metadata: dict[str, Any] = {}
  values = new_buckets()
  counts: dict[str, int] = defaultdict(int)
  by_speed: dict[str, dict[str, list[float]]] = defaultdict(new_buckets)
  by_side: dict[str, dict[str, list[float]]] = defaultdict(new_buckets)
  prev_correction: float | None = None
  prev_frame_id: int | None = None
  source_counts: dict[str, int] = defaultdict(int)

  for path in paths:
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
      which = msg.which()
      if which == "initData" and not metadata:
        metadata = init_metadata(msg.initData)
        continue
      if which in ("carState", "carControl", "controlsState"):
        latest[which] = getattr(msg, which)
        continue
      if which != "modelV2":
        continue

      model = msg.modelV2
      frame_id = int(safe_attr(model, "frameId", 0))
      if prev_frame_id == frame_id:
        continue
      prev_frame_id = frame_id
      counts["modelFrames"] += 1

      car_state = latest.get("carState")
      car_control = latest.get("carControl")
      controls_state = latest.get("controlsState")
      if car_state is None or car_control is None:
        counts["missingStateFrames"] += 1
        continue

      v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
      lat_active = bool(safe_attr(car_control, "latActive", False))
      steering_pressed = bool(safe_attr(car_state, "steeringPressed", False))
      steering_torque = safe_float(safe_attr(car_state, "steeringTorque", 0.0))
      signal = bool(safe_attr(car_state, "leftBlinker", False) or safe_attr(car_state, "rightBlinker", False))
      try:
        lane_change = int(model.meta.laneChangeState)
      except Exception:
        lane_change = 0

      action = safe_attr(model, "action")
      raw_action_curvature = safe_attr(action, "desiredCurvature")
      if raw_action_curvature is not None:
        model_curvature = safe_float(raw_action_curvature)
        source_counts["modelAction"] += 1
      else:
        model_curvature = safe_float(safe_attr(controls_state, "desiredCurvature", 0.0))
        source_counts["controlsStateFallback"] += 1

      strict_geometry, center_error, width, confidence, lookahead, lane_std, width_spread, coherence = lane_geometry(
        model, v_ego,
      )
      raw_valid, raw_geometry_correction = controller._calculate_raw_correction(
        model, v_ego, model_curvature, 0.0, 1.0,
      )
      base_eligible = lat_active and v_ego >= MIN_SPEED and lane_change == 0 and not signal
      if base_eligible:
        counts["baseEligibleFrames"] += 1
        if strict_geometry:
          counts["strictGeometryFrames"] += 1

      output = model_curvature
      centered_output = model_curvature
      for _ in range(MODEL_TICKS):
        output = controller.update(model_curvature, model, v_ego, True, 0.0, 1.0, lat_active, True,
                                   True, signal, steering_pressed, road_aware, road_edge_offset)
        if centered_reference is not None:
          centered_output = centered_reference.update(
            model_curvature, model, v_ego, True, 0.0, 1.0, lat_active, True,
            True, signal, steering_pressed, False, road_edge_offset,
          )
      correction = output - model_curvature
      centered_correction = centered_output - model_curvature

      if not (base_eligible and strict_geometry):
        if base_eligible and prev_correction is not None:
          values["releaseStepLatAccel"].append(abs(correction - prev_correction) * v_ego ** 2)
        prev_correction = correction
        continue

      counts["eligibleFrames"] += 1
      if steering_pressed:
        counts["driverPressedEligibleFrames"] += 1
      else:
        counts["autonomousEligibleFrames"] += 1

      residual = center_error - correction * lookahead ** 2 / 2.0
      centered_residual = center_error - centered_correction * lookahead ** 2 / 2.0
      topology_bias = controller._effective_road_topology_bias if road_aware else 0.0
      topology_state = controller._road_topology_state if road_aware else 0
      target_residual = center_error + topology_bias - correction * lookahead ** 2 / 2.0
      extra_shift = (correction - centered_correction) * lookahead ** 2 / 2.0
      half_width = width * 0.5
      baseline_shift = centered_correction * lookahead ** 2 / 2.0
      adaptive_shift = correction * lookahead ** 2 / 2.0
      baseline_boundary_clearance = min(half_width - center_error + baseline_shift, half_width + center_error - baseline_shift)
      adaptive_boundary_clearance = min(half_width - center_error + adaptive_shift, half_width + center_error - adaptive_shift)
      raw_abs = abs(center_error)
      residual_abs = abs(residual)
      added_lat_accel = abs(correction) * v_ego ** 2
      total_lat_accel = abs(output) * v_ego ** 2
      curve = abs(model_curvature) * v_ego ** 2 >= CURVE_LAT_ACCEL

      if not steering_pressed:
        values["rawError"].append(raw_abs)
        values["residualError"].append(residual_abs)
        values["centeredReferenceResidual"].append(abs(centered_residual))
        values["topologyTargetResidual"].append(abs(target_residual))
        values["topologyBias"].append(abs(topology_bias))
        values["baselineBoundaryClearance"].append(baseline_boundary_clearance)
        values["adaptiveBoundaryClearance"].append(adaptive_boundary_clearance)
        if topology_state:
          counts["topologyOuterFrames"] += 1
          counts["topologyLeftFrames" if topology_state < 0 else "topologyRightFrames"] += 1
          values["adjacentClearanceGain"].append(topology_state * extra_shift)
          if topology_state * extra_shift < -0.005:
            counts["topologyAlignmentViolationFrames"] += 1
        values["addedLatAccel"].append(added_lat_accel)
        values["totalLatAccel"].append(total_lat_accel)
        values["laneWidth"].append(width)
        values["laneConfidence"].append(confidence)
        values["laneStd"].append(lane_std)
        values["laneWidthSpread"].append(width_spread)
        values["correctionCoherence"].append(coherence)
        if residual_abs > raw_abs + 1e-6:
          counts["worsenedFrames"] += 1
          values["worseningM"].append(residual_abs - raw_abs)
        if residual_abs > raw_abs + 0.02:
          counts["worsenedOver2cmFrames"] += 1
        if residual_abs > raw_abs + 0.05:
          counts["worsenedOver5cmFrames"] += 1
        if correction * center_error < 0.0:
          counts["wrongDirectionFrames"] += 1
        if raw_abs > 0.2:
          counts["rawOver20cm"] += 1
        if residual_abs > 0.2:
          counts["residualOver20cm"] += 1
        if raw_abs > 0.3:
          counts["rawOver30cm"] += 1
        if residual_abs > 0.3:
          counts["residualOver30cm"] += 1
        if added_lat_accel >= 0.594:
          counts["saturatedFrames"] += 1
        if curve:
          counts["autonomousCurveFrames"] += 1
          values["curveRawError"].append(raw_abs)
          values["curveResidualError"].append(residual_abs)
          values["curveTargetResidual"].append(abs(target_residual))
          values["curveBaselineBoundaryClearance"].append(baseline_boundary_clearance)
          values["curveAdaptiveBoundaryClearance"].append(adaptive_boundary_clearance)
          values["curveAddedLatAccel"].append(added_lat_accel)

        speed_band = "5-10mps" if v_ego < 10.0 else "10-20mps" if v_ego < 20.0 else "20mps+"
        by_speed[speed_band]["raw"].append(raw_abs)
        by_speed[speed_band]["residual"].append(residual_abs)
        path_side = "modelLeftOfCenter" if center_error > 0.0 else "modelRightOfCenter"
        by_side[path_side]["raw"].append(raw_abs)
        by_side[path_side]["residual"].append(residual_abs)

        if prev_correction is not None:
          values["correctionStepLatAccel"].append(abs(correction - prev_correction) * v_ego ** 2)

      if steering_pressed and raw_valid and abs(steering_torque) >= DRIVER_TORQUE and raw_abs >= DRIVER_ERROR:
        counts["driverComparisonFrames"] += 1
        if raw_geometry_correction * steering_torque < 0.0:
          counts["driverAgreementFrames"] += 1

      prev_correction = correction

  auto = counts["autonomousEligibleFrames"]
  driver = counts["driverComparisonFrames"]
  result = {
    "route": route,
    "segments": len(paths),
    "metadata": metadata,
    "curvatureSourceFrames": dict(source_counts),
    "coverage": {
      "modelFrames": counts["modelFrames"],
      "baseEligibleFrames": counts["baseEligibleFrames"],
      "strictGeometryFrames": counts["strictGeometryFrames"],
      "strictGeometryAvailabilityPct": pct(counts["strictGeometryFrames"], counts["baseEligibleFrames"]),
      "autonomousEligibleFrames": auto,
      "autonomousCurveFrames": counts["autonomousCurveFrames"],
      "driverPressedEligibleFrames": counts["driverPressedEligibleFrames"],
    },
    "centering": {
      "rawAbsM": stats(values["rawError"]),
      "projectedResidualAbsM": stats(values["residualError"]),
      "rawOver20cmPct": pct(counts["rawOver20cm"], auto),
      "projectedOver20cmPct": pct(counts["residualOver20cm"], auto),
      "rawOver30cmPct": pct(counts["rawOver30cm"], auto),
      "projectedOver30cmPct": pct(counts["residualOver30cm"], auto),
      "worsenedPct": pct(counts["worsenedFrames"], auto),
      "worsenedOver2cmPct": pct(counts["worsenedOver2cmFrames"], auto),
      "worsenedOver5cmPct": pct(counts["worsenedOver5cmFrames"], auto),
      "wrongDirectionPct": pct(counts["wrongDirectionFrames"], auto),
      "worseningM": stats(values["worseningM"]),
    },
    "curves": {
      "rawAbsM": stats(values["curveRawError"]),
      "projectedResidualAbsM": stats(values["curveResidualError"]),
      "addedLatAccel": stats(values["curveAddedLatAccel"]),
    },
    "demand": {
      "addedLatAccel": stats(values["addedLatAccel"]),
      "totalDesiredLatAccel": stats(values["totalLatAccel"]),
      "correctionStepLatAccel": stats(values["correctionStepLatAccel"]),
      "confidenceReleaseStepLatAccel": stats(values["releaseStepLatAccel"]),
      "saturatedPct": pct(counts["saturatedFrames"], auto),
    },
    "geometry": {
      "laneWidthM": stats(values["laneWidth"]),
      "laneConfidence": stats(values["laneConfidence"]),
      "laneConfidenceP10": round(percentile(values["laneConfidence"], 10) or 0.0, 5),
      "laneStd": stats(values["laneStd"]),
      "laneWidthSpreadM": stats(values["laneWidthSpread"]),
      "correctionCoherence": stats(values["correctionCoherence"]),
      "correctionCoherenceP10": round(percentile(values["correctionCoherence"], 10) or 0.0, 5),
    },
    "driverComparison": {
      "frames": driver,
      "agreementPct": pct(counts["driverAgreementFrames"], driver),
    },
    "roadPositioning": {
      "enabled": road_aware,
      "configuredOffsetM": road_edge_offset,
      "outerFrames": counts["topologyOuterFrames"],
      "leftFrames": counts["topologyLeftFrames"],
      "rightFrames": counts["topologyRightFrames"],
      "outerAvailabilityPct": pct(counts["topologyOuterFrames"], auto),
      "controllerAlignmentViolationPct": pct(counts["topologyAlignmentViolationFrames"], counts["topologyOuterFrames"]),
      "controllerAlignmentMeaning": "Checks only that output follows the inferred topology; it does not validate the topology classification.",
      "groundTruthValidated": False,
      "classificationVerdict": "unverified-no-labelled-road-topology",
      "releaseEligible": False,
      "biasAbsM": stats(values["topologyBias"]),
      "targetResidualAbsM": stats(values["topologyTargetResidual"]),
      "centeredReferenceResidualAbsM": stats(values["centeredReferenceResidual"]),
      "adjacentClearanceGainM": stats(values["adjacentClearanceGain"]),
      "baselineBoundaryClearanceM": stats(values["baselineBoundaryClearance"]),
      "adaptiveBoundaryClearanceM": stats(values["adaptiveBoundaryClearance"]),
      "curveTargetResidualAbsM": stats(values["curveTargetResidual"]),
      "curveBaselineBoundaryClearanceM": stats(values["curveBaselineBoundaryClearance"]),
      "curveAdaptiveBoundaryClearanceM": stats(values["curveAdaptiveBoundaryClearance"]),
    },
    "speedBands": {
      name: {"frames": len(bucket["raw"]), "rawP95M": round(percentile(bucket["raw"], 95) or 0.0, 4),
             "projectedP95M": round(percentile(bucket["residual"], 95) or 0.0, 4)}
      for name, bucket in sorted(by_speed.items())
    },
    "pathSide": {
      name: {"frames": len(bucket["raw"]), "rawP95M": round(percentile(bucket["raw"], 95) or 0.0, 4),
             "projectedP95M": round(percentile(bucket["residual"], 95) or 0.0, 4)}
      for name, bucket in sorted(by_side.items())
    },
  }
  return result


def main() -> None:
  parser = argparse.ArgumentParser(description="Replay the current lane-envelope controller over historical rlogs")
  parser.add_argument("paths", nargs="+", type=Path)
  parser.add_argument("--json", action="store_true", help="emit the complete machine-readable report")
  parser.add_argument("--road-aware", action="store_true", help="enable topology-aware outer-lane positioning")
  parser.add_argument("--road-edge-offset", type=float, default=0.15)
  args = parser.parse_args()
  routes = discover_logs(args.paths)
  reports = [analyze_route(route, paths, args.road_aware, args.road_edge_offset) for route, paths in routes.items()]
  if args.json:
    print(json.dumps(reports, indent=2, sort_keys=True))
    return

  for report in reports:
    coverage = report["coverage"]
    centering = report["centering"]
    curves = report["curves"]
    demand = report["demand"]
    geometry = report["geometry"]
    driver = report["driverComparison"]
    road_positioning = report["roadPositioning"]
    fields = [
      f'{report["route"]} seg={report["segments"]}',
      f'commit={report["metadata"].get("gitCommit", "unknown")[:9]}',
      f'LC={report["metadata"].get("laneCentering", "?")}',
      f'model={report["metadata"].get("model", "?")}',
      f'eligible={coverage["autonomousEligibleFrames"]}',
      f'avail={coverage["strictGeometryAvailabilityPct"]:.1f}%',
      f'p95={centering["rawAbsM"]["p95"]}->{centering["projectedResidualAbsM"]["p95"]}m',
      f'curve_p95={curves["rawAbsM"]["p95"]}->{curves["projectedResidualAbsM"]["p95"]}m',
      f'worse={centering["worsenedPct"]:.1f}%',
      f'worse2cm={centering["worsenedOver2cmPct"]:.1f}%',
      f'worse5cm={centering["worsenedOver5cmPct"]:.1f}%',
      f'wrongDir={centering["wrongDirectionPct"]:.1f}%',
      f'sat={demand["saturatedPct"]:.1f}%',
      f'addP95={demand["addedLatAccel"]["p95"]}',
      f'stepMax={demand["correctionStepLatAccel"]["max"]}',
      f'releaseMax={demand["confidenceReleaseStepLatAccel"]["max"]}',
      f'probP10={geometry["laneConfidenceP10"]}',
      f'stdP95={geometry["laneStd"]["p95"]}',
      f'widthSpreadP95={geometry["laneWidthSpreadM"]["p95"]}',
      f'coherenceP10={geometry["correctionCoherenceP10"]}',
      f'driverProxy={driver["agreementPct"]:.1f}%/{driver["frames"]}',
    ]
    if road_positioning["enabled"]:
      fields.extend([
        f'outer={road_positioning["outerAvailabilityPct"]:.1f}%',
        f'outerL/R={road_positioning["leftFrames"]}/{road_positioning["rightFrames"]}',
        f'biasP95={road_positioning["biasAbsM"]["p95"]}',
        f'targetP95={road_positioning["targetResidualAbsM"]["p95"]}',
        f'adjGainP50={road_positioning["adjacentClearanceGainM"]["p50"]}',
        f'edgeP50={road_positioning["baselineBoundaryClearanceM"]["p50"]}/{road_positioning["adaptiveBoundaryClearanceM"]["p50"]}',
        f'edgeP05={road_positioning["baselineBoundaryClearanceM"]["p05"]}/{road_positioning["adaptiveBoundaryClearanceM"]["p05"]}',
        f'curveEdgeP05={road_positioning["curveBaselineBoundaryClearanceM"]["p05"]}/{road_positioning["curveAdaptiveBoundaryClearanceM"]["p05"]}',
        f'controllerAlignViolation={road_positioning["controllerAlignmentViolationPct"]:.1f}%',
        f'topologyVerdict={road_positioning["classificationVerdict"]}',
      ])
    print(" ".join(fields))


if __name__ == "__main__":
  main()
