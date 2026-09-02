#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from openpilot.selfdrive.controls.lib import lane_centering
from openpilot.selfdrive.controls.lib.lane_centering import (
  LaneCenteringController,
  get_raw_lane_centering_correction,
)
from openpilot.tools.lib.logreader import LogReader, ReadMode


MODEL_TICKS = 5
MIN_SPEED = 5.0
CURVE_LAT_ACCEL = 0.3
DRIVER_TORQUE = 50.0
DRIVER_ERROR = 0.15
LOOKAHEAD_MIN = 10.0
LOOKAHEAD_MAX = 35.0
CORRECTION_SIGN_EPSILON = 2e-6
TARGET_REVERSAL_CAUSAL_WINDOW_FRAMES = 20


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
    "recordedLaneCenteringRoadAware": params.get("LaneCenteringRoadAware", "unknown"),
    "model": params.get("ModelName", params.get("Model", params.get("DrivingModel", "unknown"))),
  }


def percentile(values: list[float], pct_value: float) -> float | None:
  if not values:
    return None
  return float(np.percentile(np.asarray(values, dtype=float), pct_value))


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


def sign(value: float, epsilon: float = CORRECTION_SIGN_EPSILON) -> int:
  if value > epsilon:
    return 1
  if value < -epsilon:
    return -1
  return 0


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
    valid = all(array.size >= 2 and np.isfinite(array).all() for array in arrays)
    valid = valid and all(
      x[0] <= lookahead <= x[-1] and np.all(np.diff(x) > 0)
      for x in (left_x, right_x, pos_x)
    )
    if not valid:
      return False, 0.0, 0.0, 0.0, lookahead, 0.0, 0.0, 0.0

    left = np.interp(lookaheads, left_x, left_y)
    right = np.interp(lookaheads, right_x, right_y)
    model_y = np.interp(lookaheads, pos_x, pos_y)
    widths = right - left
    center_errors = 0.5 * (left + right) - model_y
    deadbanded = np.copysign(np.maximum(np.abs(center_errors) - 0.08, 0.0), center_errors)
    terms = np.linspace(0.7, 1.0, 5) * lookaheads ** 2 * deadbanded
    term_magnitude = float(np.sum(np.abs(terms)))
    coherence = 1.0 if term_magnitude < 1e-9 else float(abs(np.sum(terms)) / term_magnitude)
    confidence = float(min(probs[1], probs[2]))
    lane_std = float(max(stds[1], stds[2]))
    width_spread = float(np.ptp(widths))
    strict = bool(
      np.all((2.6 <= widths) & (widths <= 4.8))
      and width_spread <= 0.45
      and confidence >= 0.70
      and 0.0 <= lane_std <= 0.25
    )
    return (
      strict,
      float(center_errors[-1]),
      float(widths[-1]),
      confidence,
      lookahead,
      lane_std,
      width_spread,
      coherence,
    )
  except Exception:
    return False, 0.0, 0.0, 0.0, float(np.clip(v_ego, LOOKAHEAD_MIN, LOOKAHEAD_MAX)), 0.0, 0.0, 0.0


def new_buckets() -> dict[str, list[float]]:
  return defaultdict(list)


def configured_center_gain() -> float:
  return float(getattr(lane_centering, "_CENTER_GAIN", getattr(lane_centering, "_MAX_GAIN", 0.0)))


def analyze_route(route: str, paths: list[Path], e2e_authority: float | None = 0.15,
                  curve_adaptive: bool = False) -> dict[str, Any]:
  controller = LaneCenteringController(curve_adaptive=curve_adaptive)
  latest: dict[str, Any] = {}
  metadata: dict[str, Any] = {}
  values = new_buckets()
  counts: dict[str, int] = defaultdict(int)
  by_speed: dict[str, dict[str, list[float]]] = defaultdict(new_buckets)
  by_side: dict[str, dict[str, list[float]]] = defaultdict(new_buckets)
  source_counts: dict[str, int] = defaultdict(int)
  max_step_sample: dict[str, Any] | None = None
  previous_context: dict[str, Any] | None = None
  previous_segment: int | None = None
  previous_frame_id: int | None = None
  previous_correction: float | None = None
  previous_correction_sign = 0
  previous_target_sign = 0
  target_reversal_age: int | None = None
  pending_target_sign = 0
  reversal_samples: list[dict[str, Any]] = []
  introduced_clearance_samples: list[dict[str, Any]] = []

  for path in paths:
    segment = route_and_segment(path)[1]
    if previous_segment is not None and segment != previous_segment + 1:
      controller.reset()
      previous_frame_id = None
      previous_correction = None
      previous_correction_sign = 0
      previous_target_sign = 0
      target_reversal_age = None
      pending_target_sign = 0
      previous_context = None
    previous_segment = segment

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
      which = msg.which()
      if which == "initData" and not metadata:
        metadata = init_metadata(msg.initData)
        if e2e_authority is None:
          e2e_authority = safe_float(metadata.get("laneCenteringE2EAuthority"), 0.15)
        continue
      if which in ("carState", "carControl", "controlsState"):
        latest[which] = getattr(msg, which)
        continue
      if which != "modelV2":
        continue

      model = msg.modelV2
      frame_id = int(safe_attr(model, "frameId", 0))
      if previous_frame_id == frame_id:
        continue
      previous_frame_id = frame_id
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

      geometry = lane_geometry(model, v_ego)
      strict_geometry, center_error, width, confidence, lookahead, lane_std, width_spread, coherence = geometry
      response_deadband, _, _ = lane_centering.get_lane_centering_response(
        model_curvature, v_ego, curve_adaptive,
      )
      raw_valid, raw_correction = get_raw_lane_centering_correction(
        model, v_ego, 0.0, float(e2e_authority), response_deadband,
      )
      base_eligible = lat_active and v_ego >= MIN_SPEED and lane_change == 0 and not signal
      if base_eligible:
        counts["baseEligibleFrames"] += 1
        if strict_geometry and raw_valid:
          counts["strictGeometryFrames"] += 1

      output = model_curvature
      for _ in range(MODEL_TICKS):
        output = controller.update(
          model_curvature, model, v_ego, True, 0.0, float(e2e_authority), lat_active, True,
          True, signal,
        )
      correction = output - model_curvature

      if not (base_eligible and strict_geometry and raw_valid):
        if base_eligible and previous_correction is not None:
          values["releaseStepLatAccel"].append(abs(correction - previous_correction) * v_ego ** 2)
        previous_correction = correction
        previous_correction_sign = 0
        previous_target_sign = 0
        target_reversal_age = None
        pending_target_sign = 0
        continue

      counts["eligibleFrames"] += 1
      if steering_pressed:
        counts["driverPressedEligibleFrames"] += 1
      else:
        counts["autonomousEligibleFrames"] += 1

      projected_shift = correction * lookahead ** 2 / 2.0
      residual = center_error - projected_shift
      raw_abs = abs(center_error)
      residual_abs = abs(residual)
      baseline_clearance = width * 0.5 - raw_abs
      projected_clearance = width * 0.5 - residual_abs
      added_lat_accel = abs(correction) * v_ego ** 2
      total_lat_accel = abs(output) * v_ego ** 2
      curve = abs(model_curvature) * v_ego ** 2 >= CURVE_LAT_ACCEL

      if not steering_pressed:
        values["rawError"].append(raw_abs)
        values["residualError"].append(residual_abs)
        values["baselineClearance"].append(baseline_clearance)
        values["projectedClearance"].append(projected_clearance)
        values["addedLatAccel"].append(added_lat_accel)
        values["totalLatAccel"].append(total_lat_accel)
        values["laneWidth"].append(width)
        values["laneConfidence"].append(confidence)
        values["laneStd"].append(lane_std)
        values["laneWidthSpread"].append(width_spread)
        values["correctionCoherence"].append(coherence)

        if baseline_clearance < 1.4:
          counts["baselineClearanceViolationFrames"] += 1
        if projected_clearance < 1.4:
          counts["projectedClearanceViolationFrames"] += 1
          if baseline_clearance >= 1.4:
            counts["introducedClearanceViolationFrames"] += 1
            values["introducedClearanceDeficit"].append(1.4 - projected_clearance)
            if len(introduced_clearance_samples) < 20:
              introduced_clearance_samples.append({
                "segment": segment,
                "frameId": frame_id,
                "logMonoTime": int(safe_attr(msg, "logMonoTime", 0)),
                "vEgo": round(v_ego, 5),
                "baselineClearance": round(baseline_clearance, 6),
                "projectedClearance": round(projected_clearance, 6),
                "centerError": round(center_error, 6),
                "correction": round(correction, 9),
                "rawCorrection": round(raw_correction, 9),
              })
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

        correction_sign = sign(correction)
        target_sign = sign(raw_correction)
        if target_reversal_age is not None:
          target_reversal_age += 1
        if target_sign and previous_target_sign and target_sign != previous_target_sign:
          counts["targetReversalFrames"] += 1
          target_reversal_age = 0
        if correction_sign and target_sign and correction_sign != target_sign:
          pending_target_sign = target_sign
        if correction_sign and previous_correction_sign and correction_sign != previous_correction_sign:
          counts["correctionReversalFrames"] += 1
          input_driven = (
            (target_reversal_age is not None and target_reversal_age <= TARGET_REVERSAL_CAUSAL_WINDOW_FRAMES)
            or (pending_target_sign != 0 and correction_sign == pending_target_sign)
            or (target_sign != 0 and correction_sign == target_sign)
          )
          if input_driven:
            counts["inputDrivenReversalFrames"] += 1
          else:
            counts["controllerOnlyReversalFrames"] += 1
          if len(reversal_samples) < 40:
            reversal_samples.append({
              "segment": segment,
              "frameId": frame_id,
              "logMonoTime": int(safe_attr(msg, "logMonoTime", 0)),
              "vEgo": round(v_ego, 5),
              "correction": round(correction, 9),
              "rawCorrection": round(raw_correction, 9),
              "centerError": round(center_error, 6),
              "targetSign": target_sign,
              "targetReversalAgeFrames": target_reversal_age,
              "classification": "input-driven" if input_driven else "controller-only",
              "previousFrame": previous_context,
            })
          if input_driven and correction_sign == pending_target_sign:
            pending_target_sign = 0
        elif correction_sign and target_sign and correction_sign == target_sign:
          pending_target_sign = 0
        if correction_sign:
          previous_correction_sign = correction_sign
        if target_sign:
          previous_target_sign = target_sign

        if curve:
          counts["autonomousCurveFrames"] += 1
          values["curveRawError"].append(raw_abs)
          values["curveResidualError"].append(residual_abs)
          values["curveAddedLatAccel"].append(added_lat_accel)
          values["curveProjectedClearance"].append(projected_clearance)

        speed_band = "5-10mps" if v_ego < 10.0 else "10-20mps" if v_ego < 20.0 else "20mps+"
        by_speed[speed_band]["raw"].append(raw_abs)
        by_speed[speed_band]["residual"].append(residual_abs)
        path_side = "modelLeftOfCenter" if center_error > 0.0 else "modelRightOfCenter"
        by_side[path_side]["raw"].append(raw_abs)
        by_side[path_side]["residual"].append(residual_abs)

        if previous_correction is not None:
          step_lat_accel = abs(correction - previous_correction) * v_ego ** 2
          values["correctionStepLatAccel"].append(step_lat_accel)
          if max_step_sample is None or step_lat_accel > max_step_sample["latAccelStep"]:
            max_step_sample = {
              "segment": segment,
              "frameId": frame_id,
              "logMonoTime": int(safe_attr(msg, "logMonoTime", 0)),
              "vEgo": round(v_ego, 5),
              "latAccelStep": round(step_lat_accel, 6),
              "previousCorrection": round(previous_correction, 9),
              "correction": round(correction, 9),
              "rawCorrection": round(raw_correction, 9),
              "centerError": round(center_error, 6),
              "laneWidth": round(width, 6),
              "laneCenteringReason": safe_attr(controller, "_lane_centering_reason", "starpilot-dom"),
              "previousFrame": previous_context,
            }

      if steering_pressed and raw_valid and abs(steering_torque) >= DRIVER_TORQUE and raw_abs >= DRIVER_ERROR:
        counts["driverComparisonFrames"] += 1
        if raw_correction * steering_torque < 0.0:
          counts["driverAgreementFrames"] += 1
      if steering_pressed:
        previous_correction_sign = 0
        previous_target_sign = 0
        target_reversal_age = None
        pending_target_sign = 0

      previous_correction = correction
      previous_context = {
        "segment": segment,
        "frameId": frame_id,
        "vEgo": round(v_ego, 5),
        "correction": round(correction, 9),
        "rawCorrection": round(raw_correction, 9),
        "centerError": round(center_error, 6),
        "laneWidth": round(width, 6),
        "laneCenteringReason": safe_attr(controller, "_lane_centering_reason", "starpilot-dom"),
      }

  auto = counts["autonomousEligibleFrames"]
  driver = counts["driverComparisonFrames"]
  return {
    "route": route,
    "segments": len(paths),
    "metadata": metadata,
    "architecture": {
      "referenceOwner": "trusted-primary-lane-pair",
      "roadAwareActuation": False,
      "roadEdgesConsumed": False,
      "recordedRoadAwareParamIgnored": metadata.get("recordedLaneCenteringRoadAware", "unknown"),
      "centerGain": configured_center_gain(),
      "smoothTau": lane_centering._SMOOTH_TAU,
      "curveAdaptive": curve_adaptive,
      "e2eAuthority": e2e_authority,
    },
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
    "stability": {
      "correctionReversals": counts["correctionReversalFrames"],
      "targetReversals": counts["targetReversalFrames"],
      "inputDrivenReversals": counts["inputDrivenReversalFrames"],
      "controllerOnlyReversals": counts["controllerOnlyReversalFrames"],
      "correctionReversalPct": pct(counts["correctionReversalFrames"], auto),
      "targetReversalPct": pct(counts["targetReversalFrames"], auto),
      "reversalSamples": reversal_samples,
    },
    "clearance": {
      "baselineM": stats(values["baselineClearance"]),
      "projectedM": stats(values["projectedClearance"]),
      "baselineViolationPct": pct(counts["baselineClearanceViolationFrames"], auto),
      "projectedViolationPct": pct(counts["projectedClearanceViolationFrames"], auto),
      "introducedViolationPct": pct(counts["introducedClearanceViolationFrames"], auto),
      "introducedDeficitM": stats(values["introducedClearanceDeficit"]),
      "introducedSamples": introduced_clearance_samples,
    },
    "curves": {
      "rawAbsM": stats(values["curveRawError"]),
      "projectedResidualAbsM": stats(values["curveResidualError"]),
      "addedLatAccel": stats(values["curveAddedLatAccel"]),
      "projectedClearanceM": stats(values["curveProjectedClearance"]),
    },
    "demand": {
      "addedLatAccel": stats(values["addedLatAccel"]),
      "totalDesiredLatAccel": stats(values["totalLatAccel"]),
      "correctionStepLatAccel": stats(values["correctionStepLatAccel"]),
      "maxCorrectionStepSample": max_step_sample,
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
    "speedBands": {
      name: {
        "frames": len(bucket["raw"]),
        "rawP95M": round(percentile(bucket["raw"], 95) or 0.0, 4),
        "projectedP95M": round(percentile(bucket["residual"], 95) or 0.0, 4),
      }
      for name, bucket in sorted(by_speed.items())
    },
    "pathSide": {
      name: {
        "frames": len(bucket["raw"]),
        "rawP95M": round(percentile(bucket["raw"], 95) or 0.0, 4),
        "projectedP95M": round(percentile(bucket["residual"], 95) or 0.0, 4),
      }
      for name, bucket in sorted(by_side.items())
    },
  }


def main() -> None:
  parser = argparse.ArgumentParser(description="Replay the lane-only centering controller over historical rlogs")
  parser.add_argument("paths", nargs="+", type=Path)
  parser.add_argument("--json", action="store_true", help="emit the complete machine-readable report")
  parser.add_argument("--output", type=Path, help="also write the JSON report to this path")
  parser.add_argument("--e2e-authority", type=float, default=0.15)
  parser.add_argument("--center-gain", type=float, help="override correction gain for counterfactual replay")
  parser.add_argument("--smooth-tau", type=float, help="override filter time constant for counterfactual replay")
  parser.add_argument("--use-log-settings", action="store_true", help="replay each route with its recorded E2E authority")
  parser.add_argument("--curve-adaptive", action="store_true", help="enable the Carnival curve-adaptive lane response")
  args = parser.parse_args()

  if args.center_gain is not None:
    if hasattr(lane_centering, "_CENTER_GAIN"):
      lane_centering._CENTER_GAIN = float(args.center_gain)
    else:
      lane_centering._MAX_GAIN = float(args.center_gain)
  if args.smooth_tau is not None:
    lane_centering._SMOOTH_TAU = float(args.smooth_tau)

  routes = discover_logs(args.paths)
  e2e_authority = None if args.use_log_settings else args.e2e_authority
  reports = [analyze_route(route, paths, e2e_authority, args.curve_adaptive) for route, paths in routes.items()]
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(reports, indent=2, sort_keys=True) + "\n")
  if args.json:
    print(json.dumps(reports, indent=2, sort_keys=True))
    return

  for report in reports:
    coverage = report["coverage"]
    centering = report["centering"]
    stability = report["stability"]
    clearance = report["clearance"]
    curves = report["curves"]
    demand = report["demand"]
    print(" ".join([
      f'{report["route"]} seg={report["segments"]}',
      f'commit={report["metadata"].get("gitCommit", "unknown")[:9]}',
      f'LC={report["metadata"].get("laneCentering", "?")}',
      f'loggedRA={report["metadata"].get("recordedLaneCenteringRoadAware", "?")}',
      f'eligible={coverage["autonomousEligibleFrames"]}',
      f'avail={coverage["strictGeometryAvailabilityPct"]:.1f}%',
      f'p95={centering["rawAbsM"]["p95"]}->{centering["projectedResidualAbsM"]["p95"]}m',
      f'curveP95={curves["rawAbsM"]["p95"]}->{curves["projectedResidualAbsM"]["p95"]}m',
      f'worse2cm={centering["worsenedOver2cmPct"]:.2f}%',
      f'worse5cm={centering["worsenedOver5cmPct"]:.2f}%',
      f'wrongDir={centering["wrongDirectionPct"]:.2f}%',
      f'reversals={stability["correctionReversals"]}',
      f'controllerOnlyRev={stability["controllerOnlyReversals"]}',
      f'clearanceP05={clearance["baselineM"]["p05"]}->{clearance["projectedM"]["p05"]}m',
      f'introducedClearance={clearance["introducedViolationPct"]:.2f}%',
      f'addP95={demand["addedLatAccel"]["p95"]}',
      f'stepMax={demand["correctionStepLatAccel"]["max"]}',
      f'releaseMax={demand["confidenceReleaseStepLatAccel"]["max"]}',
    ]))


if __name__ == "__main__":
  main()
