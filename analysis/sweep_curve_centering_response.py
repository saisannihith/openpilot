#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from analysis.analyze_curve_centering_decomposition import (
  MAX_LANE_STD,
  MIN_CURVE_LAT_ACCEL,
  MIN_LANE_PROB,
  MIN_SPEED,
  discover_logs,
  extract_geometry,
  pct,
  route_and_segment,
  safe_attr,
  safe_float,
  stats,
)
from openpilot.tools.lib.logreader import LogReader, ReadMode


MODEL_DT = 0.05
SMOOTH_TAU = 0.40
RELEASE_TAU = 0.20
MAX_RAW_CORRECTION = 0.004
E2E_BREAK_IN_START = 0.30
E2E_BREAK_IN_FULL = 0.60
E2E_MAX_PATH_STD = 0.35
CURVE_SCHEDULE_START = 0.20
CURVE_SCHEDULE_FULL = 0.80


@dataclass(frozen=True)
class Profile:
  name: str
  curve_gain: float
  curve_deadband: float
  curve_tau: float


PROFILES = (
  Profile("current", 0.30, 0.08, 0.40),
  Profile("curve_045_db060_tau030", 0.45, 0.06, 0.30),
  Profile("curve_055_db050_tau025", 0.55, 0.05, 0.25),
  Profile("curve_065_db040_tau020", 0.65, 0.04, 0.20),
)


class Variant:
  def __init__(self, profile: Profile) -> None:
    self.profile = profile
    self.correction = 0.0

  @staticmethod
  def smooth(target: float, previous: float, tau: float) -> float:
    alpha = 1.0 - math.exp(-MODEL_DT / tau) if tau > 0.0 else 1.0
    return alpha * target + (1.0 - alpha) * previous

  def reset(self) -> None:
    self.correction = 0.0

  def update(self, geometry: dict[str, float] | None, model_curvature: float, v_ego: float,
             hard_active: bool, soft_active: bool) -> float:
    if not hard_active:
      self.reset()
      return self.correction
    if not soft_active or geometry is None:
      self.correction = self.smooth(0.0, self.correction, RELEASE_TAU)
      return self.correction

    error = geometry["farCenter"] - geometry["farPath"]
    error_abs = abs(error)
    curve_lat_accel = abs(model_curvature) * v_ego * v_ego
    curve_weight = 0.0
    if model_curvature * error < 0.0:
      curve_weight = float(np.clip(
        (curve_lat_accel - CURVE_SCHEDULE_START) / (CURVE_SCHEDULE_FULL - CURVE_SCHEDULE_START),
        0.0,
        1.0,
      ))
    deadband = 0.08 + curve_weight * (self.profile.curve_deadband - 0.08)
    gain = 0.30 + curve_weight * (self.profile.curve_gain - 0.30)
    smooth_tau = SMOOTH_TAU + curve_weight * (self.profile.curve_tau - SMOOTH_TAU)
    error = math.copysign(max(error_abs - deadband, 0.0), error)

    if 0.0 <= geometry["farPathStd"] <= E2E_MAX_PATH_STD:
      break_in = float(np.clip(
        (error_abs - E2E_BREAK_IN_START) / (E2E_BREAK_IN_FULL - E2E_BREAK_IN_START),
        0.0,
        1.0,
      ))
      error *= 1.0 - break_in

    raw = float(np.clip(2.0 * error / geometry["farDistance"] ** 2,
                        -MAX_RAW_CORRECTION, MAX_RAW_CORRECTION))
    self.correction = self.smooth(raw * gain, self.correction, smooth_tau)
    return self.correction


def analyze_route(route: str, paths: list[Path]) -> dict[str, Any]:
  variants = {profile.name: Variant(profile) for profile in PROFILES}
  latest: dict[str, Any] = {}
  values: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
  counts: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
  previous_correction: dict[str, float] = {profile.name: 0.0 for profile in PROFILES}
  previous_frame_id: int | None = None
  previous_segment: int | None = None

  for path in paths:
    segment = route_and_segment(path)[1]
    if previous_segment is not None and segment != previous_segment + 1:
      for variant in variants.values():
        variant.reset()
      previous_correction = {profile.name: 0.0 for profile in PROFILES}
      previous_frame_id = None
    previous_segment = segment

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

      model_curvature = safe_float(safe_attr(safe_attr(model, "action"), "desiredCurvature", 0.0))
      geometry = extract_geometry(model, v_ego)
      hard_active = lat_active and v_ego >= MIN_SPEED and lane_change == 0
      soft_active = not signal
      corrections = {
        name: variant.update(geometry, model_curvature, v_ego, hard_active, soft_active)
        for name, variant in variants.items()
      }

      strict_autonomous = hard_active and soft_active and not steering_pressed and geometry is not None
      curve_lat_accel = abs(model_curvature) * v_ego * v_ego
      if not strict_autonomous or curve_lat_accel < MIN_CURVE_LAT_ACCEL:
        for name, correction in corrections.items():
          previous_correction[name] = correction
        continue

      direction_sign = 1.0 if model_curvature > 0.0 else -1.0
      direction = "right" if direction_sign > 0.0 else "left"
      model_inside = direction_sign * (geometry["farPath"] - geometry["farCenter"])
      for name, correction in corrections.items():
        projected_shift = correction * geometry["farDistance"] ** 2 / 2.0
        projected_inside = model_inside + direction_sign * projected_shift
        added_lat_accel = abs(correction) * v_ego * v_ego
        step_lat_accel = abs(correction - previous_correction[name]) * v_ego * v_ego
        values[name]["projectedInsideM"].append(projected_inside)
        values[name]["projectedAbsM"].append(abs(projected_inside))
        values[name][f"{direction}ProjectedInsideM"].append(projected_inside)
        values[name]["addedLatAccel"].append(added_lat_accel)
        values[name]["stepLatAccel"].append(step_lat_accel)
        if projected_inside > 0.10:
          counts[name]["insideOver10cm"] += 1
        if abs(projected_inside) > abs(model_inside) + 1e-6:
          counts[name]["worsened"] += 1
        if abs(projected_inside) > abs(model_inside) + 0.02:
          counts[name]["worsenedOver2cm"] += 1
        if direction_sign * correction > 1e-7:
          counts[name]["wrongDirection"] += 1
        previous_correction[name] = correction
      counts["all"]["curveFrames"] += 1

  curve_frames = counts["all"]["curveFrames"]
  return {
    "route": route,
    "segments": len(paths),
    "curveFrames": curve_frames,
    "profiles": {
      profile.name: {
        "curveGain": profile.curve_gain,
        "curveDeadband": profile.curve_deadband,
        "curveSmoothTau": profile.curve_tau,
        "projectedInsideM": stats(values[profile.name]["projectedInsideM"]),
        "projectedAbsM": stats(values[profile.name]["projectedAbsM"]),
        "rightProjectedInsideM": stats(values[profile.name]["rightProjectedInsideM"]),
        "leftProjectedInsideM": stats(values[profile.name]["leftProjectedInsideM"]),
        "addedLatAccel": stats(values[profile.name]["addedLatAccel"]),
        "stepLatAccel": stats(values[profile.name]["stepLatAccel"]),
        "insideOver10cmPct": pct(counts[profile.name]["insideOver10cm"], curve_frames),
        "worsenedPct": pct(counts[profile.name]["worsened"], curve_frames),
        "worsenedOver2cmPct": pct(counts[profile.name]["worsenedOver2cm"], curve_frames),
        "wrongDirectionPct": pct(counts[profile.name]["wrongDirection"], curve_frames),
      }
      for profile in PROFILES
    },
  }


def main() -> None:
  parser = argparse.ArgumentParser(description="One-pass curve lane-centering response sweep")
  parser.add_argument("paths", nargs="+", type=Path)
  parser.add_argument("--output", type=Path)
  parser.add_argument("--json", action="store_true")
  args = parser.parse_args()

  reports = [analyze_route(route, paths) for route, paths in discover_logs(args.paths).items()]
  rendered = json.dumps(reports, indent=2, sort_keys=True)
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
  if args.json:
    print(rendered)
  else:
    for report in reports:
      print(report["route"], f'curveFrames={report["curveFrames"]}')
      for name, profile in report["profiles"].items():
        print(
          f'  {name}: absP95={profile["projectedAbsM"]["p95"]}',
          f'rightP50={profile["rightProjectedInsideM"]["p50"]}',
          f'addedP95={profile["addedLatAccel"]["p95"]}',
          f'stepP95={profile["stepLatAccel"]["p95"]}',
          f'worse2cm={profile["worsenedOver2cmPct"]}%',
        )


if __name__ == "__main__":
  main()
