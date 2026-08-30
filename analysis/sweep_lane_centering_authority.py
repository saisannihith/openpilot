#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from analysis.evaluate_lane_centering_replay import (
  CURVE_LAT_ACCEL,
  MIN_SPEED,
  MODEL_TICKS,
  discover_logs,
  lane_geometry,
  safe_attr,
  safe_float,
)
from openpilot.selfdrive.controls.lib import lane_centering as lane_centering_module
from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController
from openpilot.tools.lib.logreader import LogReader, ReadMode


AUTHORITIES = (0.0, 0.1, 0.15, 0.2, 0.3, 0.5, 0.75, 1.0)


def percentile(values: list[float], pct: float) -> float:
  return float(np.percentile(np.asarray(values, dtype=float), pct)) if values else math.nan


def analyze_route(route: str, paths: list[Path], road_edge_offset: float,
                  topology_release_frames: int, topology_smooth_tau: float,
                  correction_tau: float, center_gain: float, topology_switch_acquire_frames: int,
                  road_aware: bool,
                  authorities: tuple[float, ...] = AUTHORITIES) -> list[dict[str, Any]]:
  lane_centering_module._ROAD_TOPOLOGY_RELEASE_FRAMES = topology_release_frames
  lane_centering_module._ROAD_TOPOLOGY_ACTIVE_DECAY = max(
    1,
    lane_centering_module._ROAD_TOPOLOGY_ACQUIRE_FRAMES *
    lane_centering_module._ROAD_TOPOLOGY_EVIDENCE_GAIN // topology_release_frames,
  )
  lane_centering_module._ROAD_TOPOLOGY_SMOOTH_TAU = topology_smooth_tau
  lane_centering_module._SMOOTH_TAU = correction_tau
  lane_centering_module._CENTER_GAIN = center_gain
  lane_centering_module._ROAD_TOPOLOGY_SWITCH_ACQUIRE_FRAMES = topology_switch_acquire_frames
  controllers = {
    authority: LaneCenteringController()
    for authority in authorities
  }
  centered_controllers = {
    authority: LaneCenteringController()
    for authority in authorities
  }
  latest: dict[str, Any] = {}
  previous_frame_id: int | None = None
  previous_correction = dict.fromkeys(authorities, 0.0)
  previous_correction_delta = dict.fromkeys(authorities, 0.0)
  previous_centered_correction = dict.fromkeys(authorities, 0.0)
  previous_centered_delta = dict.fromkeys(authorities, 0.0)
  previous_topology_correction = dict.fromkeys(authorities, 0.0)
  previous_topology_delta = dict.fromkeys(authorities, 0.0)
  values = {authority: defaultdict(list) for authority in authorities}
  counts = {authority: defaultdict(int) for authority in authorities}
  previous_segment: int | None = None

  for path in paths:
    current_segment = int(path.parent.name.rsplit("--", 1)[-1])
    if previous_segment is not None and current_segment != previous_segment + 1:
      for controller in (*controllers.values(), *centered_controllers.values()):
        controller.reset()
      previous_correction = dict.fromkeys(authorities, 0.0)
      previous_correction_delta = dict.fromkeys(authorities, 0.0)
      previous_centered_correction = dict.fromkeys(authorities, 0.0)
      previous_centered_delta = dict.fromkeys(authorities, 0.0)
      previous_topology_correction = dict.fromkeys(authorities, 0.0)
      previous_topology_delta = dict.fromkeys(authorities, 0.0)
      previous_frame_id = None
    previous_segment = current_segment
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
      which = msg.which()
      if which in ("carState", "carControl", "controlsState"):
        latest[which] = getattr(msg, which)
        continue
      if which != "modelV2" or "carState" not in latest or "carControl" not in latest:
        continue

      model = msg.modelV2
      frame_id = int(safe_attr(model, "frameId", 0))
      if frame_id == previous_frame_id:
        continue
      previous_frame_id = frame_id

      car_state = latest["carState"]
      car_control = latest["carControl"]
      controls_state = latest.get("controlsState")
      v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
      lat_active = bool(safe_attr(car_control, "latActive", False))
      steering_pressed = bool(safe_attr(car_state, "steeringPressed", False))
      signal = bool(safe_attr(car_state, "leftBlinker", False) or safe_attr(car_state, "rightBlinker", False))
      try:
        lane_change = int(model.meta.laneChangeState)
      except Exception:
        lane_change = 0

      action = safe_attr(model, "action")
      raw_action_curvature = safe_attr(action, "desiredCurvature")
      model_curvature = safe_float(raw_action_curvature) if raw_action_curvature is not None else safe_float(
        safe_attr(controls_state, "desiredCurvature", 0.0),
      )
      strict, center_error, width, _, lookahead, _, _, _ = lane_geometry(model, v_ego)
      base_eligible = lat_active and v_ego >= MIN_SPEED and lane_change == 0 and not signal and not steering_pressed
      curve = abs(model_curvature) * v_ego ** 2 >= CURVE_LAT_ACCEL

      for authority, controller in controllers.items():
        output = model_curvature
        centered_output = model_curvature
        centered_controller = centered_controllers[authority]
        for _ in range(MODEL_TICKS):
          output = controller.update(
            model_curvature, model, v_ego, True, 0.0, authority, lat_active, True,
            True, signal, steering_pressed, road_aware, road_edge_offset,
          )
          centered_output = centered_controller.update(
            model_curvature, model, v_ego, True, 0.0, authority, lat_active, True,
            True, signal, steering_pressed, False, road_edge_offset,
          )
        correction = output - model_curvature
        centered_correction = centered_output - model_curvature
        topology_correction = output - centered_output
        if not (base_eligible and strict):
          previous_correction[authority] = correction
          previous_correction_delta[authority] = 0.0
          previous_centered_correction[authority] = centered_correction
          previous_centered_delta[authority] = 0.0
          previous_topology_correction[authority] = topology_correction
          previous_topology_delta[authority] = 0.0
          continue

        topology_bias = controller._effective_road_topology_bias
        target_error = center_error + topology_bias
        projected_shift = correction * lookahead ** 2 / 2.0
        target_residual = target_error - projected_shift
        center_residual = center_error - projected_shift
        boundary_clearance = width * 0.5 - abs(center_residual)
        added_lat_accel = abs(correction) * v_ego ** 2
        correction_step = abs(correction - previous_correction[authority]) * v_ego ** 2
        correction_delta = correction - previous_correction[authority]
        centered_delta = centered_correction - previous_centered_correction[authority]
        topology_delta = topology_correction - previous_topology_correction[authority]
        topology_step = abs(topology_delta) * v_ego ** 2

        counts[authority]["frames"] += 1
        counts[authority]["curveFrames"] += int(curve)
        counts[authority]["outerFrames"] += int(controller._road_topology_state != 0)
        counts[authority]["wrongDirectionFrames"] += int(correction * target_error < 0.0)
        counts[authority]["worsened2cmFrames"] += int(abs(target_residual) > abs(target_error) + 0.02)
        counts[authority]["saturatedFrames"] += int(added_lat_accel >= 0.594)
        counts[authority]["clearanceUnder120Frames"] += int(boundary_clearance < 1.20)
        values[authority]["targetResidual"].append(abs(target_residual))
        values[authority]["centerResidual"].append(abs(center_residual))
        values[authority]["boundaryClearance"].append(boundary_clearance)
        values[authority]["addedLatAccel"].append(added_lat_accel)
        values[authority]["correctionStep"].append(correction_step)
        values[authority]["topologyStep"].append(topology_step)
        if (correction_delta * previous_correction_delta[authority] < 0.0
            and correction_step > 0.005
            and abs(previous_correction_delta[authority]) * v_ego ** 2 > 0.005):
          counts[authority]["correctionReversals"] += 1
        centered_step = abs(centered_delta) * v_ego ** 2
        if (centered_delta * previous_centered_delta[authority] < 0.0
            and centered_step > 0.005
            and abs(previous_centered_delta[authority]) * v_ego ** 2 > 0.005):
          counts[authority]["centeredCorrectionReversals"] += 1
        if (controller._road_topology_state != 0
            and topology_delta * previous_topology_delta[authority] < 0.0
            and topology_step > 0.005
            and abs(previous_topology_delta[authority]) * v_ego ** 2 > 0.005):
          counts[authority]["topologyReversals"] += 1
        if curve:
          values[authority]["curveTargetResidual"].append(abs(target_residual))
        previous_correction[authority] = correction
        previous_correction_delta[authority] = correction_delta
        previous_centered_correction[authority] = centered_correction
        previous_centered_delta[authority] = centered_delta
        previous_topology_correction[authority] = topology_correction
        previous_topology_delta[authority] = topology_delta

  reports = []
  for authority in authorities:
    frame_count = counts[authority]["frames"]
    curve_count = counts[authority]["curveFrames"]
    reports.append({
      "route": route,
      "authority": authority,
      "topologyReleaseFrames": topology_release_frames,
      "topologySmoothTau": topology_smooth_tau,
      "correctionTau": correction_tau,
      "centerGain": center_gain,
      "roadAware": road_aware,
      "topologySwitchAcquireFrames": topology_switch_acquire_frames,
      "frames": frame_count,
      "outerPct": 100.0 * counts[authority]["outerFrames"] / frame_count if frame_count else 0.0,
      "targetP95M": percentile(values[authority]["targetResidual"], 95),
      "curveTargetP95M": percentile(values[authority]["curveTargetResidual"], 95),
      "centerP95M": percentile(values[authority]["centerResidual"], 95),
      "clearanceP05M": percentile(values[authority]["boundaryClearance"], 5),
      "clearanceUnder120Pct": 100.0 * counts[authority]["clearanceUnder120Frames"] / frame_count if frame_count else 0.0,
      "addedLatAccelP95": percentile(values[authority]["addedLatAccel"], 95),
      "correctionStepMax": max(values[authority]["correctionStep"], default=math.nan),
      "topologyStepP95": percentile(values[authority]["topologyStep"], 95),
      "topologyReversalsPer1kOuter": 1000.0 * counts[authority]["topologyReversals"] / max(counts[authority]["outerFrames"], 1),
      "correctionReversalsPer1k": 1000.0 * counts[authority]["correctionReversals"] / max(frame_count, 1),
      "centeredCorrectionReversalsPer1k": 1000.0 * counts[authority]["centeredCorrectionReversals"] / max(frame_count, 1),
      "wrongDirectionPct": 100.0 * counts[authority]["wrongDirectionFrames"] / frame_count if frame_count else 0.0,
      "worsened2cmPct": 100.0 * counts[authority]["worsened2cmFrames"] / frame_count if frame_count else 0.0,
      "saturatedPct": 100.0 * counts[authority]["saturatedFrames"] / frame_count if frame_count else 0.0,
      "curveFrames": curve_count,
    })
  return reports


def main() -> None:
  parser = argparse.ArgumentParser(description="Single-pass LaneCenteringE2EAuthority sweep")
  parser.add_argument("paths", nargs="+", type=Path)
  parser.add_argument("--road-edge-offset", type=float, default=0.15)
  parser.add_argument("--topology-release-frames", type=int, default=20)
  parser.add_argument("--topology-smooth-tau", type=float, default=2.0)
  parser.add_argument("--correction-tau", type=float, default=0.4)
  parser.add_argument("--center-gain", type=float, default=0.75)
  parser.add_argument("--correction-taus", nargs="+", type=float)
  parser.add_argument("--center-gains", nargs="+", type=float)
  parser.add_argument("--road-aware", action=argparse.BooleanOptionalAction, default=True)
  parser.add_argument("--topology-switch-acquire-frames", type=int, default=60)
  parser.add_argument("--authorities", nargs="+", type=float, default=list(AUTHORITIES))
  args = parser.parse_args()

  if args.topology_release_frames <= 0 or args.topology_switch_acquire_frames <= 0:
    parser.error("topology frame counts must be positive")
  correction_taus = tuple(args.correction_taus or (args.correction_tau,))
  center_gains = tuple(args.center_gains or (args.center_gain,))
  if args.topology_smooth_tau <= 0.0 or any(value <= 0.0 for value in (*correction_taus, *center_gains)):
    parser.error("smoothing time constants must be positive")
  if not args.authorities or any(not 0.0 <= authority <= 1.0 for authority in args.authorities):
    parser.error("authorities must be in [0, 1]")

  all_reports = []
  for route, paths in discover_logs(args.paths).items():
    for correction_tau in correction_taus:
      for center_gain in center_gains:
        reports = analyze_route(
          route, paths, args.road_edge_offset,
          args.topology_release_frames, args.topology_smooth_tau, correction_tau, center_gain,
          args.topology_switch_acquire_frames, args.road_aware, tuple(args.authorities),
        )
        all_reports.extend(reports)
        for report in reports:
          print(" ".join((
        f'{route} authority={report["authority"]:.2f} frames={report["frames"]}',
        f'targetP95={report["targetP95M"]:.5f} curveP95={report["curveTargetP95M"]:.5f}',
        f'centerP95={report["centerP95M"]:.5f} clearanceP05={report["clearanceP05M"]:.5f}',
        f'clearance<1.2={report["clearanceUnder120Pct"]:.2f}% addP95={report["addedLatAccelP95"]:.5f}',
        f'stepMax={report["correctionStepMax"]:.5f} wrongDir={report["wrongDirectionPct"]:.2f}%',
        f'topologyStepP95={report["topologyStepP95"]:.5f}',
        f'topologyReversals/1k={report["topologyReversalsPer1kOuter"]:.2f}',
        f'correctionReversals/1k={report["correctionReversalsPer1k"]:.2f}',
        f'centeredReversals/1k={report["centeredCorrectionReversalsPer1k"]:.2f}',
        f'worse2cm={report["worsened2cmPct"]:.2f}% sat={report["saturatedPct"]:.2f}%',
        f'outer={report["outerPct"]:.2f}%',
        f'release={report["topologyReleaseFrames"]} tau={report["topologySmoothTau"]:.2f}',
        f'correctionTau={report["correctionTau"]:.2f}',
        f'centerGain={report["centerGain"]:.2f}',
        f'switchAcquire={report["topologySwitchAcquireFrames"]}',
          )))


if __name__ == "__main__":
  main()
