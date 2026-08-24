#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode
from openpilot.selfdrive.controls.lib.latcontrol_vehicle_tunes import get_kia_carnival_driver_override_output_scale
from opendbc.car.hyundai.carcontroller import (
  CARNIVAL_4TH_GEN_EPS_GUARD_MIN_ANGLE,
  CARNIVAL_4TH_GEN_EPS_GUARD_MIN_SPEED,
  CARNIVAL_4TH_GEN_EPS_GUARD_TORQUE_FRACTION,
  CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_MAX_SPEED,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_MIN_DRIVER_TORQUE,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_RELEASE_FRAMES,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_START_ANGLE,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_TOUCH_ANGLE,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_YIELD_ANGLE,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_YIELD_MIN_DRIVER_TORQUE,
  apply_carnival_4th_gen_eps_fault_guard,
  apply_carnival_4th_gen_manual_turn_torque_guard,
)
from opendbc.car.hyundai.values import CAR


CARNIVAL_STEER_MAX = 409
HIGH_SPEED_FAULT_MPS = 17.0
NEAR_FAULT_WINDOW_S = 2.0


@dataclass
class LateralSample:
  route: str
  segment: int
  t: float
  git_commit: str
  v_ego: float
  lat_active: bool
  enabled: bool
  steer_fault_temporary: bool
  low_speed_alert: bool
  steering_pressed: bool
  steering_torque: float
  steering_torque_eps: float
  steering_angle_deg: float
  commanded_torque: float
  output_torque: float
  output_torque_units: int
  override_release_scale: float
  manual_turn_guard_candidate: bool
  eps_guard_near_limit: bool


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


def route_name(path: Path) -> str:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 2:
      return "--".join(parts[:2])
  return path.parent.name


def segment_number(path: Path) -> int:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 3:
      try:
        return int(parts[2].split(".", 1)[0])
      except Exception:
        pass
  return -1


def expand_log_paths(paths: list[Path]) -> list[Path]:
  logs: list[Path] = []
  for path in paths:
    if path.is_dir():
      logs.extend(sorted(path.glob("qlog.zst")))
      logs.extend(sorted(path.glob("qlog.bz2")))
      logs.extend(sorted(path.glob("qlog")))
    else:
      logs.extend(sorted(path.parent.glob(path.name)))
  return sorted((path for path in logs if path.exists()), key=lambda p: (route_name(p), segment_number(p), str(p)))


def current_commit() -> str:
  for rev in ("origin/snithpilot", "HEAD"):
    try:
      return subprocess.check_output(["git", "rev-parse", rev], text=True).strip()
    except Exception:
      pass
  return "unknown"


def get_git_commit(log_init: Any) -> str:
  for name in ("gitCommit", "gitCommitDate"):
    value = safe_attr(log_init, name)
    if value and name == "gitCommit":
      return str(value)
  return "unknown"


def read_lateral_samples(path: Path, mode: ReadMode, include_lat_active_frames: bool) -> tuple[list[LateralSample], str]:
  latest: dict[str, Any] = {}
  samples: list[LateralSample] = []
  start_ns: int | None = None
  git_commit = "unknown"

  for msg in LogReader(str(path), default_mode=mode, sort_by_time=True):
    which = msg.which()
    mono_time = int(msg.logMonoTime)

    if which == "initData":
      git_commit = get_git_commit(msg.initData)
      continue

    if which in ("carState", "carControl", "carOutput", "controlsState"):
      latest[which] = getattr(msg, which)
      if start_ns is None:
        start_ns = mono_time

    if start_ns is None or "carState" not in latest:
      continue

    car_state = latest["carState"]
    car_control = latest.get("carControl")
    car_output = latest.get("carOutput")
    controls_state = latest.get("controlsState")
    actuators = safe_attr(car_control, "actuators")
    out_actuators = safe_attr(car_output, "actuatorsOutput") if car_output is not None else None

    v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
    steering_torque = safe_float(safe_attr(car_state, "steeringTorque", 0.0))
    steering_angle = safe_float(safe_attr(car_state, "steeringAngleDeg", 0.0))
    steering_pressed = bool(safe_attr(car_state, "steeringPressed", False))
    output_torque = safe_float(safe_attr(out_actuators, "torque", safe_attr(actuators, "torque", 0.0)))
    output_units = int(round(output_torque * CARNIVAL_STEER_MAX))
    torque_fraction = abs(output_units) / CARNIVAL_STEER_MAX
    lat_active = bool(safe_attr(car_control, "latActive", False))

    manual_candidate = (
      steering_pressed and
      v_ego <= CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_MAX_SPEED and
      (
        (
          abs(steering_angle) >= CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_START_ANGLE and
          abs(steering_torque) >= CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_MIN_DRIVER_TORQUE
        ) or
        (
          abs(steering_angle) >= CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_YIELD_ANGLE and
          abs(steering_torque) >= CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_YIELD_MIN_DRIVER_TORQUE
        ) or
        (
          abs(steering_angle) >= CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_TOUCH_ANGLE
        )
      )
    )
    eps_near_limit = (
      lat_active and
      v_ego >= CARNIVAL_4TH_GEN_EPS_GUARD_MIN_SPEED and
      abs(steering_angle) >= CARNIVAL_4TH_GEN_EPS_GUARD_MIN_ANGLE and
      torque_fraction >= CARNIVAL_4TH_GEN_EPS_GUARD_TORQUE_FRACTION
    )

    interesting = (
      (include_lat_active_frames and lat_active) or
      bool(safe_attr(car_state, "steerFaultTemporary", False)) or
      bool(safe_attr(car_state, "lowSpeedAlert", False)) or
      manual_candidate or
      eps_near_limit
    )
    if not interesting:
      continue

    samples.append(LateralSample(
      route=route_name(path),
      segment=segment_number(path),
      t=(mono_time - start_ns) / 1e9,
      git_commit=git_commit,
      v_ego=v_ego,
      lat_active=lat_active,
      enabled=bool(safe_attr(controls_state, "enabled", False)),
      steer_fault_temporary=bool(safe_attr(car_state, "steerFaultTemporary", False)),
      low_speed_alert=bool(safe_attr(car_state, "lowSpeedAlert", False)),
      steering_pressed=steering_pressed,
      steering_torque=steering_torque,
      steering_torque_eps=safe_float(safe_attr(car_state, "steeringTorqueEps", 0.0)),
      steering_angle_deg=steering_angle,
      commanded_torque=safe_float(safe_attr(actuators, "torque", 0.0)),
      output_torque=output_torque,
      output_torque_units=output_units,
      override_release_scale=get_kia_carnival_driver_override_output_scale(v_ego, steering_torque),
      manual_turn_guard_candidate=manual_candidate,
      eps_guard_near_limit=eps_near_limit,
    ))

  return samples, git_commit


def event_dict(sample: LateralSample) -> dict[str, Any]:
  data = asdict(sample)
  for key, value in list(data.items()):
    if isinstance(value, float):
      data[key] = round(value, 3)
  return data


def eps_guard_event_dict(sample: LateralSample, guarded_torque: int, high_torque_frames: int,
                         guard_frames: int) -> dict[str, Any]:
  data = event_dict(sample)
  data["guardedOutputTorqueUnits"] = guarded_torque
  data["guardedTorqueReductionUnits"] = abs(sample.output_torque_units) - abs(guarded_torque)
  data["guardedTorqueFraction"] = round(abs(guarded_torque) / CARNIVAL_STEER_MAX, 3)
  data["rawTorqueFraction"] = round(abs(sample.output_torque_units) / CARNIVAL_STEER_MAX, 3)
  data["simHighTorqueFrames"] = high_torque_frames
  data["simGuardFrames"] = guard_frames
  return data


def sample_key(sample: LateralSample) -> tuple[str, int]:
  return sample.route, sample.segment


def summarize(samples: list[LateralSample], commits: list[str], expected_commit: str) -> dict[str, Any]:
  temp = [s for s in samples if s.steer_fault_temporary]
  temp_lat = [s for s in temp if s.lat_active]
  high_speed_temp_lat = [s for s in temp_lat if s.v_ego >= HIGH_SPEED_FAULT_MPS]
  low_speed_alerts = [s for s in samples if s.low_speed_alert]
  covered_override = [
    s for s in temp_lat
    if s.steering_pressed and s.override_release_scale <= 0.05
  ]
  partial_override = [
    s for s in temp_lat
    if s.steering_pressed and 0.05 < s.override_release_scale < 1.0
  ]
  uncovered = [
    s for s in temp_lat
    if not s.steering_pressed or s.override_release_scale >= 1.0
  ]
  manual_candidates = [s for s in samples if s.manual_turn_guard_candidate]
  eps_near = [s for s in samples if s.eps_guard_near_limit]

  eps_risk_bursts: list[dict[str, Any]] = []
  eps_guard_sim_events: list[dict[str, Any]] = []
  eps_guard_sim_active_frames = 0
  eps_guard_sim_below_threshold_frames = 0
  manual_guard_sim_active_frames = 0
  max_eps_guard_sim_high_torque_frames = 0
  active_count: dict[tuple[str, int], int] = {}
  eps_sim_state: dict[tuple[str, int], tuple[int, int]] = {}
  manual_sim_state: dict[tuple[str, int], tuple[int, int]] = {}
  manual_sim_active_ids: set[int] = set()
  manual_sim_events: list[dict[str, Any]] = []
  for sample in samples:
    key = sample_key(sample)
    if sample.eps_guard_near_limit:
      active_count[key] = active_count.get(key, 0) + 1
      if active_count[key] == CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES:
        eps_risk_bursts.append(event_dict(sample))
    else:
      active_count[key] = max(active_count.get(key, 0) - 2, 0)

    high_torque_frames, guard_frames = eps_sim_state.get(key, (0, 0))
    guarded_torque, high_torque_frames, guard_frames, guard_active, near_limit = apply_carnival_4th_gen_eps_fault_guard(
      CAR.KIA_CARNIVAL_4TH_GEN, sample.output_torque_units, CARNIVAL_STEER_MAX, sample.v_ego,
      sample.steering_angle_deg, sample.lat_active, sample.steering_pressed, sample.output_torque_units,
      high_torque_frames, guard_frames,
    )
    eps_sim_state[key] = (high_torque_frames, guard_frames)
    max_eps_guard_sim_high_torque_frames = max(max_eps_guard_sim_high_torque_frames, high_torque_frames)
    if guard_active:
      eps_guard_sim_active_frames += 1
      if abs(guarded_torque) / CARNIVAL_STEER_MAX < CARNIVAL_4TH_GEN_EPS_GUARD_TORQUE_FRACTION:
        eps_guard_sim_below_threshold_frames += 1
      if near_limit and len(eps_guard_sim_events) < 12:
        eps_guard_sim_events.append(eps_guard_event_dict(sample, guarded_torque, high_torque_frames, guard_frames))

    manual_guard_frames, manual_last_torque = manual_sim_state.get(key, (0, sample.output_torque_units))
    manual_guarded_torque, manual_guard_active, manual_guard_frames = apply_carnival_4th_gen_manual_turn_torque_guard(
      CAR.KIA_CARNIVAL_4TH_GEN, sample.output_torque_units, CARNIVAL_STEER_MAX, sample.v_ego,
      sample.steering_angle_deg, sample.steering_torque, sample.steering_pressed, manual_last_torque,
      manual_guard_frames,
    )
    manual_sim_state[key] = (manual_guard_frames, manual_guarded_torque if manual_guard_active else sample.output_torque_units)
    if manual_guard_active:
      manual_guard_sim_active_frames += 1
      manual_sim_active_ids.add(id(sample))
      if sample.steer_fault_temporary and len(manual_sim_events) < 12:
        data = event_dict(sample)
        data["manualGuardedTorqueUnits"] = manual_guarded_torque
        data["manualGuardFrames"] = manual_guard_frames
        manual_sim_events.append(data)

  covered_by_manual_sim = [s for s in temp_lat if id(s) in manual_sim_active_ids]
  zero_output_temp = [s for s in temp_lat if abs(s.output_torque_units) <= CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_RELEASE_FRAMES]
  uncovered_after_manual_sim = [
    s for s in temp_lat
    if (
      id(s) not in manual_sim_active_ids and
      abs(s.output_torque_units) > CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_RELEASE_FRAMES and
      not (s.steering_pressed and s.override_release_scale <= 0.05)
    )
  ]

  near_fault_precursors = []
  for fault in temp_lat:
    window = [
      s for s in samples
      if sample_key(s) == sample_key(fault) and 0.0 <= fault.t - s.t <= NEAR_FAULT_WINDOW_S
    ]
    if window:
      near_fault_precursors.append({
        "fault": event_dict(fault),
        "maxAbsOutputTorqueUnitsBeforeFault": max(abs(s.output_torque_units) for s in window),
        "epsNearLimitFramesBeforeFault": sum(1 for s in window if s.eps_guard_near_limit),
        "manualTurnGuardCandidatesBeforeFault": sum(1 for s in window if s.manual_turn_guard_candidate),
      })

  matching_commits = [commit for commit in commits if commit == expected_commit]
  status = "pass"
  if uncovered_after_manual_sim or high_speed_temp_lat:
    status = "fail"
  elif not matching_commits or low_speed_alerts or eps_risk_bursts:
    status = "warn"

  return {
    "status": status,
    "expectedCommit": expected_commit,
    "logCommits": sorted(set(commits)),
    "matchingCommitFiles": len(matching_commits),
    "sampleFrames": len(samples),
    "tempFaultFrames": len(temp),
    "tempFaultLatActiveFrames": len(temp_lat),
    "highSpeedTempFaultLatActiveFrames": len(high_speed_temp_lat),
    "lowSpeedAlertFrames": len(low_speed_alerts),
    "coveredByStrongDriverOverrideFrames": len(covered_override),
    "partialDriverOverrideFrames": len(partial_override),
    "uncoveredLatActiveTempFaultFrames": len(uncovered),
    "manualTurnGuardSimCoveredTempFaultFrames": len(covered_by_manual_sim),
    "zeroOutputTempFaultFrames": len(zero_output_temp),
    "uncoveredAfterManualTurnGuardSimFrames": len(uncovered_after_manual_sim),
    "manualTurnGuardCandidateFrames": len(manual_candidates),
    "epsNearLimitFrames": len(eps_near),
    "epsRiskBursts": eps_risk_bursts[:12],
    "epsGuardSimulation": {
      "activeFrames": eps_guard_sim_active_frames,
      "belowNearLimitThresholdFrames": eps_guard_sim_below_threshold_frames,
      "maxHighTorqueFrames": max_eps_guard_sim_high_torque_frames,
      "triggerFrames": CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES,
      "events": eps_guard_sim_events[:12],
    },
    "manualTurnGuardSimulation": {
      "activeFrames": manual_guard_sim_active_frames,
      "events": manual_sim_events[:12],
    },
    "highSpeedFaultExamples": [event_dict(s) for s in high_speed_temp_lat[:12]],
    "uncoveredExamples": [event_dict(s) for s in uncovered[:12]],
    "uncoveredAfterManualTurnGuardSimExamples": [event_dict(s) for s in uncovered_after_manual_sim[:12]],
    "coveredExamples": [event_dict(s) for s in covered_override[:12]],
    "nearFaultPrecursors": near_fault_precursors[:20],
  }


def console_summary(report: dict[str, Any]) -> dict[str, Any]:
  keys = (
    "status",
    "expectedCommit",
    "logCommits",
    "matchingCommitFiles",
    "logFiles",
    "scanMode",
    "sampleFrames",
    "tempFaultFrames",
    "tempFaultLatActiveFrames",
    "highSpeedTempFaultLatActiveFrames",
    "lowSpeedAlertFrames",
    "coveredByStrongDriverOverrideFrames",
    "partialDriverOverrideFrames",
    "uncoveredLatActiveTempFaultFrames",
    "manualTurnGuardSimCoveredTempFaultFrames",
    "zeroOutputTempFaultFrames",
    "uncoveredAfterManualTurnGuardSimFrames",
    "manualTurnGuardCandidateFrames",
    "epsNearLimitFrames",
  )
  summary = {key: report.get(key) for key in keys if key in report}
  summary["epsRiskBurstCount"] = len(report.get("epsRiskBursts", []))
  eps_guard_sim = report.get("epsGuardSimulation", {})
  summary["epsGuardSimActiveFrames"] = eps_guard_sim.get("activeFrames")
  summary["epsGuardSimBelowThresholdFrames"] = eps_guard_sim.get("belowNearLimitThresholdFrames")
  summary["epsGuardSimMaxHighTorqueFrames"] = eps_guard_sim.get("maxHighTorqueFrames")
  summary["epsGuardSimTriggerFrames"] = eps_guard_sim.get("triggerFrames")
  summary["highSpeedFaultExampleCount"] = len(report.get("highSpeedFaultExamples", []))
  summary["uncoveredExampleCount"] = len(report.get("uncoveredExamples", []))
  return summary


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--out", type=Path)
  parser.add_argument("--include-lat-active-frames", action="store_true",
                      help="Store every latActive frame. Slower; default stores only fault/risk frames.")
  parser.add_argument("--summary-only", action="store_true",
                      help="Print a concise summary while still writing the full JSON report to --out.")
  parser.add_argument("logs", nargs="+", type=Path)
  args = parser.parse_args()

  all_samples: list[LateralSample] = []
  commits: list[str] = []
  logs = expand_log_paths(args.logs)
  for path in logs:
    samples, commit = read_lateral_samples(path, ReadMode.AUTO_INTERACTIVE, args.include_lat_active_frames)
    all_samples.extend(samples)
    commits.append(commit)

  report = summarize(all_samples, commits, current_commit())
  report["logFiles"] = len(logs)
  report["scanMode"] = "lat_active_full" if args.include_lat_active_frames else "event_risk_only"
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.out:
    args.out.write_text(text + "\n", encoding="utf-8")
  print(json.dumps(console_summary(report) if args.summary_only else report, indent=2, sort_keys=True))


if __name__ == "__main__":
  main()
