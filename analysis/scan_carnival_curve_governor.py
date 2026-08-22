#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean
from types import SimpleNamespace
from typing import Any

import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_planner import apply_carnival_lateral_feasibility_speed_cap
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.tools.lib.logreader import LogReader, ReadMode


CARNIVAL_STEER_MAX = 409
MS_TO_MPH = 2.2369362920544
SATURATED_TORQUE_FRAC = 0.98
UNDERTRACK_LAT_ACCEL_ERROR = 0.35
NEAR_GOVERNOR_WINDOW_S = 2.0


@dataclass
class GovernorSample:
  route: str
  segment: int
  t: float
  git_commit: str
  v_ego: float
  lat_active: bool
  long_active: bool
  enabled: bool
  steer_fault_temporary: bool
  steering_pressed: bool
  requested_torque_units: int
  output_torque_units: int
  desired_curvature: float
  raw_model_curvature: float
  final_curvature_extra: float
  actual_curvature: float
  desired_lat_accel: float
  actual_lat_accel: float
  speed_reduction_mps: float
  target_speed_mps: float
  target_time_s: float
  target_curvature: float
  min_target_speed_mps: float
  min_target_index: int
  min_target_time_s: float
  curvature_at_min_target: float


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


def safe_bool(value: Any, default: bool = False) -> bool:
  try:
    return bool(value)
  except Exception:
    return default


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
  value = safe_attr(log_init, "gitCommit")
  return str(value) if value else "unknown"


def get_model_speed_error(model_msg: Any, v_ego: float) -> float:
  temporal_pose = safe_attr(model_msg, "temporalPoseDEPRECATED")
  if temporal_pose is None:
    temporal_pose = safe_attr(model_msg, "temporalPose")
  if temporal_pose is None or not len(safe_attr(temporal_pose, "trans", [])):
    return 0.0
  return float(np.clip(safe_float(temporal_pose.trans[0]) - v_ego, -5.0, 5.0))


def parse_model_speed(model_msg: Any, v_ego: float) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
  if (
    len(model_msg.position.x) != ModelConstants.IDX_N or
    len(model_msg.velocity.x) != ModelConstants.IDX_N or
    len(model_msg.acceleration.x) != ModelConstants.IDX_N
  ):
    return None

  model_error = get_model_speed_error(model_msg, v_ego)
  v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
  a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
  j = np.zeros(len(T_IDXS_MPC))
  return v, a, j


def is_undertracking(sample: GovernorSample) -> bool:
  desired = sample.desired_lat_accel
  actual = sample.actual_lat_accel
  if abs(desired) < UNDERTRACK_LAT_ACCEL_ERROR:
    return False
  wrong_direction = desired * actual <= 0.0
  weak_same_direction = abs(actual) < abs(desired) and desired * actual > 0.0
  return (wrong_direction or weak_same_direction) and abs(desired - actual) >= UNDERTRACK_LAT_ACCEL_ERROR


def event_dict(sample: GovernorSample) -> dict[str, Any]:
  data = asdict(sample)
  data["currentSpeedMph"] = round(sample.v_ego * MS_TO_MPH, 1)
  data["targetSpeedMph"] = round(sample.target_speed_mps * MS_TO_MPH, 1)
  data["minTargetSpeedMph"] = round(sample.min_target_speed_mps * MS_TO_MPH, 1)
  data["speedReductionMph"] = round(sample.speed_reduction_mps * MS_TO_MPH, 1)
  for key, value in list(data.items()):
    if isinstance(value, float):
      data[key] = round(value, 4)
  return data


def read_governor_samples(path: Path, mode: ReadMode) -> tuple[list[GovernorSample], int, str]:
  latest: dict[str, Any] = {}
  samples: list[GovernorSample] = []
  start_ns: int | None = None
  git_commit = "unknown"
  model_frames = 0
  cp = SimpleNamespace(carFingerprint="KIA_CARNIVAL_4TH_GEN")

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
      continue

    if which != "modelV2" or start_ns is None or "carState" not in latest:
      continue

    model_msg = msg.modelV2
    car_state = latest["carState"]
    v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
    parsed = parse_model_speed(model_msg, v_ego)
    if parsed is None:
      continue
    model_frames += 1

    v, a, j = parsed
    model_yaw_rate = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)
    curvature = model_yaw_rate / np.clip(v, 0.3, 100.0)
    raw_model_curvature = float(curvature[0])

    car_control = latest.get("carControl")
    controls_state = latest.get("controlsState")
    lat_active = safe_bool(safe_attr(car_control, "latActive", False))
    current_desired_curvature = safe_attr(controls_state, "desiredCurvature") if lat_active else None
    capped_v, _, _ = apply_carnival_lateral_feasibility_speed_cap(
      cp, model_msg, v_ego, v, a, j, current_desired_curvature)
    reductions = np.maximum(v - capped_v, 0.0)
    max_reduction = float(np.max(reductions))
    if max_reduction <= 1e-3:
      continue

    target_idx = int(np.argmax(reductions))
    min_idx = int(np.argmin(capped_v))

    car_output = latest.get("carOutput")
    actuators = safe_attr(car_control, "actuators")
    out_actuators = safe_attr(car_output, "actuatorsOutput") if car_output is not None else None
    lateral_state = safe_attr(controls_state, "lateralControlState") if controls_state is not None else None
    lateral_log = None
    if lateral_state is not None:
      try:
        lateral_log = getattr(lateral_state, lateral_state.which())
      except Exception:
        lateral_log = None

    output_torque = safe_float(safe_attr(out_actuators, "torque", safe_attr(actuators, "torque", 0.0)))
    requested_torque = safe_float(safe_attr(lateral_log, "output", output_torque))
    desired_curvature = safe_float(safe_attr(controls_state, "desiredCurvature", safe_attr(actuators, "curvature", 0.0)))
    actual_curvature = safe_float(safe_attr(controls_state, "curvature", 0.0))
    final_curvature_extra = max(abs(desired_curvature) - abs(raw_model_curvature), 0.0)

    samples.append(GovernorSample(
      route=route_name(path),
      segment=segment_number(path),
      t=(mono_time - start_ns) / 1e9,
      git_commit=git_commit,
      v_ego=v_ego,
      lat_active=safe_bool(safe_attr(car_control, "latActive", False)),
      long_active=safe_bool(safe_attr(car_control, "longActive", False)),
      enabled=safe_bool(safe_attr(controls_state, "enabled", False)),
      steer_fault_temporary=safe_bool(safe_attr(car_state, "steerFaultTemporary", False)),
      steering_pressed=safe_bool(safe_attr(car_state, "steeringPressed", False)),
      requested_torque_units=int(round(requested_torque * CARNIVAL_STEER_MAX)),
      output_torque_units=int(round(output_torque * CARNIVAL_STEER_MAX)),
      desired_curvature=desired_curvature,
      raw_model_curvature=raw_model_curvature,
      final_curvature_extra=final_curvature_extra,
      actual_curvature=actual_curvature,
      desired_lat_accel=desired_curvature * v_ego * v_ego,
      actual_lat_accel=actual_curvature * v_ego * v_ego,
      speed_reduction_mps=max_reduction,
      target_speed_mps=float(capped_v[target_idx]),
      target_time_s=float(T_IDXS_MPC[target_idx]),
      target_curvature=float(curvature[target_idx]),
      min_target_speed_mps=float(capped_v[min_idx]),
      min_target_index=min_idx,
      min_target_time_s=float(T_IDXS_MPC[min_idx]),
      curvature_at_min_target=float(curvature[min_idx]),
    ))

  return samples, model_frames, git_commit


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  idx = min(max(int(math.ceil((pct / 100.0) * len(ordered))) - 1, 0), len(ordered) - 1)
  return ordered[idx]


def summarize(samples: list[GovernorSample], model_frames: int, commits: list[str], expected_commit: str, log_files: int) -> dict[str, Any]:
  lat_active = [s for s in samples if s.lat_active]
  long_active = [s for s in samples if s.long_active]
  temp_fault = [s for s in samples if s.steer_fault_temporary]
  saturated_undertrack = [
    s for s in samples
    if s.lat_active and abs(s.output_torque_units) >= round(CARNIVAL_STEER_MAX * SATURATED_TORQUE_FRAC) and is_undertracking(s)
  ]
  governor_times: dict[tuple[str, int], list[float]] = {}
  for sample in samples:
    governor_times.setdefault((sample.route, sample.segment), []).append(sample.t)

  saturated_near_governor = []
  for sample in saturated_undertrack:
    nearby = governor_times.get((sample.route, sample.segment), [])
    if any(abs(sample.t - t) <= NEAR_GOVERNOR_WINDOW_S for t in nearby):
      saturated_near_governor.append(sample)

  reductions = [s.speed_reduction_mps for s in samples]
  targets = [s.target_speed_mps for s in samples]
  min_targets = [s.min_target_speed_mps for s in samples]
  final_demand_extra = [s.final_curvature_extra for s in samples if s.final_curvature_extra > 1e-5]
  matching_commits = [commit for commit in commits if commit == expected_commit]
  status = "pass"
  if temp_fault:
    status = "fail"
  elif saturated_near_governor or not matching_commits:
    status = "warn"

  return {
    "status": status,
    "expectedCommit": expected_commit,
    "logCommits": sorted(set(commits)),
    "matchingCommitFiles": len(matching_commits),
    "logFiles": log_files,
    "modelFrames": model_frames,
    "governorActiveFrames": len(samples),
    "latActiveGovernorFrames": len(lat_active),
    "longActiveGovernorFrames": len(long_active),
    "tempFaultGovernorFrames": len(temp_fault),
    "saturatedUndertrackNearGovernorFrames": len(saturated_near_governor),
    "finalDemandExtraGovernorFrames": len(final_demand_extra),
    "nearGovernorWindowS": NEAR_GOVERNOR_WINDOW_S,
    "finalDemandExtra": {
      "maxCurvature": 0.0 if not final_demand_extra else round(max(final_demand_extra), 6),
      "p95Curvature": None if not final_demand_extra else round(percentile(final_demand_extra, 95.0), 6),
    },
    "speedReduction": {
      "maxMps": 0.0 if not reductions else round(max(reductions), 3),
      "maxMph": 0.0 if not reductions else round(max(reductions) * MS_TO_MPH, 1),
      "meanMph": None if not reductions else round(mean(reductions) * MS_TO_MPH, 1),
      "p95Mph": None if not reductions else round(percentile(reductions, 95.0) * MS_TO_MPH, 1),
    },
    "targetSpeed": {
      "minGovernorTargetMph": None if not targets else round(min(targets) * MS_TO_MPH, 1),
      "p05GovernorTargetMph": None if not targets else round(percentile(targets, 5.0) * MS_TO_MPH, 1),
      "minFutureTargetMph": None if not min_targets else round(min(min_targets) * MS_TO_MPH, 1),
      "p05FutureTargetMph": None if not min_targets else round(percentile(min_targets, 5.0) * MS_TO_MPH, 1),
    },
    "examples": {
      "governorActive": [event_dict(s) for s in samples[:16]],
      "largestSpeedReduction": [event_dict(s) for s in sorted(samples, key=lambda s: -s.speed_reduction_mps)[:16]],
      "largestFinalDemandExtra": [event_dict(s) for s in sorted(samples, key=lambda s: -s.final_curvature_extra)[:16] if s.final_curvature_extra > 1e-5],
      "saturatedUndertrackNearGovernor": [event_dict(s) for s in saturated_near_governor[:16]],
      "tempFaultNearGovernor": [event_dict(s) for s in temp_fault[:16]],
    },
  }


def console_summary(report: dict[str, Any]) -> dict[str, Any]:
  return {
    "status": report["status"],
    "expectedCommit": report["expectedCommit"],
    "logCommits": report["logCommits"],
    "matchingCommitFiles": report["matchingCommitFiles"],
    "logFiles": report["logFiles"],
    "modelFrames": report["modelFrames"],
    "governorActiveFrames": report["governorActiveFrames"],
    "latActiveGovernorFrames": report["latActiveGovernorFrames"],
    "longActiveGovernorFrames": report["longActiveGovernorFrames"],
    "tempFaultGovernorFrames": report["tempFaultGovernorFrames"],
    "saturatedUndertrackNearGovernorFrames": report["saturatedUndertrackNearGovernorFrames"],
    "finalDemandExtraGovernorFrames": report["finalDemandExtraGovernorFrames"],
    "finalDemandExtra": report["finalDemandExtra"],
    "speedReduction": report["speedReduction"],
    "targetSpeed": report["targetSpeed"],
  }


def main() -> None:
  parser = argparse.ArgumentParser(description="Measure Carnival curve governor activation from qlogs.")
  parser.add_argument("--out", type=Path)
  parser.add_argument("--summary-only", action="store_true")
  parser.add_argument("logs", nargs="+", type=Path)
  args = parser.parse_args()

  logs = expand_log_paths(args.logs)
  all_samples: list[GovernorSample] = []
  commits: list[str] = []
  model_frames = 0
  for path in logs:
    samples, frame_count, commit = read_governor_samples(path, ReadMode.AUTO_INTERACTIVE)
    all_samples.extend(samples)
    model_frames += frame_count
    commits.append(commit)

  report = summarize(all_samples, model_frames, commits, current_commit(), len(logs))
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.out:
    args.out.write_text(text + "\n", encoding="utf-8")
  print(json.dumps(console_summary(report) if args.summary_only else report, indent=2, sort_keys=True))


if __name__ == "__main__":
  main()
