#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode


CARNIVAL_STEER_MAX = 409
DT_CTRL = 0.01
HIGHWAY_SPEED_MPS = 20.0
LOW_SPEED_MPS = 8.8
STRONG_DRIVER_TORQUE = 150.0
NEAR_LIMIT_TORQUE_FRAC = 0.92
SATURATED_TORQUE_FRAC = 0.98
PING_PONG_MIN_SPEED = 12.0
PING_PONG_MIN_TORQUE_UNITS = 80
PING_PONG_MAX_INTERVAL_S = 1.2
CURVATURE_ERROR_WARN = 0.0015
LAT_ACCEL_ERROR_WARN = 0.35
SATURATED_UNDERTRACK_LAT_ACCEL_ERROR = 0.35
SATURATION_BURST_MIN_FRAMES = 20


@dataclass
class QualitySample:
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
  steering_angle_deg: float
  steering_rate_deg: float
  output_torque_units: int
  desired_curvature: float
  actual_curvature: float
  curvature_error: float
  desired_lat_accel: float
  actual_lat_accel: float
  lat_accel_error: float


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


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  idx = min(max(int(math.ceil((pct / 100.0) * len(ordered))) - 1, 0), len(ordered) - 1)
  return ordered[idx]


def event_dict(sample: QualitySample) -> dict[str, Any]:
  data = asdict(sample)
  for key, value in list(data.items()):
    if isinstance(value, float):
      data[key] = round(value, 4)
  return data


def read_quality_samples(path: Path, mode: ReadMode) -> tuple[list[QualitySample], str]:
  latest: dict[str, Any] = {}
  samples: list[QualitySample] = []
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

    lat_active = safe_bool(safe_attr(car_control, "latActive", False))
    steer_fault_temporary = safe_bool(safe_attr(car_state, "steerFaultTemporary", False))
    low_speed_alert = safe_bool(safe_attr(car_state, "lowSpeedAlert", False))
    steering_pressed = safe_bool(safe_attr(car_state, "steeringPressed", False))
    v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
    output_torque = safe_float(safe_attr(out_actuators, "torque", safe_attr(actuators, "torque", 0.0)))
    desired_curvature = safe_float(safe_attr(controls_state, "desiredCurvature", safe_attr(actuators, "curvature", 0.0)))
    actual_curvature = safe_float(safe_attr(controls_state, "curvature", 0.0))
    curvature_error = desired_curvature - actual_curvature
    desired_lat_accel = desired_curvature * v_ego * v_ego
    actual_lat_accel = actual_curvature * v_ego * v_ego

    interesting = (
      lat_active or steer_fault_temporary or low_speed_alert or steering_pressed or
      abs(output_torque) >= 0.35
    )
    if not interesting:
      continue

    samples.append(QualitySample(
      route=route_name(path),
      segment=segment_number(path),
      t=(mono_time - start_ns) / 1e9,
      git_commit=git_commit,
      v_ego=v_ego,
      lat_active=lat_active,
      enabled=safe_bool(safe_attr(controls_state, "enabled", False)),
      steer_fault_temporary=steer_fault_temporary,
      low_speed_alert=low_speed_alert,
      steering_pressed=steering_pressed,
      steering_torque=safe_float(safe_attr(car_state, "steeringTorque", 0.0)),
      steering_angle_deg=safe_float(safe_attr(car_state, "steeringAngleDeg", 0.0)),
      steering_rate_deg=safe_float(safe_attr(car_state, "steeringRateDeg", 0.0)),
      output_torque_units=int(round(output_torque * CARNIVAL_STEER_MAX)),
      desired_curvature=desired_curvature,
      actual_curvature=actual_curvature,
      curvature_error=curvature_error,
      desired_lat_accel=desired_lat_accel,
      actual_lat_accel=actual_lat_accel,
      lat_accel_error=desired_lat_accel - actual_lat_accel,
    ))

  return samples, git_commit


def sample_key(sample: QualitySample) -> tuple[str, int]:
  return sample.route, sample.segment


def summarize_ping_pong(samples: list[QualitySample]) -> dict[str, Any]:
  events: list[dict[str, Any]] = []
  last_sign: dict[tuple[str, int], int] = {}
  last_time: dict[tuple[str, int], float] = {}
  counts: dict[tuple[str, int], int] = {}

  for sample in samples:
    if not sample.lat_active or sample.steering_pressed or sample.v_ego < PING_PONG_MIN_SPEED:
      continue
    if abs(sample.output_torque_units) < PING_PONG_MIN_TORQUE_UNITS:
      continue
    sign = 1 if sample.output_torque_units > 0 else -1
    key = sample_key(sample)
    if key in last_sign and sign != last_sign[key] and sample.t - last_time.get(key, -999.0) <= PING_PONG_MAX_INTERVAL_S:
      counts[key] = counts.get(key, 0) + 1
      if len(events) < 12:
        events.append(event_dict(sample))
    last_sign[key] = sign
    last_time[key] = sample.t

  return {
    "events": events,
    "segments": [{"route": k[0], "segment": k[1], "count": v} for k, v in sorted(counts.items(), key=lambda item: -item[1])[:12]],
    "totalEvents": sum(counts.values()),
  }


def summarize_saturation_bursts(samples: list[QualitySample]) -> dict[str, Any]:
  bursts: list[dict[str, Any]] = []
  current_key: tuple[str, int] | None = None
  current_start: QualitySample | None = None
  current_last: QualitySample | None = None
  current_frames = 0
  current_max_error = 0.0

  def close_burst() -> None:
    nonlocal current_key, current_start, current_last, current_frames, current_max_error
    if current_key is not None and current_start is not None and current_last is not None and current_frames >= SATURATION_BURST_MIN_FRAMES:
      bursts.append({
        "route": current_key[0],
        "segment": current_key[1],
        "startT": round(current_start.t, 3),
        "endT": round(current_last.t, 3),
        "durationS": round(max(current_last.t - current_start.t, 0.0), 3),
        "frames": current_frames,
        "startVEgo": round(current_start.v_ego, 3),
        "endVEgo": round(current_last.v_ego, 3),
        "maxLatAccelError": round(current_max_error, 4),
        "start": event_dict(current_start),
      })
    current_key = None
    current_start = None
    current_last = None
    current_frames = 0
    current_max_error = 0.0

  for sample in samples:
    key = sample_key(sample)
    saturated = sample.lat_active and abs(sample.output_torque_units) >= round(CARNIVAL_STEER_MAX * SATURATED_TORQUE_FRAC)
    if saturated:
      if key != current_key:
        close_burst()
        current_key = key
        current_start = sample
        current_frames = 0
        current_max_error = 0.0
      current_last = sample
      current_frames += 1
      current_max_error = max(current_max_error, abs(sample.lat_accel_error))
    else:
      close_burst()
  close_burst()

  return {
    "minFrames": SATURATION_BURST_MIN_FRAMES,
    "totalBursts": len(bursts),
    "maxDurationS": None if not bursts else max(b["durationS"] for b in bursts),
    "maxFrames": 0 if not bursts else max(b["frames"] for b in bursts),
    "bursts": sorted(bursts, key=lambda b: (-b["frames"], -b["maxLatAccelError"]))[:12],
  }


def summarize(samples: list[QualitySample], commits: list[str], expected_commit: str, log_files: int) -> dict[str, Any]:
  lat = [s for s in samples if s.lat_active]
  enabled_lat = [s for s in lat if s.enabled]
  highway_lat = [s for s in lat if s.v_ego >= HIGHWAY_SPEED_MPS]
  low_speed_lat = [s for s in lat if s.v_ego <= LOW_SPEED_MPS]
  temp_lat = [s for s in lat if s.steer_fault_temporary]
  low_speed_alerts = [s for s in samples if s.low_speed_alert]
  driver_interventions = [
    s for s in enabled_lat
    if s.steering_pressed and abs(s.steering_torque) >= STRONG_DRIVER_TORQUE
  ]
  near_limit = [s for s in lat if abs(s.output_torque_units) >= round(CARNIVAL_STEER_MAX * NEAR_LIMIT_TORQUE_FRAC)]
  saturated = [s for s in lat if abs(s.output_torque_units) >= round(CARNIVAL_STEER_MAX * SATURATED_TORQUE_FRAC)]
  saturated_undertrack = [
    s for s in saturated
    if s.v_ego >= 3.0 and abs(s.lat_accel_error) >= SATURATED_UNDERTRACK_LAT_ACCEL_ERROR
  ]
  low_speed_saturated_undertrack = [s for s in saturated_undertrack if s.v_ego <= LOW_SPEED_MPS]
  highway_saturated_undertrack = [s for s in saturated_undertrack if s.v_ego >= HIGHWAY_SPEED_MPS]
  curvature_errors = [abs(s.curvature_error) for s in lat if s.v_ego >= 3.0]
  highway_curvature_errors = [abs(s.curvature_error) for s in highway_lat]
  lat_accel_errors = [abs(s.lat_accel_error) for s in lat if s.v_ego >= 3.0]
  highway_lat_accel_errors = [abs(s.lat_accel_error) for s in highway_lat]
  ping_pong = summarize_ping_pong(lat)
  saturation_bursts = summarize_saturation_bursts(lat)
  matching_commits = [commit for commit in commits if commit == expected_commit]

  status = "pass"
  if temp_lat or low_speed_alerts:
    status = "fail"
  elif driver_interventions or saturated_undertrack or saturated or ping_pong["totalEvents"] > 0:
    status = "warn"
  elif not matching_commits:
    status = "warn"

  return {
    "status": status,
    "expectedCommit": expected_commit,
    "logCommits": sorted(set(commits)),
    "matchingCommitFiles": len(matching_commits),
    "logFiles": log_files,
    "sampleFrames": len(samples),
    "latActiveFrames": len(lat),
    "enabledLatActiveFrames": len(enabled_lat),
    "highwayLatActiveFrames": len(highway_lat),
    "lowSpeedLatActiveFrames": len(low_speed_lat),
    "tempFaultLatActiveFrames": len(temp_lat),
    "lowSpeedAlertFrames": len(low_speed_alerts),
    "strongDriverInterventionFrames": len(driver_interventions),
    "nearLimitTorqueFrames": len(near_limit),
    "saturatedTorqueFrames": len(saturated),
    "saturatedUndertrackFrames": len(saturated_undertrack),
    "lowSpeedSaturatedUndertrackFrames": len(low_speed_saturated_undertrack),
    "highwaySaturatedUndertrackFrames": len(highway_saturated_undertrack),
    "curvatureError": {
      "mean": None if not curvature_errors else round(mean(curvature_errors), 6),
      "p95": None if not curvature_errors else round(percentile(curvature_errors, 95.0), 6),
      "p99": None if not curvature_errors else round(percentile(curvature_errors, 99.0), 6),
      "highwayP95": None if not highway_curvature_errors else round(percentile(highway_curvature_errors, 95.0), 6),
      "warnFrames": sum(1 for value in curvature_errors if value >= CURVATURE_ERROR_WARN),
    },
    "latAccelError": {
      "mean": None if not lat_accel_errors else round(mean(lat_accel_errors), 4),
      "p95": None if not lat_accel_errors else round(percentile(lat_accel_errors, 95.0), 4),
      "p99": None if not lat_accel_errors else round(percentile(lat_accel_errors, 99.0), 4),
      "highwayP95": None if not highway_lat_accel_errors else round(percentile(highway_lat_accel_errors, 95.0), 4),
      "warnFrames": sum(1 for value in lat_accel_errors if value >= LAT_ACCEL_ERROR_WARN),
    },
    "pingPong": ping_pong,
    "saturationBursts": saturation_bursts,
    "examples": {
      "tempFaultLatActive": [event_dict(s) for s in temp_lat[:12]],
      "lowSpeedAlerts": [event_dict(s) for s in low_speed_alerts[:12]],
      "strongDriverInterventions": [event_dict(s) for s in driver_interventions[:12]],
      "saturatedTorque": [event_dict(s) for s in saturated[:12]],
      "saturatedUndertrack": [event_dict(s) for s in saturated_undertrack[:12]],
    },
  }


def console_summary(report: dict[str, Any]) -> dict[str, Any]:
  return {
    "status": report["status"],
    "expectedCommit": report["expectedCommit"],
    "logCommits": report["logCommits"],
    "matchingCommitFiles": report["matchingCommitFiles"],
    "logFiles": report["logFiles"],
    "latActiveFrames": report["latActiveFrames"],
    "enabledLatActiveFrames": report["enabledLatActiveFrames"],
    "tempFaultLatActiveFrames": report["tempFaultLatActiveFrames"],
    "lowSpeedAlertFrames": report["lowSpeedAlertFrames"],
    "strongDriverInterventionFrames": report["strongDriverInterventionFrames"],
    "nearLimitTorqueFrames": report["nearLimitTorqueFrames"],
    "saturatedTorqueFrames": report["saturatedTorqueFrames"],
    "saturatedUndertrackFrames": report["saturatedUndertrackFrames"],
    "lowSpeedSaturatedUndertrackFrames": report["lowSpeedSaturatedUndertrackFrames"],
    "highwaySaturatedUndertrackFrames": report["highwaySaturatedUndertrackFrames"],
    "saturationBursts": report["saturationBursts"]["totalBursts"],
    "maxSaturationBurstFrames": report["saturationBursts"]["maxFrames"],
    "curvatureErrorP95": report["curvatureError"]["p95"],
    "curvatureErrorHighwayP95": report["curvatureError"]["highwayP95"],
    "latAccelErrorP95": report["latAccelError"]["p95"],
    "latAccelErrorHighwayP95": report["latAccelError"]["highwayP95"],
    "pingPongEvents": report["pingPong"]["totalEvents"],
  }


def main() -> None:
  parser = argparse.ArgumentParser(description="Measure Carnival lateral quality from qlogs.")
  parser.add_argument("--out", type=Path)
  parser.add_argument("--summary-only", action="store_true")
  parser.add_argument("logs", nargs="+", type=Path)
  args = parser.parse_args()

  logs = expand_log_paths(args.logs)
  all_samples: list[QualitySample] = []
  commits: list[str] = []
  for path in logs:
    samples, commit = read_quality_samples(path, ReadMode.AUTO_INTERACTIVE)
    all_samples.extend(samples)
    commits.append(commit)

  report = summarize(all_samples, commits, current_commit(), len(logs))
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.out:
    args.out.write_text(text + "\n", encoding="utf-8")
  print(json.dumps(console_summary(report) if args.summary_only else report, indent=2, sort_keys=True))


if __name__ == "__main__":
  main()
