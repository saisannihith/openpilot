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
from scan_longitudinal_quality import analyze as analyze_longitudinal
from scan_longitudinal_quality import expand_logs, read_samples_and_metadata


@dataclass
class SteeringFaultSample:
  route: str
  segment: int
  t: float
  v_ego: float
  steer_fault_temporary: bool
  low_speed_alert: bool
  lat_active: bool
  enabled: bool
  steering_pressed: bool
  steering_torque: float
  steering_torque_eps: float
  commanded_torque: float
  output_torque: float
  steering_angle_deg: float


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


def current_commit() -> str:
  try:
    return subprocess.check_output(["git", "rev-parse", "origin/snithpilot"], text=True).strip()
  except Exception:
    try:
      return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    except Exception:
      return "unknown"


def read_steering_fault_samples(path: Path, mode: ReadMode) -> list[SteeringFaultSample]:
  latest: dict[str, Any] = {}
  samples: list[SteeringFaultSample] = []
  start_ns: int | None = None
  for msg in LogReader(str(path), default_mode=mode, sort_by_time=True):
    which = msg.which()
    mono_time = int(msg.logMonoTime)
    if start_ns is None and which in ("carState", "carControl", "carOutput", "controlsState"):
      start_ns = mono_time
    if which in ("carState", "carControl", "carOutput", "controlsState"):
      latest[which] = getattr(msg, which)
    if start_ns is None or "carState" not in latest:
      continue

    car_state = latest["carState"]
    steer_fault_temporary = bool(safe_attr(car_state, "steerFaultTemporary", False))
    low_speed_alert = bool(safe_attr(car_state, "lowSpeedAlert", False))
    if not steer_fault_temporary and not low_speed_alert:
      continue

    car_control = latest.get("carControl")
    car_output = latest.get("carOutput")
    controls_state = latest.get("controlsState")
    actuators = safe_attr(car_control, "actuators")
    out_actuators = safe_attr(car_output, "actuatorsOutput") if car_output is not None else None
    steering_torque = safe_float(safe_attr(car_state, "steeringTorque", 0.0))
    v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
    samples.append(SteeringFaultSample(
      route=route_name(path),
      segment=segment_number(path),
      t=(mono_time - start_ns) / 1e9,
      v_ego=v_ego,
      steer_fault_temporary=steer_fault_temporary,
      low_speed_alert=low_speed_alert,
      lat_active=bool(safe_attr(car_control, "latActive", False)),
      enabled=bool(safe_attr(controls_state, "enabled", False)),
      steering_pressed=bool(safe_attr(car_state, "steeringPressed", False)),
      steering_torque=steering_torque,
      steering_torque_eps=safe_float(safe_attr(car_state, "steeringTorqueEps", 0.0)),
      commanded_torque=safe_float(safe_attr(actuators, "torque", 0.0)),
      output_torque=safe_float(safe_attr(out_actuators, "torque", safe_attr(actuators, "torque", 0.0))),
      steering_angle_deg=safe_float(safe_attr(car_state, "steeringAngleDeg", 0.0)),
    ))
  return samples


def event_dict(sample: SteeringFaultSample) -> dict[str, Any]:
  data = asdict(sample)
  for key, value in list(data.items()):
    if isinstance(value, float):
      data[key] = round(value, 3)
  return data


def summarize_steering_faults(samples: list[SteeringFaultSample]) -> dict[str, Any]:
  temp = [s for s in samples if s.steer_fault_temporary]
  temp_lat = [s for s in temp if s.lat_active]
  low_speed = [s for s in samples if s.low_speed_alert]
  driver_override = [s for s in temp_lat if s.steering_pressed]
  spontaneous = [s for s in temp_lat if not s.steering_pressed]
  return {
    "tempFaultFrames": len(temp),
    "tempFaultLatActiveFrames": len(temp_lat),
    "lowSpeedAlertFrames": len(low_speed),
    "driverOverrideLatActiveTempFaultFrames": len(driver_override),
    "spontaneousLatActiveTempFaultFrames": len(spontaneous),
    "maxFaultSpeedMps": None if not temp else round(max(s.v_ego for s in temp), 3),
    "maxFaultDriverTorque": None if not temp else round(max(abs(s.steering_torque) for s in temp), 3),
    "driverOverrideExamples": [event_dict(s) for s in driver_override[:8]],
    "spontaneousExamples": [event_dict(s) for s in spontaneous[:8]],
  }


def add_check(checks: list[dict[str, Any]], name: str, status: str, detail: str, evidence: Any) -> None:
  checks.append({
    "name": name,
    "status": status,
    "detail": detail,
    "evidence": evidence,
  })


def build_report(logs: list[Path], include_stale: bool) -> dict[str, Any]:
  mode = ReadMode.AUTO_INTERACTIVE
  all_long_samples = []
  all_metadata = []
  all_steering_samples = []
  for path in logs:
    long_samples, metadata = read_samples_and_metadata(path, mode)
    all_long_samples.extend(long_samples)
    if metadata is not None:
      all_metadata.append(metadata)
    all_steering_samples.extend(read_steering_fault_samples(path, mode))

  commit = current_commit()
  matching_files = [
    metadata for metadata in all_metadata
    if metadata.get("gitCommit") == commit
  ]
  if not include_stale and all_metadata and not matching_files:
    scoring_long_samples = []
    scoring_steering_samples = []
  else:
    scoring_long_samples = all_long_samples
    scoring_steering_samples = all_steering_samples

  long_summary = analyze_longitudinal(scoring_long_samples, all_metadata)
  steering_summary = summarize_steering_faults(scoring_steering_samples)
  checks: list[dict[str, Any]] = []

  if all_metadata and not matching_files:
    add_check(
      checks,
      "current_commit_log_coverage",
      "warn" if include_stale else "fail",
      f"{len(matching_files)} qlog files match current origin/snithpilot commit {commit}; include_stale={include_stale}.",
      {"currentCommit": commit, "logSoftware": long_summary.get("software", [])},
    )
  else:
    add_check(
      checks,
      "current_commit_log_coverage",
      "pass",
      f"{len(matching_files)} qlog files match current origin/snithpilot commit {commit}.",
      {"currentCommit": commit, "matchingFiles": len(matching_files)},
    )

  no_context = long_summary.get("noContextHighwayHardBrakes", [])
  add_check(
    checks,
    "no_context_highway_hard_brakes",
    "pass" if not no_context else "fail",
    f"{len(no_context)} hard-brake samples without lead/stop context.",
    no_context[:8],
  )

  jumps = long_summary.get("accelJumps", [])
  add_check(
    checks,
    "planner_accel_jumps",
    "pass" if not jumps else "fail",
    f"{len(jumps)} large planner acceleration jumps.",
    jumps[:8],
  )

  temp_lat = steering_summary["tempFaultLatActiveFrames"]
  spontaneous = steering_summary["spontaneousLatActiveTempFaultFrames"]
  add_check(
    checks,
    "steering_temp_faults_while_active",
    "pass" if temp_lat == 0 else "fail",
    f"{temp_lat} temporary steering fault frames overlapped latActive; {spontaneous} began without steeringPressed.",
    {
      "driverOverrideExamples": steering_summary["driverOverrideExamples"],
      "spontaneousExamples": steering_summary["spontaneousExamples"],
    },
  )

  stop_releases = long_summary.get("stopReleaseOpportunities", [])
  lead_departures = long_summary.get("leadDepartureOpportunities", [])
  add_check(
    checks,
    "stop_and_go_coverage",
    "pass" if stop_releases or lead_departures else "warn",
    f"{len(stop_releases)} stop-release opportunities; {len(lead_departures)} lead-departure opportunities.",
    {
      "stopReleaseOpportunities": stop_releases[:5],
      "leadDepartureOpportunities": lead_departures[:5],
    },
  )

  status = "pass"
  if any(check["status"] == "fail" for check in checks):
    status = "fail"
  elif any(check["status"] == "warn" for check in checks):
    status = "warn"

  return {
    "status": status,
    "checks": checks,
    "currentCommit": commit,
    "includeStaleLogs": include_stale,
    "filesScanned": len(logs),
    "routes": sorted(set(long_summary.get("routes", []))),
    "samples": long_summary.get("samples", 0),
    "longitudinal": long_summary,
    "steering": steering_summary,
    "nextDriveRequests": [] if status == "pass" else [
      "Drive on the current origin/snithpilot commit with longitudinal alpha active.",
      "Include one neighborhood/low-speed section with normal manual turns.",
      "Include one yellow/red-light approach where you do not brake unless intervention is required.",
      "Include stop-and-go behind a lead vehicle if traffic allows.",
    ],
  }


def main() -> int:
  parser = argparse.ArgumentParser(description="Combined Carnival steering and longitudinal readiness report.")
  parser.add_argument("logs", nargs="*", help="qlog paths/globs.")
  parser.add_argument("--recent-routes", type=int, default=2, help="Newest route ids to scan when logs are omitted on-device.")
  parser.add_argument("--include-stale", action="store_true", help="Include logs from older commits in scoring.")
  parser.add_argument("--out", help="Optional JSON output path.")
  args = parser.parse_args()

  patterns = args.logs
  if not patterns:
    realdata = Path("/data/media/0/realdata")
    if realdata.exists():
      route_ids = sorted({"--".join(path.name.split("--")[:2]) for path in realdata.glob("*--*--*")})[-args.recent_routes:]
      patterns = [str(realdata / f"{route_id}--*" / "qlog*") for route_id in route_ids]
  logs = expand_logs(patterns)
  report = build_report(logs, args.include_stale)
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
