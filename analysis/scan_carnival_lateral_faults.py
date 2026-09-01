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


CARNIVAL_STEER_MAX = 384
HIGH_SPEED_FAULT_MPS = 17.0
NEAR_FAULT_WINDOW_S = 2.0
STRONG_DRIVER_OVERRIDE_TORQUE = 300.0
EPS_RISK_MIN_SPEED = 8.0
EPS_RISK_MIN_ANGLE = 3.0
EPS_RISK_TORQUE_FRACTION = 0.88
EPS_RISK_TRIGGER_FRAMES = 24
MANUAL_TURN_MONITOR_MAX_SPEED = 10.5
MANUAL_TURN_MONITOR_ANGLE = 35.0


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
  for rev in ("HEAD", "origin/snithpilot"):
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

    # Analyze each physical state update once. Sampling again when carControl,
    # carOutput, or controlsState arrives inflates fault duration and guard
    # persistence because all four messages reuse the same carState.
    if which != "carState":
      continue

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
      v_ego <= MANUAL_TURN_MONITOR_MAX_SPEED and
      abs(steering_angle) >= MANUAL_TURN_MONITOR_ANGLE
    )
    eps_near_limit = (
      lat_active and
      v_ego >= EPS_RISK_MIN_SPEED and
      abs(steering_angle) >= EPS_RISK_MIN_ANGLE and
      torque_fraction >= EPS_RISK_TORQUE_FRACTION
    )

    interesting = (
      (include_lat_active_frames and lat_active) or
      (lat_active and steering_pressed) or
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


def sample_key(sample: LateralSample) -> tuple[str, int]:
  return sample.route, sample.segment


def summarize_fault_episodes(samples: list[LateralSample]) -> list[dict[str, Any]]:
  episodes: list[list[LateralSample]] = []
  current: list[LateralSample] = []
  for sample in samples:
    if not sample.steer_fault_temporary:
      continue
    if (current and
        (sample_key(sample) != sample_key(current[-1]) or sample.t - current[-1].t > 0.15)):
      episodes.append(current)
      current = []
    current.append(sample)
  if current:
    episodes.append(current)

  reports: list[dict[str, Any]] = []
  for episode in episodes:
    first = episode[0]
    key = sample_key(first)
    pre = [s for s in samples if sample_key(s) == key and 0.0 < first.t - s.t <= NEAR_FAULT_WINDOW_S]
    strong_driver_override = [
      s for s in pre
      if s.steering_pressed and abs(s.steering_torque) >= STRONG_DRIVER_OVERRIDE_TORQUE
    ]
    reversals = 0
    previous_sign = 0
    for sample in pre:
      if abs(sample.output_torque_units) < 40:
        continue
      sign = 1 if sample.output_torque_units > 0 else -1
      if previous_sign and sign != previous_sign:
        reversals += 1
      previous_sign = sign
    reports.append({
      "route": first.route,
      "segment": first.segment,
      "startT": round(first.t, 3),
      "endT": round(episode[-1].t, 3),
      "durationS": round(max(episode[-1].t - first.t, 0.0), 3),
      "frames": len(episode),
      "latActiveFrames": sum(1 for s in episode if s.lat_active),
      "maxSpeedMps": round(max(s.v_ego for s in episode), 3),
      "maxAbsAngleDeg": round(max(abs(s.steering_angle_deg) for s in episode), 3),
      "maxAbsOutputTorqueUnits": max(abs(s.output_torque_units) for s in episode),
      "preFaultMaxAbsCommandTorque": round(max((abs(s.commanded_torque) for s in pre), default=0.0), 3),
      "preFaultMaxAbsOutputTorqueUnits": max((abs(s.output_torque_units) for s in pre), default=0),
      "preFaultNearLimitFrames": sum(1 for s in pre if s.eps_guard_near_limit),
      "preFaultStrongDriverOverrideFrames": len(strong_driver_override),
      "preFaultMaxAbsDriverTorque": round(max((abs(s.steering_torque) for s in pre), default=0.0), 3),
      "preFaultLastStrongDriverOverrideAgoS": (
        round(first.t - strong_driver_override[-1].t, 3) if strong_driver_override else None
      ),
      "preFaultTorqueReversals": reversals,
      "firstFault": event_dict(first),
      "lastPreFault": event_dict(pre[-1]) if pre else None,
    })
  return reports


def summarize(samples: list[LateralSample], commits: list[str], expected_commit: str) -> dict[str, Any]:
  temp = [s for s in samples if s.steer_fault_temporary]
  temp_lat = [s for s in temp if s.lat_active]
  high_speed_temp_lat = [s for s in temp_lat if s.v_ego >= HIGH_SPEED_FAULT_MPS]
  low_speed_alerts = [s for s in samples if s.low_speed_alert]
  driver_override_faults = [s for s in temp_lat if s.steering_pressed]
  non_driver_override_faults = [s for s in temp_lat if not s.steering_pressed]
  manual_candidates = [s for s in samples if s.manual_turn_guard_candidate]
  eps_near = [s for s in samples if s.eps_guard_near_limit]
  fault_episodes = summarize_fault_episodes(samples)
  driver_override_associated_episodes = [
    episode for episode in fault_episodes
    if episode["preFaultStrongDriverOverrideFrames"] > 0
  ]

  eps_risk_bursts: list[dict[str, Any]] = []
  active_count: dict[tuple[str, int], int] = {}
  for sample in samples:
    key = sample_key(sample)
    if sample.eps_guard_near_limit:
      active_count[key] = active_count.get(key, 0) + 1
      if active_count[key] == EPS_RISK_TRIGGER_FRAMES:
        eps_risk_bursts.append(event_dict(sample))
    else:
      active_count[key] = max(active_count.get(key, 0) - 2, 0)

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
        "strongDriverOverrideFramesBeforeFault": sum(
          1 for s in window
          if s.steering_pressed and abs(s.steering_torque) >= STRONG_DRIVER_OVERRIDE_TORQUE
        ),
        "maxAbsDriverTorqueBeforeFault": round(max(abs(s.steering_torque) for s in window), 3),
      })

  matching_commits = [commit for commit in commits if commit == expected_commit]
  status = "pass"
  if temp_lat:
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
    "driverOverrideTempFaultFrames": len(driver_override_faults),
    "nonDriverOverrideTempFaultFrames": len(non_driver_override_faults),
    "manualTurnMonitorFrames": len(manual_candidates),
    "epsNearLimitFrames": len(eps_near),
    "faultEpisodes": fault_episodes,
    "driverOverrideAssociatedFaultEpisodes": driver_override_associated_episodes,
    "epsRiskBursts": eps_risk_bursts[:12],
    "highSpeedFaultExamples": [event_dict(s) for s in high_speed_temp_lat[:12]],
    "nonDriverOverrideExamples": [event_dict(s) for s in non_driver_override_faults[:12]],
    "driverOverrideExamples": [event_dict(s) for s in driver_override_faults[:12]],
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
    "driverOverrideTempFaultFrames",
    "nonDriverOverrideTempFaultFrames",
    "manualTurnMonitorFrames",
    "epsNearLimitFrames",
  )
  summary = {key: report.get(key) for key in keys if key in report}
  summary["epsRiskBurstCount"] = len(report.get("epsRiskBursts", []))
  summary["driverOverrideAssociatedFaultEpisodeCount"] = len(report.get("driverOverrideAssociatedFaultEpisodes", []))
  summary["highSpeedFaultExampleCount"] = len(report.get("highSpeedFaultExamples", []))
  summary["nonDriverOverrideExampleCount"] = len(report.get("nonDriverOverrideExamples", []))
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
