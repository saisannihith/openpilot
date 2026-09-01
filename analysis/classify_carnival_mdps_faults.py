#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Iterable
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from opendbc.car.lateral import apply_driver_steer_torque_limits
from openpilot.tools.lib.logreader import LogReader, ReadMode


MDPS_ADDR = 0xEA
LFA_ADDR = 0x12A
STEER_MAX = 409
PRE_FAULT_S = 2.0
POST_FAULT_S = 0.5
DRIVER_CONFLICT_TORQUE = 100.0
STRONG_DRIVER_CONFLICT_TORQUE = 300.0
HIGH_ANGLE_DEG = 90.0
CANDIDATE_CONFLICT_TORQUE = 300.0
CANDIDATE_CONFLICT_MIN_COMMAND = 40
CANDIDATE_CONFLICT_HOLD_FRAMES = 20
CANDIDATE_CONFLICT_UNWIND_STEP = 10


@dataclass(frozen=True)
class CarSample:
  t: float
  v_ego: float
  lat_active: bool
  steer_fault: bool
  steering_pressed: bool
  driver_torque: float
  eps_torque: float
  steering_angle: float
  steering_rate: float
  commanded_torque: float
  output_torque_units: int
  current_limiter_output_units: int
  conflict_hold_output_units: int
  conflict_hold_active: bool


@dataclass(frozen=True)
class MdpsSample:
  t: float
  src: int
  counter: int
  warning_lamp: int
  plugin: int
  toi_active: int
  toi_unavailable: int
  toi_fault: int
  fail: int


@dataclass(frozen=True)
class CommandSample:
  t: float
  src: int
  request: int
  torque_units: int
  toi_fault: int


def safe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def finite_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except Exception:
    return default
  return result if math.isfinite(result) else default


def get_bits(data: bytes, start: int, size: int) -> int:
  raw = int.from_bytes(data, byteorder="little", signed=False)
  return (raw >> start) & ((1 << size) - 1)


def route_segment(path: Path) -> str:
  return path.parent.name


def sibling_log(path: Path, stem: str) -> Path | None:
  for suffix in (".zst", ".bz2", ""):
    candidate = path.parent / f"{stem}{suffix}"
    if candidate.exists():
      return candidate
  return None


def discover_segments(roots: Iterable[Path]) -> list[tuple[Path, Path]]:
  segments: dict[Path, dict[str, Path]] = {}
  for root in roots:
    paths = [root] if root.is_file() else root.rglob("*")
    for path in paths:
      if not path.is_file():
        continue
      if path.name.startswith("qlog"):
        segments.setdefault(path.parent, {})["qlog"] = path
      elif path.name.startswith("rlog"):
        segments.setdefault(path.parent, {})["rlog"] = path

  discovered: list[tuple[Path, Path]] = []
  for _directory, logs in sorted(segments.items(), key=lambda item: str(item[0])):
    rlog = logs.get("rlog")
    if rlog is None:
      continue
    discovered.append((logs.get("qlog", rlog), rlog))
  return discovered


def has_fault(path: Path) -> bool:
  for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
    if msg.which() == "carState" and bool(msg.carState.steerFaultTemporary):
      return True
  return False


def decode_mdps(t: float, frame: Any) -> MdpsSample:
  data = bytes(frame.dat)
  return MdpsSample(
    t=t,
    src=int(frame.src),
    counter=get_bits(data, 16, 8),
    warning_lamp=get_bits(data, 24, 3),
    plugin=get_bits(data, 46, 2),
    toi_active=get_bits(data, 48, 2),
    toi_unavailable=get_bits(data, 50, 2),
    toi_fault=get_bits(data, 52, 2),
    fail=get_bits(data, 54, 2),
  )


def decode_command(t: float, frame: Any) -> CommandSample:
  data = bytes(frame.dat)
  return CommandSample(
    t=t,
    src=int(frame.src),
    request=get_bits(data, 52, 2),
    torque_units=get_bits(data, 41, 11) - 1024,
    toi_fault=get_bits(data, 54, 2),
  )


def read_segment(path: Path) -> tuple[list[CarSample], list[MdpsSample], list[CommandSample], dict[str, str]]:
  latest: dict[str, Any] = {}
  route_info: dict[str, str] = {}
  cars: list[CarSample] = []
  mdps: list[MdpsSample] = []
  commands: list[CommandSample] = []
  start_ns: int | None = None
  current_limiter_output = 0
  conflict_hold_output = 0
  conflict_hold_frames = 0

  for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
    which = msg.which()
    mono_ns = int(msg.logMonoTime)
    if start_ns is None:
      start_ns = mono_ns
    t = (mono_ns - start_ns) / 1e9

    if which == "initData":
      route_info = {
        "gitCommit": str(safe_attr(msg.initData, "gitCommit", "")),
        "gitBranch": str(safe_attr(msg.initData, "gitBranch", "")),
        "version": str(safe_attr(msg.initData, "version", "")),
      }
      continue

    if which in ("carControl", "carOutput"):
      latest[which] = getattr(msg, which)
      continue

    if which == "can":
      for frame in msg.can:
        if int(frame.address) == MDPS_ADDR and len(frame.dat) >= 8:
          mdps.append(decode_mdps(t, frame))
      continue

    if which == "sendcan":
      for frame in msg.sendcan:
        if int(frame.address) == LFA_ADDR and len(frame.dat) >= 8:
          commands.append(decode_command(t, frame))
      continue

    if which != "carState":
      continue

    cs = msg.carState
    cc = latest.get("carControl")
    co = latest.get("carOutput")
    requested = safe_attr(cc, "actuators")
    output = safe_attr(co, "actuatorsOutput")
    output_units = int(safe_attr(output, "torqueOutputCan", 0) or 0)
    if output_units == 0:
      output_units = int(round(finite_float(safe_attr(output, "torque", 0.0)) * STEER_MAX))
    v_ego_raw = finite_float(safe_attr(cs, "vEgoRaw", safe_attr(cs, "vEgo", 0.0)))
    requested_torque = finite_float(safe_attr(requested, "torque", 0.0))
    low_speed = v_ego_raw < 15.0
    current_limits = SimpleNamespace(
      STEER_MAX=STEER_MAX,
      STEER_DRIVER_ALLOWANCE=100,
      STEER_DRIVER_MULTIPLIER=2,
      STEER_DRIVER_FACTOR=1,
      STEER_DELTA_UP=10 if low_speed else 2,
      STEER_DELTA_DOWN=8 if low_speed else 3,
      STEER_DRIVER_DELTA_DOWN=10,
    )
    lat_active = bool(safe_attr(cc, "latActive", False))
    current_limiter_output = apply_driver_steer_torque_limits(
      int(round(requested_torque * STEER_MAX)), current_limiter_output,
      finite_float(safe_attr(cs, "steeringTorque", 0.0)), current_limits,
    )
    if not lat_active:
      current_limiter_output = 0

    conflict_hold_output_last = conflict_hold_output
    conflict_hold_output = apply_driver_steer_torque_limits(
      int(round(requested_torque * STEER_MAX)), conflict_hold_output_last,
      finite_float(safe_attr(cs, "steeringTorque", 0.0)), current_limits,
    )
    strong_driver_override = lat_active and abs(finite_float(safe_attr(cs, "steeringTorque", 0.0))) >= CANDIDATE_CONFLICT_TORQUE
    opposing_driver_override = strong_driver_override and (
      conflict_hold_frames > 0 or
      (abs(conflict_hold_output_last) >= CANDIDATE_CONFLICT_MIN_COMMAND and
       conflict_hold_output_last * finite_float(safe_attr(cs, "steeringTorque", 0.0)) < 0)
    )
    if opposing_driver_override:
      conflict_hold_frames = CANDIDATE_CONFLICT_HOLD_FRAMES
    elif conflict_hold_frames > 0:
      conflict_hold_frames -= 1
    if conflict_hold_frames > 0:
      conflict_hold_output = int(max(conflict_hold_output_last - CANDIDATE_CONFLICT_UNWIND_STEP,
                                     min(0, conflict_hold_output_last + CANDIDATE_CONFLICT_UNWIND_STEP)))
    if not lat_active:
      conflict_hold_output = 0
      conflict_hold_frames = 0
    cars.append(CarSample(
      t=t,
      v_ego=finite_float(safe_attr(cs, "vEgo", 0.0)),
      lat_active=lat_active,
      steer_fault=bool(safe_attr(cs, "steerFaultTemporary", False)),
      steering_pressed=bool(safe_attr(cs, "steeringPressed", False)),
      driver_torque=finite_float(safe_attr(cs, "steeringTorque", 0.0)),
      eps_torque=finite_float(safe_attr(cs, "steeringTorqueEps", 0.0)),
      steering_angle=finite_float(safe_attr(cs, "steeringAngleDeg", 0.0)),
      steering_rate=finite_float(safe_attr(cs, "steeringRateDeg", 0.0)),
      commanded_torque=requested_torque,
      output_torque_units=output_units,
      current_limiter_output_units=current_limiter_output,
      conflict_hold_output_units=conflict_hold_output,
      conflict_hold_active=conflict_hold_frames > 0,
    ))
  return cars, mdps, commands, route_info


def window(samples: Iterable[Any], start: float, end: float) -> list[Any]:
  return [sample for sample in samples if start <= sample.t <= end]


def first_nonzero(samples: list[MdpsSample], field: str) -> float | None:
  for sample in samples:
    if int(getattr(sample, field)) != 0:
      return sample.t
  return None


def first_rising(samples: list[MdpsSample], field: str) -> tuple[float | None, bool]:
  if not samples:
    return None, False
  initially_active = int(getattr(samples[0], field)) != 0
  previous = initially_active
  for sample in samples[1:]:
    active = int(getattr(sample, field)) != 0
    if active and not previous:
      return sample.t, initially_active
    previous = active
  return None, initially_active


def max_run_duration(samples: list[CarSample], predicate) -> float:
  longest = 0.0
  start: float | None = None
  previous = 0.0
  for sample in samples:
    if predicate(sample):
      if start is None:
        start = sample.t
      previous = sample.t
    elif start is not None:
      longest = max(longest, previous - start)
      start = None
  if start is not None:
    longest = max(longest, previous - start)
  return longest


def classify_event(event_t: float, cars: list[CarSample], mdps: list[MdpsSample], commands: list[CommandSample]) -> dict[str, Any]:
  pre_cars = window(cars, event_t - PRE_FAULT_S, event_t)
  causal_cars = window(cars, event_t - 0.5, event_t)
  mdps_window = window(mdps, event_t - PRE_FAULT_S, event_t + POST_FAULT_S)
  pre_commands = window(commands, event_t - PRE_FAULT_S, event_t)
  causal_commands = window(commands, event_t - 0.5, event_t)

  conflict = [sample for sample in causal_cars if sample.output_torque_units * sample.driver_torque < 0 and
              abs(sample.driver_torque) >= DRIVER_CONFLICT_TORQUE]
  strong_conflict = [sample for sample in conflict if abs(sample.driver_torque) >= STRONG_DRIVER_CONFLICT_TORQUE]
  high_angle_duration = max_run_duration(pre_cars, lambda sample: abs(sample.steering_angle) >= HIGH_ANGLE_DEG)
  request_drops = [sample for sample in causal_commands if sample.request == 0]
  outgoing_faults = [sample for sample in causal_commands if sample.toi_fault != 0]
  command_steps = [abs(second.torque_units - first.torque_units)
                   for first, second in zip(pre_commands, pre_commands[1:], strict=False)]

  unavailable_t, unavailable_persisted = first_rising(mdps_window, "toi_unavailable")
  toi_fault_t, toi_fault_persisted = first_rising(mdps_window, "toi_fault")
  fail_t, fail_persisted = first_rising(mdps_window, "fail")
  active_drop = next((sample.t for sample in mdps_window if sample.toi_active == 0 and sample.t >= event_t - 0.25), None)

  request_guard = high_angle_duration >= 0.75 and bool(outgoing_faults)
  sustained_conflict = max_run_duration(causal_cars, lambda sample: sample.output_torque_units * sample.driver_torque < 0 and
                                        abs(sample.driver_torque) >= DRIVER_CONFLICT_TORQUE) >= 0.08
  last_opposing_ages = {}
  for threshold in (250, 300, 350, 400):
    opposing = [sample for sample in causal_cars if sample.output_torque_units * sample.driver_torque < 0 and
                abs(sample.driver_torque) >= threshold]
    last_opposing_ages[f"lastOpposing{threshold}AgeS"] = round(event_t - opposing[-1].t, 3) if opposing else None

  if (toi_fault_persisted or fail_persisted) and toi_fault_t is None and fail_t is None:
    classification = "continued_mdps_fault_reengagement"
  elif request_guard:
    classification = "high_angle_request_guard"
  elif sustained_conflict and unavailable_t is not None:
    classification = "driver_conflict_unavailable"
  elif sustained_conflict and toi_fault_t is not None:
    classification = "driver_conflict_toi_fault"
  elif sustained_conflict:
    classification = "driver_conflict_aggregate_fault"
  elif unavailable_t is not None:
    classification = "mdps_unavailable_without_driver_conflict"
  elif toi_fault_t is not None:
    classification = "mdps_toi_fault_without_driver_conflict"
  elif fail_t is not None:
    classification = "mdps_fail_unspecified"
  else:
    classification = "carstate_fault_without_raw_mdps_transition"

  return {
    "t": round(event_t, 3),
    "classification": classification,
    "preFaultFrames": len(pre_cars),
    "preFaultLatActiveFrames": sum(sample.lat_active for sample in pre_cars),
    "causalFrames": len(causal_cars),
    "causalLatActiveFrames": sum(sample.lat_active for sample in causal_cars),
    "maxSpeedMps": round(max((sample.v_ego for sample in pre_cars), default=0.0), 3),
    "maxAbsAngleDeg": round(max((abs(sample.steering_angle) for sample in pre_cars), default=0.0), 3),
    "maxAbsSteeringRateDegS": round(max((abs(sample.steering_rate) for sample in pre_cars), default=0.0), 3),
    "causalMaxAbsSteeringRateDegS": round(max((abs(sample.steering_rate) for sample in causal_cars), default=0.0), 3),
    "highAngleDurationS": round(high_angle_duration, 3),
    "driverConflictFrames": len(conflict),
    "strongDriverConflictFrames": len(strong_conflict),
    "maxAbsDriverTorque": round(max((abs(sample.driver_torque) for sample in pre_cars), default=0.0), 3),
    "causalMaxAbsDriverTorque": round(max((abs(sample.driver_torque) for sample in causal_cars), default=0.0), 3),
    "causalMaxAbsEpsTorque": round(max((abs(sample.eps_torque) for sample in causal_cars), default=0.0), 3),
    "maxAbsOutputTorqueUnits": max((abs(sample.output_torque_units) for sample in pre_cars), default=0),
    "causalMaxAbsOutputTorqueUnits": max((abs(sample.output_torque_units) for sample in causal_cars), default=0),
    "causalMaxAbsCurrentLimiterTorqueUnits": max((abs(sample.current_limiter_output_units) for sample in causal_cars), default=0),
    "causalMaxAbsConflictHoldTorqueUnits": max((abs(sample.conflict_hold_output_units) for sample in causal_cars), default=0),
    "causalConflictHoldFrames": sum(sample.conflict_hold_active for sample in causal_cars),
    "faultEdgeOutputTorqueUnits": causal_cars[-1].output_torque_units if causal_cars else 0,
    "faultEdgeCurrentLimiterTorqueUnits": causal_cars[-1].current_limiter_output_units if causal_cars else 0,
    "faultEdgeConflictHoldTorqueUnits": causal_cars[-1].conflict_hold_output_units if causal_cars else 0,
    "faultEdgeDriverTorque": round(causal_cars[-1].driver_torque, 3) if causal_cars else 0.0,
    "faultEdgeEpsTorque": round(causal_cars[-1].eps_torque, 3) if causal_cars else 0.0,
    "faultEdgeSteeringAngleDeg": round(causal_cars[-1].steering_angle, 3) if causal_cars else 0.0,
    "faultEdgeSteeringRateDegS": round(causal_cars[-1].steering_rate, 3) if causal_cars else 0.0,
    "maxCommandStepUnits": max(command_steps, default=0),
    "requestDropFrames": len(request_drops),
    "outgoingToiFaultFrames": len(outgoing_faults),
    **last_opposing_ages,
    "mdpsUnavailableAt": round(unavailable_t, 3) if unavailable_t is not None else None,
    "mdpsToiFaultAt": round(toi_fault_t, 3) if toi_fault_t is not None else None,
    "mdpsFailAt": round(fail_t, 3) if fail_t is not None else None,
    "mdpsActiveDropAt": round(active_drop, 3) if active_drop is not None else None,
    "mdpsUnavailablePersisted": unavailable_persisted,
    "mdpsToiFaultPersisted": toi_fault_persisted,
    "mdpsFailPersisted": fail_persisted,
    "mdpsStates": [asdict(sample) for sample in mdps_window if sample.toi_unavailable or sample.toi_fault or sample.fail],
  }


def analyze_segment(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
  cars, mdps, commands, route_info = read_segment(path)
  events: list[dict[str, Any]] = []
  previous_fault = False
  for sample in cars:
    if sample.steer_fault and not previous_fault:
      event = classify_event(sample.t, cars, mdps, commands)
      event["segment"] = route_segment(path)
      event["rlog"] = str(path)
      event["rawMdpsFrames"] = len(mdps)
      event["steeringCommandFrames"] = len(commands)
      event.update(route_info)
      events.append(event)
    previous_fault = sample.steer_fault
  hold_episodes = sum(sample.conflict_hold_active and (index == 0 or not cars[index - 1].conflict_hold_active)
                      for index, sample in enumerate(cars))
  stats = {
    **route_info,
    "carStateFrames": len(cars),
    "latActiveFrames": sum(sample.lat_active for sample in cars),
    "strongDriverFrames": sum(abs(sample.driver_torque) >= CANDIDATE_CONFLICT_TORQUE and sample.lat_active for sample in cars),
    "conflictHoldFrames": sum(sample.conflict_hold_active for sample in cars),
    "conflictHoldEpisodes": hold_episodes,
  }
  return events, stats


def main() -> None:
  parser = argparse.ArgumentParser(description="Classify 2024 Carnival MDPS temporary steering fault episodes.")
  parser.add_argument("roots", nargs="+", type=Path)
  parser.add_argument("--out", type=Path)
  parser.add_argument("--all-rlogs", action="store_true", help="Skip the qlog fault prefilter.")
  args = parser.parse_args()

  segments = discover_segments(args.roots)
  candidates: list[Path] = []
  for index, (prefilter, rlog) in enumerate(segments, 1):
    if args.all_rlogs or has_fault(prefilter):
      candidates.append(rlog)
    if index % 50 == 0:
      print(f"prefiltered {index}/{len(segments)} segments", file=sys.stderr)

  events: list[dict[str, Any]] = []
  segment_stats: list[dict[str, Any]] = []
  errors: list[dict[str, str]] = []
  for index, rlog in enumerate(candidates, 1):
    try:
      segment_events, stats = analyze_segment(rlog)
      events.extend(segment_events)
      segment_stats.append(stats)
    except Exception as exc:
      errors.append({"rlog": str(rlog), "error": f"{type(exc).__name__}: {exc}"})
    print(f"analyzed {index}/{len(candidates)} candidate segments", file=sys.stderr)

  counts = Counter(event["classification"] for event in events)
  counterfactual_stats = {
    key: sum(int(stats.get(key, 0)) for stats in segment_stats)
    for key in ("carStateFrames", "latActiveFrames", "strongDriverFrames", "conflictHoldFrames", "conflictHoldEpisodes")
  }
  counterfactual_stats_by_commit: dict[str, dict[str, int]] = {}
  for stats in segment_stats:
    commit = str(stats.get("gitCommit", "unknown")) or "unknown"
    commit_stats = counterfactual_stats_by_commit.setdefault(commit, dict.fromkeys(counterfactual_stats, 0))
    for key in commit_stats:
      commit_stats[key] += int(stats.get(key, 0))
  report = {
    "segmentsDiscovered": len(segments),
    "candidateFaultSegments": len(candidates),
    "faultEvents": len(events),
    "classificationCounts": dict(sorted(counts.items())),
    "counterfactualStats": counterfactual_stats,
    "counterfactualStatsByCommit": counterfactual_stats_by_commit,
    "events": events,
    "errors": errors,
  }
  payload = json.dumps(report, indent=2, sort_keys=True)
  if args.out:
    args.out.write_text(payload + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("segmentsDiscovered", "candidateFaultSegments", "faultEvents",
                                                   "classificationCounts")}, indent=2, sort_keys=True))
  else:
    print(payload)


if __name__ == "__main__":
  main()
