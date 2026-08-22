#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode

from scan_longitudinal_quality import expand_logs


PRE_WINDOW_S = 6.0
POST_WINDOW_S = 2.0
MIN_BRAKE_EVENT_GAP_S = 4.0


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


def safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
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


@dataclass
class Sample:
  route: str
  segment: int
  t: float
  enabled: bool
  lat_active: bool
  long_active: bool
  v_ego: float
  a_ego: float
  brake_pressed: bool
  gas_pressed: bool
  steer_fault_temporary: bool
  low_speed_alert: bool
  steering_pressed: bool
  steering_torque: float
  steering_torque_eps: float
  steering_angle_deg: float
  steer_warning: str
  desired_curvature: float
  actual_curvature: float
  command_torque: float
  output_torque: float
  command_accel: float
  output_accel: float
  plan_accel: float
  should_stop: bool
  model_should_stop: bool
  red_light: bool
  forcing_stop: bool
  forcing_stop_length: float
  tracking_lead: bool
  lead_status: bool
  lead_d_rel: float
  lead_v_rel: float
  lead_v_lead: float
  lead_y_rel: float
  lead_model_prob: float
  lead_radar: bool
  lead_radar_track_id: int
  source: str


def make_sample(path: Path, start_ns: int, mono_time: int, latest: dict[str, Any]) -> Sample | None:
  if "carState" not in latest:
    return None

  car_state = latest["carState"]
  car_control = latest.get("carControl")
  car_output = latest.get("carOutput")
  controls_state = latest.get("controlsState")
  long_plan = latest.get("longitudinalPlan")
  starpilot_plan = latest.get("starpilotPlan")
  radar_state = latest.get("radarState")
  model = latest.get("modelV2")
  lateral_plan = latest.get("lateralPlan")

  actuators = safe_attr(car_control, "actuators")
  out_actuators = safe_attr(car_output, "actuatorsOutput") if car_output is not None else None
  model_action = safe_attr(model, "action")
  lead = safe_attr(radar_state, "leadOne") if radar_state is not None else None
  lead_status = bool(safe_attr(lead, "status", False))
  lateral_debug = safe_attr(lateral_plan, "debugState") if lateral_plan is not None else None

  return Sample(
    route=route_name(path),
    segment=segment_number(path),
    t=(mono_time - start_ns) / 1e9,
    enabled=bool(safe_attr(controls_state, "enabled", False)),
    lat_active=bool(safe_attr(car_control, "latActive", False)),
    long_active=bool(safe_attr(car_control, "longActive", False)),
    v_ego=safe_float(safe_attr(car_state, "vEgo", 0.0)),
    a_ego=safe_float(safe_attr(car_state, "aEgo", 0.0)),
    brake_pressed=bool(safe_attr(car_state, "brakePressed", False)),
    gas_pressed=bool(safe_attr(car_state, "gasPressed", False)),
    steer_fault_temporary=bool(safe_attr(car_state, "steerFaultTemporary", False)),
    low_speed_alert=bool(safe_attr(car_state, "lowSpeedAlert", False)),
    steering_pressed=bool(safe_attr(car_state, "steeringPressed", False)),
    steering_torque=safe_float(safe_attr(car_state, "steeringTorque", 0.0)),
    steering_torque_eps=safe_float(safe_attr(car_state, "steeringTorqueEps", 0.0)),
    steering_angle_deg=safe_float(safe_attr(car_state, "steeringAngleDeg", 0.0)),
    steer_warning=str(safe_attr(car_state, "steerWarning", "")),
    desired_curvature=safe_float(safe_attr(lateral_plan, "desiredCurvature", 0.0)),
    actual_curvature=safe_float(safe_attr(lateral_debug, "actualCurvature", 0.0)),
    command_torque=safe_float(safe_attr(actuators, "torque", 0.0)),
    output_torque=safe_float(safe_attr(out_actuators, "torque", safe_attr(actuators, "torque", 0.0))),
    command_accel=safe_float(safe_attr(actuators, "accel", 0.0)),
    output_accel=safe_float(safe_attr(out_actuators, "accel", safe_attr(actuators, "accel", 0.0))),
    plan_accel=safe_float(safe_attr(long_plan, "aTarget", 0.0)),
    should_stop=bool(safe_attr(long_plan, "shouldStop", False)),
    model_should_stop=bool(safe_attr(model_action, "shouldStop", False)),
    red_light=bool(safe_attr(starpilot_plan, "redLight", False)),
    forcing_stop=bool(safe_attr(starpilot_plan, "forcingStop", False)),
    forcing_stop_length=safe_float(safe_attr(starpilot_plan, "forcingStopLength", 0.0)),
    tracking_lead=bool(safe_attr(starpilot_plan, "trackingLead", False)),
    lead_status=lead_status,
    lead_d_rel=safe_float(safe_attr(lead, "dRel", 0.0)) if lead_status else 0.0,
    lead_v_rel=safe_float(safe_attr(lead, "vRel", 0.0)) if lead_status else 0.0,
    lead_v_lead=safe_float(safe_attr(lead, "vLead", 0.0)) if lead_status else 0.0,
    lead_y_rel=safe_float(safe_attr(lead, "yRel", 0.0)) if lead_status else 0.0,
    lead_model_prob=safe_float(safe_attr(lead, "modelProb", 0.0)) if lead_status else 0.0,
    lead_radar=bool(safe_attr(lead, "radar", False)) if lead_status else False,
    lead_radar_track_id=int(safe_attr(lead, "radarTrackId", -1)) if lead_status else -1,
    source=str(safe_attr(long_plan, "longitudinalPlanSource", "unknown")),
  )


def summarize_window(samples: list[Sample]) -> dict[str, Any]:
  if not samples:
    return {}
  return {
    "count": len(samples),
    "startT": round(samples[0].t, 2),
    "endT": round(samples[-1].t, 2),
    "minVEgo": round(min(s.v_ego for s in samples), 3),
    "maxVEgo": round(max(s.v_ego for s in samples), 3),
    "maxAbsCommandTorque": round(max(abs(s.command_torque) for s in samples), 3),
    "maxAbsOutputTorque": round(max(abs(s.output_torque) for s in samples), 3),
    "maxAbsDriverTorque": round(max(abs(s.steering_torque) for s in samples), 3),
    "maxAbsSteeringAngleDeg": round(max(abs(s.steering_angle_deg) for s in samples), 3),
    "steeringPressedFrames": sum(s.steering_pressed for s in samples),
    "latActiveFrames": sum(s.lat_active for s in samples),
    "enabledFrames": sum(s.enabled for s in samples),
    "longActiveFrames": sum(s.long_active for s in samples),
    "brakePressedFrames": sum(s.brake_pressed for s in samples),
    "redLightFrames": sum(s.red_light for s in samples),
    "forcingStopFrames": sum(s.forcing_stop for s in samples),
    "modelShouldStopFrames": sum(s.model_should_stop for s in samples),
    "shouldStopFrames": sum(s.should_stop for s in samples),
    "minPlanAccel": round(min(s.plan_accel for s in samples), 3),
    "minCommandAccel": round(min(s.command_accel for s in samples), 3),
    "minOutputAccel": round(min(s.output_accel for s in samples), 3),
    "minLeadDRel": None if not any(s.lead_status for s in samples) else round(min(s.lead_d_rel for s in samples if s.lead_status), 3),
    "sources": sorted(set(s.source for s in samples)),
  }


def event_sample(sample: Sample) -> dict[str, Any]:
  data = asdict(sample)
  for key, value in list(data.items()):
    if isinstance(value, float):
      data[key] = round(value, 3)
  return data


def build_event(kind: str, sample: Sample, prior: list[Sample], after: list[Sample]) -> dict[str, Any]:
  return {
    "kind": kind,
    "sample": event_sample(sample),
    "prior": summarize_window(prior),
    "after": summarize_window(after),
    "priorTail": [event_sample(s) for s in prior[-8:]],
    "afterHead": [event_sample(s) for s in after[:8]],
  }


def read_route(path: Path) -> list[Sample]:
  latest: dict[str, Any] = {}
  samples: list[Sample] = []
  start_ns: int | None = None
  for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
    which = msg.which()
    mono_time = int(msg.logMonoTime)
    if start_ns is None and which in ("carState", "carControl", "controlsState"):
      start_ns = mono_time
    if which in (
      "carState", "carControl", "carOutput", "controlsState", "longitudinalPlan",
      "starpilotPlan", "radarState", "modelV2", "lateralPlan",
    ):
      latest[which] = getattr(msg, which)
    if start_ns is None or which not in ("carState", "carControl", "controlsState", "longitudinalPlan", "lateralPlan"):
      continue
    sample = make_sample(path, start_ns, mono_time, latest)
    if sample is not None:
      samples.append(sample)
  return samples


def analyze_segment(samples: list[Sample]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
  steering_events: list[dict[str, Any]] = []
  brake_events: list[dict[str, Any]] = []
  prior: deque[Sample] = deque()
  in_fault = False
  brake_was_pressed = False
  last_brake_event_t = -1e9

  for i, sample in enumerate(samples):
    while prior and sample.t - prior[0].t > PRE_WINDOW_S:
      prior.popleft()

    if sample.steer_fault_temporary and not in_fault:
      after = [s for s in samples[i:i + 80] if s.t - sample.t <= POST_WINDOW_S]
      steering_events.append(build_event("steerFaultTemporary", sample, list(prior), after))
      in_fault = True
    elif not sample.steer_fault_temporary:
      in_fault = False

    brake_rising = sample.brake_pressed and not brake_was_pressed
    if brake_rising and sample.t - last_brake_event_t >= MIN_BRAKE_EVENT_GAP_S:
      prior_window = list(prior)
      if sample.enabled or sample.long_active or any(s.long_active for s in prior_window[-20:]):
        after = [s for s in samples[i:i + 80] if s.t - sample.t <= POST_WINDOW_S]
        brake_events.append(build_event("manualBrake", sample, prior_window, after))
        last_brake_event_t = sample.t
    brake_was_pressed = sample.brake_pressed

    prior.append(sample)

  return steering_events, brake_events


def analyze(samples: list[Sample]) -> dict[str, Any]:
  steering_events: list[dict[str, Any]] = []
  brake_events: list[dict[str, Any]] = []
  segment_samples: dict[tuple[str, int], list[Sample]] = {}
  for sample in samples:
    segment_samples.setdefault((sample.route, sample.segment), []).append(sample)

  for segment in segment_samples.values():
    segment_steering, segment_brakes = analyze_segment(sorted(segment, key=lambda s: s.t))
    steering_events.extend(segment_steering)
    brake_events.extend(segment_brakes)

  return {
    "samples": len(samples),
    "routes": sorted(set(s.route for s in samples)),
    "segments": sorted(set(f"{s.route}--{s.segment}" for s in samples)),
    "steeringFaultEvents": steering_events,
    "manualBrakeEvents": brake_events,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description="Find Carnival steering-fault and manual-brake intervention context.")
  parser.add_argument("logs", nargs="+", help="qlog paths/globs")
  parser.add_argument("--out", help="Optional JSON output path")
  args = parser.parse_args()

  all_samples: list[Sample] = []
  for path in expand_logs(args.logs):
    all_samples.extend(read_route(path))
  result = analyze(sorted(all_samples, key=lambda s: (s.route, s.segment, s.t)))
  text = json.dumps(result, indent=2, sort_keys=True)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
