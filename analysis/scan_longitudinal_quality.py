#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  CARNIVAL_PRE_RED_STOP_EVIDENCE_HOLD_TIME,
  CARNIVAL_PRE_RED_STOP_EVIDENCE_MAX_LENGTH,
  CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_BRAKE,
  CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_DROP,
  CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_LENGTH,
  CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_RATIO,
  CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_SPEED,
  get_carnival_red_light_stop_line_decel,
  LONE_HIGH_SPEED_RED_LIGHT_MAX_BRAKE,
  update_carnival_lone_high_speed_red_light_suppression,
)


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


@dataclass
class Sample:
  route: str
  segment: int
  t: float
  long_active: bool
  enabled: bool
  v_ego: float
  a_ego: float
  gas_pressed: bool
  brake_pressed: bool
  standstill: bool
  cruise_standstill: bool
  long_control_state: int
  red_light: bool
  forcing_stop: bool
  forcing_stop_length: float
  tracking_lead: bool
  should_stop: bool
  model_should_stop: bool
  lead_status: bool
  lead_d_rel: float
  lead_v_rel: float
  lead_v_lead: float
  lead_a_lead: float
  lead_y_rel: float
  plan_accel: float
  cmd_accel: float
  out_accel: float
  source: str


def make_sample(path: Path, start_ns: int, mono_time: int, latest: dict[str, Any]) -> Sample | None:
  required = ("carState", "carControl", "controlsState", "longitudinalPlan", "starpilotPlan", "radarState")
  if not all(k in latest for k in required):
    return None

  car_state = latest["carState"]
  car_control = latest["carControl"]
  controls_state = latest["controlsState"]
  long_plan = latest["longitudinalPlan"]
  starpilot_plan = latest["starpilotPlan"]
  radar_state = latest["radarState"]
  car_output = latest.get("carOutput")
  model = latest.get("modelV2")
  model_action = safe_attr(model, "action")
  lead = radar_state.leadOne
  actuators = safe_attr(car_control, "actuators")
  out_actuators = safe_attr(car_output, "actuatorsOutput") if car_output is not None else None
  lead_status = bool(safe_attr(lead, "status", False))

  return Sample(
    route=route_name(path),
    segment=segment_number(path),
    t=(mono_time - start_ns) / 1e9,
    long_active=bool(safe_attr(car_control, "longActive", False)),
    enabled=bool(safe_attr(controls_state, "enabled", False)),
    v_ego=safe_float(safe_attr(car_state, "vEgo", 0.0)),
    a_ego=safe_float(safe_attr(car_state, "aEgo", 0.0)),
    gas_pressed=bool(safe_attr(car_state, "gasPressed", False)),
    brake_pressed=bool(safe_attr(car_state, "brakePressed", False)),
    standstill=bool(safe_attr(car_state, "standstill", False)),
    cruise_standstill=bool(safe_attr(safe_attr(car_state, "cruiseState"), "standstill", False)),
    long_control_state=int(safe_attr(safe_attr(controls_state, "longControlState"), "raw", -1)),
    red_light=bool(safe_attr(starpilot_plan, "redLight", False)),
    forcing_stop=bool(safe_attr(starpilot_plan, "forcingStop", False)),
    forcing_stop_length=safe_float(safe_attr(starpilot_plan, "forcingStopLength", 0.0)),
    tracking_lead=bool(safe_attr(starpilot_plan, "trackingLead", False)),
    should_stop=bool(safe_attr(long_plan, "shouldStop", False)),
    model_should_stop=bool(safe_attr(model_action, "shouldStop", False)),
    lead_status=lead_status,
    lead_d_rel=safe_float(safe_attr(lead, "dRel", 0.0)) if lead_status else 0.0,
    lead_v_rel=safe_float(safe_attr(lead, "vRel", 0.0)) if lead_status else 0.0,
    lead_v_lead=safe_float(safe_attr(lead, "vLead", 0.0)) if lead_status else 0.0,
    lead_a_lead=safe_float(safe_attr(lead, "aLeadK", 0.0)) if lead_status else 0.0,
    lead_y_rel=safe_float(safe_attr(lead, "yRel", 0.0)) if lead_status else 0.0,
    plan_accel=safe_float(safe_attr(long_plan, "aTarget", 0.0)),
    cmd_accel=safe_float(safe_attr(actuators, "accel", 0.0)),
    out_accel=safe_float(safe_attr(out_actuators, "accel", safe_attr(actuators, "accel", 0.0))),
    source=str(safe_attr(long_plan, "longitudinalPlanSource", "unknown")),
  )


def snapshot_software(init_data: Any) -> dict[str, Any]:
  return {
    "version": str(safe_attr(init_data, "version", "unknown")),
    "gitCommit": str(safe_attr(init_data, "gitCommit", "unknown")),
    "gitSrcCommit": str(safe_attr(init_data, "gitSrcCommit", "")),
    "gitBranch": str(safe_attr(init_data, "gitBranch", "unknown")),
    "gitRemote": str(safe_attr(init_data, "gitRemote", "unknown")),
    "dirty": bool(safe_attr(init_data, "dirty", False)),
  }


def read_samples_and_metadata(path: Path, mode: ReadMode) -> tuple[list[Sample], dict[str, Any] | None]:
  latest: dict[str, Any] = {}
  samples: list[Sample] = []
  start_ns: int | None = None
  software: dict[str, Any] | None = None
  for msg in LogReader(str(path), default_mode=mode, sort_by_time=True):
    which = msg.which()
    mono_time = int(msg.logMonoTime)
    if start_ns is None and which in ("carState", "longitudinalPlan", "controlsState"):
      start_ns = mono_time
    if which == "initData" and software is None:
      software = snapshot_software(msg.initData)
    if which in ("carState", "carControl", "carOutput", "controlsState", "longitudinalPlan", "starpilotPlan", "radarState", "modelV2"):
      latest[which] = getattr(msg, which)
    if which == "longitudinalPlan" and start_ns is not None:
      sample = make_sample(path, start_ns, mono_time, latest)
      if sample is not None:
        samples.append(sample)
  return samples, software


def read_samples(path: Path, mode: ReadMode) -> list[Sample]:
  samples, _software = read_samples_and_metadata(path, mode)
  return samples


def event_dict(sample: Sample) -> dict[str, Any]:
  data = asdict(sample)
  for key, value in list(data.items()):
    if isinstance(value, float):
      data[key] = round(value, 3)
  return data


def group_by_gap(samples: list[Sample], predicate, max_gap: float = 1.0) -> list[list[Sample]]:
  groups: list[list[Sample]] = []
  current: list[Sample] = []
  for sample in samples:
    if predicate(sample):
      current.append(sample)
    elif current:
      if sample.t - current[-1].t <= max_gap:
        current.append(sample)
      else:
        groups.append(current)
        current = []
  if current:
    groups.append(current)
  return [group for group in groups if len(group) >= 3]


def summarize_stop(group: list[Sample]) -> dict[str, Any]:
  use = [s for s in group if s.long_active] or group
  first = use[0]
  final = use[-1]
  standstill = next((s for s in use if s.standstill or abs(s.v_ego) < 0.05), None)
  return {
    "route": first.route,
    "segment": first.segment,
    "startT": round(first.t, 2),
    "endT": round(final.t, 2),
    "duration": round(final.t - first.t, 2),
    "maxSpeedMps": round(max(s.v_ego for s in use), 3),
    "minSpeedMps": round(min(s.v_ego for s in use), 3),
    "minCmdAccel": round(min(s.cmd_accel for s in use), 3),
    "minPlanAccel": round(min(s.plan_accel for s in use), 3),
    "maxForceStopLength": round(max(s.forcing_stop_length for s in use), 3),
    "finalForceStopLength": round(final.forcing_stop_length, 3),
    "standstillT": None if standstill is None else round(standstill.t, 2),
    "standstillForceStopLength": None if standstill is None else round(standstill.forcing_stop_length, 3),
    "manualBrake": any(s.brake_pressed for s in use),
    "manualGas": any(s.gas_pressed for s in use),
    "leadSeen": any(s.lead_status for s in use),
    "minLeadDRel": None if not any(s.lead_status for s in use) else round(min(s.lead_d_rel for s in use if s.lead_status), 3),
    "modelShouldStopFrames": sum(s.model_should_stop for s in use),
    "shouldStopFrames": sum(s.should_stop for s in use),
    "redLightFrames": sum(s.red_light for s in use),
    "forcingStopFrames": sum(s.forcing_stop for s in use),
    "sources": sorted(set(s.source for s in use)),
  }


def summarize_lead_departures(samples: list[Sample]) -> list[dict[str, Any]]:
  opportunities: list[dict[str, Any]] = []
  i = 0
  while i < len(samples):
    sample = samples[i]
    ready = (
      sample.long_active and
      sample.v_ego < 0.35 and
      sample.lead_status and
      2.0 <= sample.lead_d_rel <= 20.0 and
      sample.lead_v_lead >= 0.55 and
      sample.lead_v_rel >= 0.25 and
      not sample.red_light and
      not sample.forcing_stop and
      not sample.should_stop and
      not sample.brake_pressed
    )
    if not ready:
      i += 1
      continue

    start = sample
    window: list[Sample] = []
    j = i
    while j < len(samples) and samples[j].t - start.t <= 6.0:
      window.append(samples[j])
      j += 1

    ego_move = next((s for s in window if s.v_ego >= 0.75), None)
    accel_floor = max((s.plan_accel for s in window[:25]), default=0.0)
    move_time = None if ego_move is None else ego_move.t
    manual = next((
      s for s in window
      if (s.brake_pressed or s.gas_pressed) and (move_time is None or s.t < move_time)
    ), None)
    manual_any = next((s for s in window if s.brake_pressed or s.gas_pressed), None)
    opportunities.append({
      "route": start.route,
      "segment": start.segment,
      "startT": round(start.t, 2),
      "startStandstill": start.standstill,
      "startCruiseStandstill": start.cruise_standstill,
      "startLongControlState": start.long_control_state,
      "startLeadDRel": round(start.lead_d_rel, 3),
      "startLeadVLead": round(start.lead_v_lead, 3),
      "startLeadVRel": round(start.lead_v_rel, 3),
      "startPlanAccel": round(start.plan_accel, 3),
      "startCmdAccel": round(start.cmd_accel, 3),
      "maxEarlyPlanAccel": round(accel_floor, 3),
      "egoMoveDelay": None if ego_move is None else round(ego_move.t - start.t, 2),
      "egoMoveLongControlState": None if ego_move is None else ego_move.long_control_state,
      "manualOverrideBeforeMoveDelay": None if manual is None else round(manual.t - start.t, 2),
      "manualOverrideAnyDelay": None if manual_any is None else round(manual_any.t - start.t, 2),
    })
    i = max(j, i + 1)
  return opportunities


def summarize_stop_releases(samples: list[Sample]) -> list[dict[str, Any]]:
  opportunities: list[dict[str, Any]] = []
  seen_stop_context = False
  previous_route: str | None = None
  previous_segment: int | None = None
  i = 0
  ordered = sorted(samples, key=lambda s: (s.route, s.segment, s.t))
  while i < len(ordered):
    sample = ordered[i]
    if sample.route != previous_route or sample.segment != previous_segment:
      seen_stop_context = False
      previous_route = sample.route
      previous_segment = sample.segment

    stop_context = sample.red_light or sample.forcing_stop or sample.should_stop or sample.model_should_stop
    if stop_context:
      seen_stop_context = True
      i += 1
      continue

    ready = (
      seen_stop_context and
      sample.long_active and
      sample.v_ego < 0.35 and
      not sample.brake_pressed and
      not sample.gas_pressed
    )
    if not ready:
      i += 1
      continue

    start = sample
    window: list[Sample] = []
    j = i
    while j < len(ordered):
      candidate = ordered[j]
      if candidate.route != start.route or candidate.segment != start.segment or candidate.t - start.t > 6.0:
        break
      window.append(candidate)
      j += 1

    ego_move = next((s for s in window if s.v_ego >= 0.75), None)
    move_time = None if ego_move is None else ego_move.t
    manual = next((
      s for s in window
      if (s.brake_pressed or s.gas_pressed) and (move_time is None or s.t < move_time)
    ), None)
    manual_any = next((s for s in window if s.brake_pressed or s.gas_pressed), None)
    opportunities.append({
      "route": start.route,
      "segment": start.segment,
      "startT": round(start.t, 2),
      "startStandstill": start.standstill,
      "startCruiseStandstill": start.cruise_standstill,
      "startLongControlState": start.long_control_state,
      "startLeadStatus": start.lead_status,
      "startLeadDRel": None if not start.lead_status else round(start.lead_d_rel, 3),
      "startPlanAccel": round(start.plan_accel, 3),
      "startCmdAccel": round(start.cmd_accel, 3),
      "maxEarlyPlanAccel": round(max((s.plan_accel for s in window[:25]), default=0.0), 3),
      "egoMoveDelay": None if ego_move is None else round(ego_move.t - start.t, 2),
      "egoMoveLongControlState": None if ego_move is None else ego_move.long_control_state,
      "manualOverrideBeforeMoveDelay": None if manual is None else round(manual.t - start.t, 2),
      "manualOverrideAnyDelay": None if manual_any is None else round(manual_any.t - start.t, 2),
    })
    seen_stop_context = False
    i = max(j, i + 1)
  return opportunities


def summarize_accel_jumps(samples: list[Sample]) -> list[dict[str, Any]]:
  jumps: list[dict[str, Any]] = []
  previous: Sample | None = None
  for sample in samples:
    if previous is not None and sample.route == previous.route and sample.segment == previous.segment and sample.long_active and previous.long_active:
      dt = sample.t - previous.t
      if 0.015 <= dt <= 0.35:
        step = sample.plan_accel - previous.plan_accel
        jerk = step / dt
        if abs(step) >= 0.75 or abs(jerk) >= 4.0:
          jumps.append({
            "route": sample.route,
            "segment": sample.segment,
            "t": round(sample.t, 2),
            "dt": round(dt, 3),
            "planAccelBefore": round(previous.plan_accel, 3),
            "planAccelAfter": round(sample.plan_accel, 3),
            "step": round(step, 3),
            "jerk": round(jerk, 3),
            "vEgo": round(sample.v_ego, 3),
            "lead": sample.lead_status,
            "redLight": sample.red_light,
            "forcingStop": sample.forcing_stop,
            "shouldStop": sample.should_stop,
            "source": sample.source,
          })
    previous = sample
  jumps.sort(key=lambda event: abs(event["jerk"]), reverse=True)
  return jumps[:80]


def summarize_current_red_light_gate(samples: list[Sample]) -> dict[str, Any]:
  class CP:
    carFingerprint = "KIA_CARNIVAL_4TH_GEN"

  cap = -LONE_HIGH_SPEED_RED_LIGHT_MAX_BRAKE
  suppressed = False
  previous_route: str | None = None
  previous_segment: int | None = None
  summary: dict[str, Any] = {
    "capAccel": round(cap, 3),
    "redNoLeadNoModelStopFrames": 0,
    "currentSuppressedFrames": 0,
    "loggedBelowCurrentCapFrames": 0,
    "allowedStrongBrakeFrames": 0,
    "allowedStrongBrakeLongActiveFrames": 0,
    "allowedStrongBrakeLongActiveEnabledFrames": 0,
    "stopLineEvidenceFrames": 0,
    "suppressedExamples": [],
    "allowedStrongBrakeExamples": [],
    "stopLineEvidenceExamples": [],
  }

  for sample in sorted(samples, key=lambda s: (s.route, s.segment, s.t)):
    if previous_route != sample.route or previous_segment != sample.segment:
      suppressed = False
      previous_route = sample.route
      previous_segment = sample.segment

    lead_control_active = bool(sample.tracking_lead or sample.lead_status)
    stop_line_decel = get_carnival_red_light_stop_line_decel(
      CP,
      sample.v_ego,
      sample.red_light,
      sample.model_should_stop,
      lead_control_active,
      sample.forcing_stop,
      sample.forcing_stop_length,
    )
    suppressed = update_carnival_lone_high_speed_red_light_suppression(
      CP,
      sample.v_ego,
      sample.red_light,
      sample.model_should_stop,
      lead_control_active,
      sample.forcing_stop,
      suppressed,
      sample.forcing_stop_length,
    )

    risk_context = (
      sample.red_light and
      not lead_control_active and
      not sample.model_should_stop and
      sample.v_ego >= 8.0
    )
    if not risk_context:
      continue

    summary["redNoLeadNoModelStopFrames"] += 1
    if stop_line_decel is not None:
      summary["stopLineEvidenceFrames"] += 1
      if len(summary["stopLineEvidenceExamples"]) < 12:
        summary["stopLineEvidenceExamples"].append({
          "route": sample.route,
          "segment": sample.segment,
          "t": round(sample.t, 2),
          "vEgo": round(sample.v_ego, 2),
          "forcingStop": sample.forcing_stop,
          "forcingStopLength": round(sample.forcing_stop_length, 1),
          "requiredDecel": round(stop_line_decel, 3),
          "loggedPlanAccel": round(sample.plan_accel, 3),
          "loggedCmdAccel": round(sample.cmd_accel, 3),
          "longActive": sample.long_active,
          "enabled": sample.enabled,
        })
    if suppressed:
      summary["currentSuppressedFrames"] += 1
      if sample.plan_accel < cap:
        summary["loggedBelowCurrentCapFrames"] += 1
        if len(summary["suppressedExamples"]) < 12:
          summary["suppressedExamples"].append({
            "route": sample.route,
            "segment": sample.segment,
            "t": round(sample.t, 2),
            "vEgo": round(sample.v_ego, 2),
            "forcingStop": sample.forcing_stop,
            "forcingStopLength": round(sample.forcing_stop_length, 1),
            "loggedPlanAccel": round(sample.plan_accel, 3),
            "loggedCmdAccel": round(sample.cmd_accel, 3),
            "currentCapAccel": round(cap, 3),
            "longActive": sample.long_active,
            "enabled": sample.enabled,
          })
    elif sample.plan_accel <= -1.2:
      summary["allowedStrongBrakeFrames"] += 1
      if sample.long_active and sample.cmd_accel <= -1.2:
        summary["allowedStrongBrakeLongActiveFrames"] += 1
        if sample.enabled:
          summary["allowedStrongBrakeLongActiveEnabledFrames"] += 1
      if len(summary["allowedStrongBrakeExamples"]) < 12:
        summary["allowedStrongBrakeExamples"].append({
          "route": sample.route,
          "segment": sample.segment,
          "t": round(sample.t, 2),
          "vEgo": round(sample.v_ego, 2),
          "forcingStop": sample.forcing_stop,
          "forcingStopLength": round(sample.forcing_stop_length, 1),
          "loggedPlanAccel": round(sample.plan_accel, 3),
          "loggedCmdAccel": round(sample.cmd_accel, 3),
          "longActive": sample.long_active,
          "enabled": sample.enabled,
        })

  return summary


def pre_red_stop_context_keys(samples: list[Sample]) -> set[tuple[str, int, float]]:
  keys: set[tuple[str, int, float]] = set()
  previous_route: str | None = None
  previous_segment: int | None = None
  start_length: float | None = None
  start_t = 0.0
  evidence_until = 0.0

  for sample in sorted(samples, key=lambda s: (s.route, s.segment, s.t)):
    if sample.route != previous_route or sample.segment != previous_segment:
      previous_route = sample.route
      previous_segment = sample.segment
      start_length = None
      start_t = sample.t
      evidence_until = 0.0

    valid_length = (
      math.isfinite(sample.forcing_stop_length) and
      CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_LENGTH <= sample.forcing_stop_length <= CARNIVAL_PRE_RED_STOP_EVIDENCE_MAX_LENGTH
    )
    if sample.red_light:
      if sample.t < evidence_until:
        keys.add((sample.route, sample.segment, sample.t))
      continue
    if sample.forcing_stop or not valid_length or sample.v_ego < CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_SPEED:
      start_length = None
      start_t = sample.t
      continue

    if start_length is None or sample.forcing_stop_length > start_length + 10.0:
      start_length = sample.forcing_stop_length
      start_t = sample.t

    elapsed = max(0.0, sample.t - start_t)
    model_drop = max(0.0, start_length - sample.forcing_stop_length)
    distance_travelled = max(1.0, sample.v_ego * max(elapsed, 0.05))
    drop_ratio = model_drop / distance_travelled
    braking_evidence = sample.plan_accel <= -CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_BRAKE
    if (
      braking_evidence and
      model_drop >= CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_DROP and
      drop_ratio >= CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_RATIO
    ):
      evidence_until = sample.t + CARNIVAL_PRE_RED_STOP_EVIDENCE_HOLD_TIME

    if sample.t < evidence_until:
      keys.add((sample.route, sample.segment, sample.t))

  return keys


def summarize_software_metadata(metadata: list[dict[str, Any]]) -> list[dict[str, Any]]:
  seen: dict[tuple[Any, ...], dict[str, Any]] = {}
  for item in metadata:
    key = (
      item.get("gitCommit"),
      item.get("gitSrcCommit"),
      item.get("gitBranch"),
      item.get("gitRemote"),
      item.get("version"),
      item.get("dirty"),
    )
    if key not in seen:
      seen[key] = {
        "gitCommit": item.get("gitCommit"),
        "gitSrcCommit": item.get("gitSrcCommit"),
        "gitBranch": item.get("gitBranch"),
        "gitRemote": item.get("gitRemote"),
        "version": item.get("version"),
        "dirty": item.get("dirty"),
        "files": 0,
      }
    seen[key]["files"] += 1
  return sorted(seen.values(), key=lambda item: (-int(item["files"]), str(item.get("gitCommit"))))


def analyze(samples: list[Sample], software_metadata: list[dict[str, Any]] | None = None) -> dict[str, Any]:
  stop_groups = group_by_gap(samples, lambda s: s.red_light or s.forcing_stop or s.should_stop or s.model_should_stop, max_gap=1.5)
  pre_red_context = pre_red_stop_context_keys(samples)
  no_context_brakes = [
    event_dict(s) for s in samples
    if s.long_active and s.v_ego >= 12.0 and s.cmd_accel <= -1.8 and
    not s.lead_status and not s.red_light and not s.forcing_stop and not s.should_stop and not s.model_should_stop and
    (s.route, s.segment, s.t) not in pre_red_context
  ][:80]
  stop_context_brakes = [
    event_dict(s) for s in samples
    if s.long_active and s.v_ego >= 12.0 and s.cmd_accel <= -1.8 and
    not s.lead_status and (
      s.red_light or s.forcing_stop or s.should_stop or s.model_should_stop or
      (s.route, s.segment, s.t) in pre_red_context
    )
  ][:80]
  return {
    "samples": len(samples),
    "routes": sorted(set(s.route for s in samples)),
    "segments": len(set((s.route, s.segment) for s in samples)),
    "software": summarize_software_metadata(software_metadata or []),
    "leadDepartureOpportunities": summarize_lead_departures(samples),
    "stopReleaseOpportunities": summarize_stop_releases(samples),
    "noContextHighwayHardBrakes": no_context_brakes,
    "stopContextHighwayHardBrakes": stop_context_brakes,
    "stopEpisodes": [summarize_stop(group) for group in stop_groups],
    "accelJumps": summarize_accel_jumps(samples),
    "currentRedLightGateAudit": summarize_current_red_light_gate(samples),
  }


def expand_logs(patterns: list[str]) -> list[Path]:
  paths: list[Path] = []
  for pattern in patterns:
    root = Path("/") if pattern.startswith("/") else Path()
    paths.extend(path for path in root.glob(pattern.lstrip("/")) if path.is_file())
  return sorted(set(paths))


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--mode", choices=("qlog", "rlog"), default="qlog")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  mode = ReadMode.QLOG if args.mode == "qlog" else ReadMode.RLOG
  samples: list[Sample] = []
  software_metadata: list[dict[str, Any]] = []
  for path in expand_logs(args.logs):
    path_samples, software = read_samples_and_metadata(path, mode)
    samples.extend(path_samples)
    if software is not None:
      software_metadata.append(software)

  payload = analyze(samples, software_metadata)
  text = json.dumps(payload, indent=2, sort_keys=True)
  print(text)
  if args.out is not None:
    args.out.write_text(text + "\n")


if __name__ == "__main__":
  main()
