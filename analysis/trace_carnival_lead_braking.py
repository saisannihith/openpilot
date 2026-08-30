#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode


CARNIVAL_CONFIRMATION_TRACK_ID_MIN = 0xC4100
CARNIVAL_CONFIRMATION_TRACK_ID_MAX = 0xC41FF
CARNIVAL_PRIMARY_TRACK_ID_MIN = 0xC4200
CARNIVAL_PRIMARY_TRACK_ID_MAX = 0xC42FF


def is_carnival_r0100_track(track_id: int) -> bool:
  return (
    CARNIVAL_CONFIRMATION_TRACK_ID_MIN <= track_id <= CARNIVAL_CONFIRMATION_TRACK_ID_MAX or
    CARNIVAL_PRIMARY_TRACK_ID_MIN <= track_id <= CARNIVAL_PRIMARY_TRACK_ID_MAX
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


def segment_number(path: Path) -> int:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 3:
      try:
        return int(parts[2].split(".", 1)[0])
      except ValueError:
        pass
  return -1


def route_name(path: Path) -> str:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 2:
      return "--".join(parts[:2])
  return path.parent.name


@dataclass
class Sample:
  route: str
  segment: int
  t: float
  v_ego: float
  a_ego: float
  brake_pressed: bool
  gas_pressed: bool
  long_active: bool
  cmd_accel: float
  plan_accel: float
  source: str
  red_light: bool
  forcing_stop: bool
  should_stop: bool
  model_desired_accel: float
  model_should_stop: bool
  lead_status: bool
  lead_radar: bool
  lead_track_id: int
  lead_d_rel: float
  lead_y_rel: float
  lead_v_rel: float
  lead_v_lead: float
  lead_prob: float
  model_lead_prob: float
  model_lead_d_rel: float
  model_lead_v_rel: float
  confirmation_tracks: int
  closest_path_track_d_rel: float
  closest_path_track_v_rel: float
  closest_path_track_id: int

  @property
  def ttc(self) -> float:
    if not self.lead_status or self.lead_v_rel >= -0.1:
      return 99.0
    return self.lead_d_rel / -self.lead_v_rel


def make_sample(path: Path, start_ns: int, mono_time: int, latest: dict[str, Any]) -> Sample | None:
  required = ("carState", "carControl", "longitudinalPlan", "starpilotPlan", "radarState", "modelV2")
  if not all(key in latest for key in required):
    return None

  car_state = latest["carState"]
  car_control = latest["carControl"]
  long_plan = latest["longitudinalPlan"]
  starpilot_plan = latest["starpilotPlan"]
  radar_state = latest["radarState"]
  model = latest["modelV2"]
  lead = radar_state.leadOne
  model_leads = list(safe_attr(model, "leadsV3", []))
  model_lead = model_leads[0] if model_leads else None
  model_v_ego = safe_float((list(safe_attr(safe_attr(model, "velocity"), "x", [])) or [0.0])[0])

  lead_status = bool(safe_attr(lead, "status", False))
  lead_track_id = int(safe_attr(lead, "radarTrackId", -1)) if lead_status else -1
  live_tracks = list(safe_attr(latest.get("liveTracks"), "points", []))
  confirmation_tracks = [
    track for track in live_tracks
    if is_carnival_r0100_track(int(safe_attr(track, "trackId", -1)))
  ]
  path_tracks = [track for track in confirmation_tracks if abs(safe_float(safe_attr(track, "yRel", 99.0))) <= 1.25]
  closest_path_track = min(path_tracks, key=lambda track: safe_float(safe_attr(track, "dRel", 999.0)), default=None)
  actuators = safe_attr(car_control, "actuators")
  model_action = safe_attr(model, "action")

  model_d_rel = 0.0
  model_v_rel = 0.0
  model_prob = 0.0
  if model_lead is not None:
    model_x = list(safe_attr(model_lead, "x", []))
    model_v = list(safe_attr(model_lead, "v", []))
    model_prob = safe_float(safe_attr(model_lead, "prob", 0.0))
    model_d_rel = safe_float(model_x[0] - 1.52) if model_x else 0.0
    model_v_rel = safe_float(model_v[0] - model_v_ego) if model_v else 0.0

  return Sample(
    route=route_name(path),
    segment=segment_number(path),
    t=(mono_time - start_ns) / 1e9,
    v_ego=safe_float(safe_attr(car_state, "vEgo", 0.0)),
    a_ego=safe_float(safe_attr(car_state, "aEgo", 0.0)),
    brake_pressed=bool(safe_attr(car_state, "brakePressed", False)),
    gas_pressed=bool(safe_attr(car_state, "gasPressed", False)),
    long_active=bool(safe_attr(car_control, "longActive", False)),
    cmd_accel=safe_float(safe_attr(actuators, "accel", 0.0)),
    plan_accel=safe_float(safe_attr(long_plan, "aTarget", 0.0)),
    source=str(safe_attr(long_plan, "longitudinalPlanSource", "unknown")),
    red_light=bool(safe_attr(starpilot_plan, "redLight", False)),
    forcing_stop=bool(safe_attr(starpilot_plan, "forcingStop", False)),
    should_stop=bool(safe_attr(long_plan, "shouldStop", False)),
    model_desired_accel=safe_float(safe_attr(model_action, "desiredAcceleration", 0.0)),
    model_should_stop=bool(safe_attr(model_action, "shouldStop", False)),
    lead_status=lead_status,
    lead_radar=bool(safe_attr(lead, "radar", False)) if lead_status else False,
    lead_track_id=lead_track_id,
    lead_d_rel=safe_float(safe_attr(lead, "dRel", 0.0)) if lead_status else 0.0,
    lead_y_rel=safe_float(safe_attr(lead, "yRel", 0.0)) if lead_status else 0.0,
    lead_v_rel=safe_float(safe_attr(lead, "vRel", 0.0)) if lead_status else 0.0,
    lead_v_lead=safe_float(safe_attr(lead, "vLead", 0.0)) if lead_status else 0.0,
    lead_prob=safe_float(safe_attr(lead, "modelProb", 0.0)) if lead_status else 0.0,
    model_lead_prob=model_prob,
    model_lead_d_rel=model_d_rel,
    model_lead_v_rel=model_v_rel,
    confirmation_tracks=len(confirmation_tracks),
    closest_path_track_d_rel=(safe_float(safe_attr(closest_path_track, "dRel", 0.0)) if closest_path_track is not None else 0.0),
    closest_path_track_v_rel=(safe_float(safe_attr(closest_path_track, "vRel", 0.0)) if closest_path_track is not None else 0.0),
    closest_path_track_id=(int(safe_attr(closest_path_track, "trackId", -1)) if closest_path_track is not None else -1),
  )


def read_samples(path: Path) -> list[Sample]:
  latest: dict[str, Any] = {}
  samples: list[Sample] = []
  start_ns: int | None = None
  for msg in LogReader(str(path), default_mode=ReadMode.RLOG, sort_by_time=True):
    which = msg.which()
    mono_time = int(msg.logMonoTime)
    if start_ns is None and which == "carState":
      start_ns = mono_time
    if which in ("carState", "carControl", "longitudinalPlan", "starpilotPlan", "radarState", "modelV2", "liveTracks"):
      latest[which] = getattr(msg, which)
    if which == "longitudinalPlan" and start_ns is not None:
      sample = make_sample(path, start_ns, mono_time, latest)
      if sample is not None:
        samples.append(sample)
  return samples


def sample_dict(sample: Sample) -> dict[str, Any]:
  result = asdict(sample)
  result["ttc"] = sample.ttc
  for key, value in result.items():
    if isinstance(value, float):
      result[key] = round(value, 3)
  return result


def event_windows(samples: list[Sample]) -> list[dict[str, Any]]:
  events: list[tuple[str, int]] = []
  previous: Sample | None = None
  risk_active = False
  headway_risk_active = False
  for i, sample in enumerate(samples):
    if previous is None or (sample.route, sample.segment) != (previous.route, previous.segment):
      risk_active = False
      headway_risk_active = False
    if sample.brake_pressed and (previous is None or not previous.brake_pressed):
      events.append(("manualBrake", i))
    risk = sample.long_active and sample.v_ego >= 5.0 and sample.lead_status and sample.ttc <= 4.0
    if risk and not risk_active:
      events.append(("closingRisk", i))
    headway_risk = bool(
      sample.long_active and sample.v_ego >= 8.0 and sample.lead_status and
      sample.lead_d_rel / max(sample.v_ego, 0.1) <= 0.9
    )
    if headway_risk and not headway_risk_active:
      events.append(("headwayRisk", i))
    risk_active = risk
    headway_risk_active = headway_risk
    previous = sample

  result: list[dict[str, Any]] = []
  for event_type, index in events:
    center = samples[index]
    window = [
      sample for sample in samples
      if (sample.route, sample.segment) == (center.route, center.segment) and center.t - 6.0 <= sample.t <= center.t + 2.0
    ]
    timeline: list[dict[str, Any]] = []
    last_bucket = -1
    for sample in window:
      bucket = int((sample.t - (center.t - 6.0)) / 0.25)
      if bucket != last_bucket or sample.brake_pressed:
        timeline.append(sample_dict(sample))
        last_bucket = bucket
    result.append({
      "type": event_type,
      "route": center.route,
      "segment": center.segment,
      "t": round(center.t, 3),
      "snapshot": sample_dict(center),
      "timeline": timeline,
    })
  return result


def lead_episodes(samples: list[Sample]) -> list[dict[str, Any]]:
  groups: list[list[Sample]] = []
  current: list[Sample] = []
  for sample in samples:
    continuous = bool(
      current and
      sample.route == current[-1].route and
      sample.segment == current[-1].segment and
      sample.t - current[-1].t <= 0.20
    )
    if sample.long_active and sample.lead_status:
      if not continuous and current:
        groups.append(current)
        current = []
      current.append(sample)
    elif current:
      groups.append(current)
      current = []
  if current:
    groups.append(current)

  result: list[dict[str, Any]] = []
  for group in groups:
    if group[-1].t - group[0].t < 0.5:
      continue
    closest = min(group, key=lambda sample: sample.lead_d_rel)
    lowest_ttc = min(group, key=lambda sample: sample.ttc)
    first_radar = next((sample for sample in group if sample.lead_radar), None)
    result.append({
      "route": group[0].route,
      "segment": group[0].segment,
      "startT": round(group[0].t, 3),
      "endT": round(group[-1].t, 3),
      "duration": round(group[-1].t - group[0].t, 3),
      "first": sample_dict(group[0]),
      "firstRadar": None if first_radar is None else sample_dict(first_radar),
      "closest": sample_dict(closest),
      "lowestTtc": sample_dict(lowest_ttc),
      "minHeadway": round(min(sample.lead_d_rel / max(sample.v_ego, 0.1) for sample in group), 3),
      "maxClosingSpeed": round(max(-sample.lead_v_rel for sample in group), 3),
      "minCmdAccel": round(min(sample.cmd_accel for sample in group), 3),
      "radarFraction": round(sum(sample.lead_radar for sample in group) / len(group), 3),
    })
  return sorted(result, key=lambda episode: episode["minHeadway"])


def expand_logs(patterns: list[str]) -> list[Path]:
  paths: list[Path] = []
  for pattern in patterns:
    root = Path("/") if pattern.startswith("/") else Path()
    paths.extend(path for path in root.glob(pattern.lstrip("/")) if path.is_file())
  return sorted(set(paths))


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  paths = expand_logs(args.logs)
  samples: list[Sample] = []
  for path in paths:
    samples.extend(read_samples(path))
  payload = {
    "files": len(paths),
    "samples": len(samples),
    "events": event_windows(samples),
    "leadEpisodes": lead_episodes(samples),
  }
  output = json.dumps(payload, indent=2, sort_keys=True)
  print(output)
  if args.out is not None:
    args.out.write_text(output + "\n")


if __name__ == "__main__":
  main()
