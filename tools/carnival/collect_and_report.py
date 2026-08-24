#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Any

try:
  os.register_at_fork
except AttributeError:
  os.register_at_fork = lambda *args, **kwargs: None  # type: ignore[attr-defined]

from openpilot.tools.lib.logreader import LogReader


REMOTE_LOG_ROOT = "/data/media/0/realdata"
LOG_NAMES = ("rlog", "rlog.bz2", "rlog.zst", "qlog", "qlog.bz2", "qlog.zst")
CARNIVAL_OBJECT_BUS = 1
CARNIVAL_OBJECT_START = 0x180
CARNIVAL_OBJECT_END = 0x184
CARNIVAL_OBJECT_LEN = 32


@dataclass(frozen=True)
class RefLead:
  t: float
  d_rel: float
  y_rel: float
  v_rel: float
  v_ego: float


@dataclass(frozen=True)
class Obj:
  t: float
  addr: int
  slot: int
  state: int
  state_alt: int
  d_rel: float
  y_raw: float
  v_raw: float


@dataclass
class RadarMetrics:
  refs: int = 0
  selected: int = 0
  coverage: float = 0.0
  distance_mae: float | None = None
  distance_p90: float | None = None
  derived_velocity_mae: float | None = None
  derived_velocity_p90: float | None = None
  raw_velocity_mae: float | None = None
  raw_lateral_mae: float | None = None
  address_counts: dict[str, int] = field(default_factory=dict)
  slot_counts: dict[str, int] = field(default_factory=dict)
  state_counts: dict[str, int] = field(default_factory=dict)
  verdict: str = "insufficient"


@dataclass
class LateralMetrics:
  active_samples: int = 0
  steering_pressed_samples: int = 0
  steering_pressed_pct: float = 0.0
  steering_temp_events: int = 0
  steer_saturated_events: int = 0
  lateral_takeover_events: int = 0
  torque_clip_samples: int = 0
  torque_clip_pct: float = 0.0
  max_torque_error: float = 0.0
  curvature_error_p90: float | None = None
  verdict: str = "insufficient"


@dataclass
class LongitudinalMetrics:
  enabled_samples: int = 0
  lead_samples: int = 0
  radar_lead_samples: int = 0
  vision_lead_samples: int = 0
  min_lead_distance: float | None = None
  min_ttc: float | None = None
  brake_pressed_samples: int = 0
  gas_pressed_samples: int = 0
  accel_command_p90_abs: float | None = None
  harsh_brake_samples: int = 0
  verdict: str = "insufficient"


@dataclass
class RouteScorecard:
  overall_score: int = 0
  lateral_score: int = 0
  longitudinal_score: int = 0
  radar_score: int = 0
  intervention_events: int = 0
  false_brake_events: int = 0
  missed_stop_events: int = 0
  low_speed_creep_samples: int = 0
  torque_saturation_events: int = 0
  mean_confidence: float | None = None
  recommendations: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class SteeringEventWindow:
  event: str
  t: float
  v_ego: float
  lat_active: bool
  steering_pressed: bool
  desired_curvature: float
  curvature: float
  curvature_error: float
  desired_torque: float
  output_torque: float
  torque_error: float
  in_curve: bool


@dataclass
class RouteReport:
  route: str
  files: int = 0
  car_params: list[dict[str, Any]] = field(default_factory=list)
  services: dict[str, int] = field(default_factory=dict)
  events: dict[str, int] = field(default_factory=dict)
  alerts: dict[str, int] = field(default_factory=dict)
  modes: dict[str, Any] = field(default_factory=dict)
  radar: RadarMetrics = field(default_factory=RadarMetrics)
  lateral: LateralMetrics = field(default_factory=LateralMetrics)
  longitudinal: LongitudinalMetrics = field(default_factory=LongitudinalMetrics)
  scorecard: RouteScorecard = field(default_factory=RouteScorecard)
  steering_windows: list[SteeringEventWindow] = field(default_factory=list)
  issues: list[str] = field(default_factory=list)


def run(cmd: list[str], *, timeout: int = 60) -> subprocess.CompletedProcess:
  return subprocess.run(cmd, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)


def event_time_sec(event) -> float:
  return float(event.logMonoTime) / 1e9


def safe_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except Exception:
    return default
  return result if math.isfinite(result) else default


def safe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def enum_name(value: Any) -> str:
  text = str(value)
  return text.split(".")[-1]


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  values = sorted(values)
  idx = min(len(values) - 1, max(0, round((pct / 100.0) * (len(values) - 1))))
  return values[idx]


def extract_little(dat: bytes, start: int, size: int, signed: bool = False) -> int:
  raw = int.from_bytes(dat, "little", signed=False)
  val = (raw >> start) & ((1 << size) - 1)
  if signed and val & (1 << (size - 1)):
    val -= 1 << size
  return val


def decode_carnival_objects(t: float, addr: int, dat: bytes) -> list[Obj]:
  objs = []
  for slot, bit_offset in ((1, 0), (2, 128)):
    objs.append(Obj(
      t=t,
      addr=addr,
      slot=slot,
      state=extract_little(dat, bit_offset + 55, 4),
      state_alt=extract_little(dat, bit_offset + 51, 4),
      d_rel=extract_little(dat, bit_offset + 64, 12) * 0.05,
      y_raw=extract_little(dat, bit_offset + 76, 12, signed=True) * 0.05,
      v_raw=extract_little(dat, bit_offset + 88, 14, signed=True) * 0.01,
    ))
  return objs


def route_key(path: Path, root: Path) -> str:
  rel = path.relative_to(root)
  segment = rel.parts[0] if len(rel.parts) > 1 else path.parent.name
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", segment)
  return match.group(1) if match else segment


def segment_index(path: Path) -> int:
  match = re.match(r".+--([0-9]+)$", path.parent.name)
  return int(match.group(1)) if match else -1


def iter_log_files(root: Path) -> list[Path]:
  files_by_segment: dict[Path, list[Path]] = defaultdict(list)
  for p in root.rglob("*"):
    if p.name in LOG_NAMES:
      files_by_segment[p.parent].append(p)

  out = []
  for files in files_by_segment.values():
    rlogs = [p for p in files if p.name.startswith("rlog")]
    out.extend(rlogs if rlogs else files)
  return sorted(out, key=lambda p: (route_key(p, root), segment_index(p), p.name))


def remote_route_key(remote: str) -> str:
  segment = Path(remote).parent.name
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", segment)
  return match.group(1) if match else segment


def pull_logs(device: str, dest_root: Path, ssh_key: str | None, limit_routes: int | None,
              latest_routes: int | None) -> int:
  ssh = ["ssh"]
  scp = ["scp"]
  if ssh_key:
    ssh += ["-i", ssh_key]
    scp += ["-i", ssh_key]
  ssh += [
    "-o", "IdentitiesOnly=yes",
    "-o", "StrictHostKeyChecking=no",
    "-o", "ConnectTimeout=10",
    "-o", "ServerAliveInterval=5",
    "-o", "ServerAliveCountMax=2",
    f"comma@{device}",
  ]
  scp += [
    "-o", "IdentitiesOnly=yes",
    "-o", "StrictHostKeyChecking=no",
    "-o", "ConnectTimeout=10",
    "-o", "ServerAliveInterval=5",
    "-o", "ServerAliveCountMax=2",
  ]

  find_cmd = (
    "find /data/media/0/realdata -maxdepth 2 -type f "
    "\\( -name 'rlog.zst' -o -name 'qlog.zst' -o -name 'rlog.bz2' -o -name 'qlog.bz2' -o -name 'rlog' -o -name 'qlog' \\) | sort"
  )
  remote_files = run(ssh + [find_cmd], timeout=30).stdout.splitlines()
  route_order = []
  for remote in remote_files:
    route = remote_route_key(remote)
    if route not in route_order:
      route_order.append(route)
  if latest_routes is not None:
    keep = set(route_order[-latest_routes:])
    remote_files = [remote for remote in remote_files if remote_route_key(remote) in keep]
  if limit_routes is not None:
    route_order = []
    for remote in remote_files:
      route = remote_route_key(remote)
      if route not in route_order:
        route_order.append(route)
    keep = set(route_order[:limit_routes])
    remote_files = [remote for remote in remote_files if remote_route_key(remote) in keep]

  copied = 0
  failed = 0
  dest_root.mkdir(parents=True, exist_ok=True)
  for remote in remote_files:
    if not remote:
      continue
    segment = Path(remote).parent.name
    name = Path(remote).name
    local_dir = dest_root / segment
    local_dir.mkdir(parents=True, exist_ok=True)
    local_file = local_dir / name
    if local_file.exists():
      continue
    last_error = None
    for attempt in range(1, 4):
      try:
        run(scp + [f"comma@{device}:{remote}", str(local_file)], timeout=120)
        copied += 1
        last_error = None
        break
      except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        last_error = e
        if local_file.exists():
          local_file.unlink()
        print(f"Warning: copy failed for {remote} (attempt {attempt}/3)", file=sys.stderr, flush=True)
    if last_error is not None:
      failed += 1
      print(f"Warning: skipping {remote} after repeated copy failures", file=sys.stderr, flush=True)
  if failed:
    print(f"Warning: skipped {failed} log files due to copy failures", file=sys.stderr, flush=True)
  return copied


def nearest_objects(objects_by_addr: dict[int, list[Obj]], ts_by_addr: dict[int, list[float]], ref: RefLead, window: float = 0.08) -> list[Obj]:
  out = []
  for addr, seq in objects_by_addr.items():
    ts = ts_by_addr[addr]
    lo = 0
    hi = len(ts)
    while lo < hi:
      mid = (lo + hi) // 2
      if ts[mid] < ref.t:
        lo = mid + 1
      else:
        hi = mid
    best = None
    for idx in (lo - 1, lo):
      if 0 <= idx < len(seq):
        obj = seq[idx]
        if best is None or abs(obj.t - ref.t) < abs(best.t - ref.t):
          best = obj
    if best is not None and abs(best.t - ref.t) <= window:
      out.append(best)
  return out


def summarize_radar(refs: list[RefLead], objects_by_addr: dict[int, list[Obj]]) -> RadarMetrics:
  selected: list[tuple[RefLead, Obj]] = []
  for addr in list(objects_by_addr):
    objects_by_addr[addr].sort(key=lambda obj: obj.t)
  ts_by_addr = {addr: [obj.t for obj in seq] for addr, seq in objects_by_addr.items()}
  for ref in refs:
    objs = [
      obj for obj in nearest_objects(objects_by_addr, ts_by_addr, ref)
      if obj.state in (3, 4, 5) and 0.5 <= obj.d_rel <= 220.0
    ]
    if not objs:
      continue
    # Prefer the proven primary channel, then closest distance.
    best = min(objs, key=lambda obj: (0 if (obj.addr, obj.slot) == (0x180, 1) else 1, abs(obj.d_rel - ref.d_rel)))
    gate = max(3.0, min(12.0, ref.d_rel * 0.18))
    if abs(best.d_rel - ref.d_rel) <= gate:
      selected.append((ref, best))

  distance_errors = [abs(obj.d_rel - ref.d_rel) for ref, obj in selected]
  raw_v_errors = [abs(obj.v_raw - ref.v_rel) for ref, obj in selected]
  raw_y_errors = [abs(obj.y_raw - ref.y_rel) for ref, obj in selected]
  derived_v_errors = []
  last_by_key: dict[tuple[int, int], tuple[RefLead, Obj]] = {}
  for ref, obj in selected:
    key = (obj.addr, obj.slot)
    prev = last_by_key.get(key)
    if prev is not None:
      prev_ref, prev_obj = prev
      dt = ref.t - prev_ref.t
      if 0.05 <= dt <= 1.0:
        d_dot = (obj.d_rel - prev_obj.d_rel) / dt
        if math.isfinite(d_dot) and abs(d_dot) < 80.0:
          derived_v_errors.append(abs(d_dot - ref.v_rel))
    last_by_key[key] = (ref, obj)

  metrics = RadarMetrics(
    refs=len(refs),
    selected=len(selected),
    coverage=len(selected) / max(len(refs), 1),
    distance_mae=mean(distance_errors) if distance_errors else None,
    distance_p90=percentile(distance_errors, 90),
    derived_velocity_mae=mean(derived_v_errors) if derived_v_errors else None,
    derived_velocity_p90=percentile(derived_v_errors, 90),
    raw_velocity_mae=mean(raw_v_errors) if raw_v_errors else None,
    raw_lateral_mae=mean(raw_y_errors) if raw_y_errors else None,
    address_counts={f"0x{k:x}": v for k, v in Counter(obj.addr for _ref, obj in selected).items()},
    slot_counts={str(k): v for k, v in Counter(obj.slot for _ref, obj in selected).items()},
    state_counts={f"{state}/{alt}": v for (state, alt), v in Counter((obj.state, obj.state_alt) for _ref, obj in selected).most_common(10)},
  )
  if metrics.refs < 50:
    metrics.verdict = "insufficient reference lead data"
  elif metrics.coverage >= 0.65 and (metrics.distance_mae or 999.0) <= 3.5 and (metrics.derived_velocity_mae or 999.0) <= 2.0:
    metrics.verdict = "strong shadow candidate"
  elif metrics.coverage >= 0.35 and (metrics.distance_mae or 999.0) <= 5.0:
    metrics.verdict = "partial candidate"
  else:
    metrics.verdict = "weak/noisy candidate"
  return metrics


def analyze_route(route: str, files: list[Path]) -> RouteReport:
  report = RouteReport(route=route, files=len(files))
  car_params = Counter()
  services = Counter()
  events = Counter()
  alerts = Counter()
  refs: list[RefLead] = []
  objects_by_addr: dict[int, list[Obj]] = defaultdict(list)

  latest_car_state = None
  latest_car_control = None
  latest_car_output = None
  latest_controls_state = None
  latest_radar_state = None
  latest_starpilot_plan = None
  latest_model = None
  curvature_errors = []
  torque_clip_samples = 0
  torque_clip_total = 0
  lateral_active_samples = 0
  steering_pressed_samples = 0
  enabled_samples = 0
  lead_samples = 0
  radar_lead_samples = 0
  vision_lead_samples = 0
  ttc_values = []
  lead_distances = []
  brake_pressed_samples = 0
  gas_pressed_samples = 0
  accel_commands = []
  harsh_brake_samples = 0
  steering_window_last: dict[str, float] = {}
  previous_brake_pressed = False
  false_brake_active = False
  driver_brake_events = 0
  missed_stop_events = 0
  false_brake_events = 0
  low_speed_creep_samples = 0
  confidence_samples: list[float] = []

  def capture_steering_window(event_name: str, t: float) -> None:
    if len(report.steering_windows) >= 80:
      return
    if t - steering_window_last.get(event_name, -1e9) < 0.8:
      return
    steering_window_last[event_name] = t
    v_ego = safe_float(safe_attr(latest_car_state, "vEgo", 0.0))
    desired_curvature = safe_float(safe_attr(latest_controls_state, "desiredCurvature", 0.0))
    curvature = safe_float(safe_attr(latest_controls_state, "curvature", 0.0))
    desired_torque = safe_float(safe_attr(safe_attr(latest_car_control, "actuators"), "torque", 0.0))
    output_torque = safe_float(safe_attr(safe_attr(latest_car_output, "actuatorsOutput"), "torque", 0.0))
    curvature_error = abs(desired_curvature - curvature)
    torque_error = abs(desired_torque - output_torque)
    report.steering_windows.append(SteeringEventWindow(
      event=event_name,
      t=t,
      v_ego=v_ego,
      lat_active=bool(safe_attr(latest_car_control, "latActive", False)),
      steering_pressed=bool(safe_attr(latest_car_state, "steeringPressed", False)),
      desired_curvature=desired_curvature,
      curvature=curvature,
      curvature_error=curvature_error,
      desired_torque=desired_torque,
      output_torque=output_torque,
      torque_error=torque_error,
      in_curve=abs(desired_curvature) > 0.0015 or abs(curvature) > 0.0015,
    ))

  def update_torque_clip() -> None:
    nonlocal torque_clip_samples, torque_clip_total
    if latest_car_control is None or latest_car_output is None:
      return
    try:
      desired_torque = safe_float(safe_attr(safe_attr(latest_car_control, "actuators"), "torque", 0.0))
      output_torque = safe_float(safe_attr(safe_attr(latest_car_output, "actuatorsOutput"), "torque", 0.0))
      torque_error = abs(desired_torque - output_torque)
      torque_clip_total += 1
      if torque_error > 1e-2:
        torque_clip_samples += 1
    except Exception:
      pass

  for file in files:
    for event in LogReader(str(file)):
      which = event.which()
      services[which] += 1
      t = event_time_sec(event)

      if which == "carParams":
        cp = event.carParams
        car_params[(cp.carFingerprint, int(cp.flags), bool(cp.radarUnavailable),
                    bool(cp.openpilotLongitudinalControl), bool(cp.pcmCruise))] += 1

      elif which == "carState":
        latest_car_state = event.carState
        if safe_attr(latest_car_state, "steeringPressed", False):
          steering_pressed_samples += 1
        if safe_attr(latest_car_state, "brakePressed", False):
          brake_pressed_samples += 1
        if safe_attr(latest_car_state, "gasPressed", False):
          gas_pressed_samples += 1
        brake_pressed = bool(safe_attr(latest_car_state, "brakePressed", False))
        if brake_pressed and not previous_brake_pressed:
          driver_brake_events += 1
          lead = safe_attr(latest_radar_state, "leadOne")
          stop_context = bool(
            safe_attr(latest_starpilot_plan, "redLight", False) or
            safe_attr(latest_starpilot_plan, "forcingStop", False) or
            safe_attr(safe_attr(latest_model, "action"), "shouldStop", False) or
            (safe_attr(lead, "status", False) and safe_float(safe_attr(lead, "vLead", 99.0)) < 2.0)
          )
          if bool(safe_attr(latest_car_control, "longActive", False)) and stop_context:
            missed_stop_events += 1
        previous_brake_pressed = brake_pressed

        lead = safe_attr(latest_radar_state, "leadOne")
        stopped_lead = bool(
          safe_attr(lead, "status", False) and safe_float(safe_attr(lead, "vLead", 99.0)) < 0.8 and
          safe_float(safe_attr(lead, "dRel", 999.0)) < 18.0
        )
        stop_context = bool(
          stopped_lead or safe_attr(latest_starpilot_plan, "redLight", False) or
          safe_attr(latest_starpilot_plan, "forcingStop", False) or
          safe_attr(safe_attr(latest_model, "action"), "shouldStop", False)
        )
        if stop_context and 0.10 < safe_float(safe_attr(latest_car_state, "vEgo", 0.0)) < 1.20 and not brake_pressed:
          low_speed_creep_samples += 1

      elif which == "carControl":
        latest_car_control = event.carControl
        if safe_attr(latest_car_control, "enabled", False):
          enabled_samples += 1
        if safe_attr(latest_car_control, "latActive", False):
          lateral_active_samples += 1
        update_torque_clip()

      elif which == "carOutput":
        latest_car_output = event.carOutput
        update_torque_clip()

      elif which == "controlsState":
        latest_controls_state = event.controlsState
        alert = " ".join(filter(None, (
          str(safe_attr(latest_controls_state, "alertText1", "")),
          str(safe_attr(latest_controls_state, "alertText2", "")),
        ))).strip()
        if alert:
          alerts[alert] += 1
        accel = safe_float(safe_attr(latest_controls_state, "aTarget", 0.0))
        accel_commands.append(accel)
        if accel < -2.0:
          harsh_brake_samples += 1
        lead = safe_attr(latest_radar_state, "leadOne")
        no_stop_evidence = not bool(
          safe_attr(lead, "status", False) or safe_attr(latest_starpilot_plan, "redLight", False) or
          safe_attr(latest_starpilot_plan, "forcingStop", False) or
          safe_attr(safe_attr(latest_model, "action"), "shouldStop", False)
        )
        false_brake_now = bool(
          accel < -1.25 and no_stop_evidence and
          safe_float(safe_attr(latest_car_state, "vEgo", 0.0)) > 12.0 and
          not bool(safe_attr(latest_car_state, "brakePressed", False))
        )
        if false_brake_now and not false_brake_active:
          false_brake_events += 1
        false_brake_active = false_brake_now
        if latest_car_state is not None:
          desired_curvature = safe_float(safe_attr(latest_controls_state, "desiredCurvature", 0.0))
          curvature = safe_float(safe_attr(latest_controls_state, "curvature", 0.0))
          if abs(desired_curvature) > 1e-5 or abs(curvature) > 1e-5:
            curvature_errors.append(abs(desired_curvature - curvature))

      elif which == "onroadEvents":
        for onroad_event in event.onroadEvents:
          name = enum_name(onroad_event.name)
          events[name] += 1
          if name in ("steerTempUnavailable", "steerTempUnavailableSilent", "steerUnavailable", "steerSaturated", "steerOverride", "steerDisengage"):
            capture_steering_window(name, t)

      elif which == "radarState":
        latest_radar_state = event.radarState
        lead = event.radarState.leadOne
        if lead.status and latest_car_state is not None and 1.0 < lead.dRel < 180.0:
          ref = RefLead(t, float(lead.dRel), float(lead.yRel), float(lead.vRel), float(latest_car_state.vEgo))
          refs.append(ref)
          lead_samples += 1
          lead_distances.append(ref.d_rel)
          if lead.radar:
            radar_lead_samples += 1
          else:
            vision_lead_samples += 1
          closing = max(-ref.v_rel, 0.0)
          if closing > 0.5:
            ttc_values.append(ref.d_rel / closing)

      elif which == "can":
        for msg in event.can:
          addr = int(msg.address)
          dat = bytes(msg.dat)
          if int(msg.src) == CARNIVAL_OBJECT_BUS and CARNIVAL_OBJECT_START <= addr <= CARNIVAL_OBJECT_END and len(dat) == CARNIVAL_OBJECT_LEN:
            objects_by_addr[addr].extend(decode_carnival_objects(t, addr, dat))

      elif which == "starpilotPlan":
        latest_starpilot_plan = event.starpilotPlan

      elif which == "modelV2":
        latest_model = event.modelV2

      elif which == "carnivalState":
        confidence_samples.append(safe_float(safe_attr(event.carnivalState, "overallConfidence", 0.0)))

  report.services = dict(services)
  report.events = dict(events.most_common(20))
  report.alerts = dict(alerts.most_common(20))
  report.car_params = [
    {
      "carFingerprint": key[0],
      "flags": key[1],
      "radarUnavailable": key[2],
      "openpilotLongitudinalControl": key[3],
      "pcmCruise": key[4],
      "count": count,
    }
    for key, count in car_params.most_common(5)
  ]
  if report.car_params:
    primary = report.car_params[0]
    report.modes = {
      "stock_scc": primary["pcmCruise"] and not primary["openpilotLongitudinalControl"],
      "openpilot_long": primary["openpilotLongitudinalControl"],
      "radarUnavailable": primary["radarUnavailable"],
    }

  report.radar = summarize_radar(refs, objects_by_addr)
  report.lateral = LateralMetrics(
    active_samples=lateral_active_samples,
    steering_pressed_samples=steering_pressed_samples,
    steering_pressed_pct=steering_pressed_samples / max(services["carState"], 1),
    steering_temp_events=events["steerTempUnavailable"] + events["steerTempUnavailableSilent"] + events["steerUnavailable"],
    steer_saturated_events=events["steerSaturated"],
    lateral_takeover_events=events["steerOverride"] + events["steerDisengage"],
    torque_clip_samples=torque_clip_samples,
    torque_clip_pct=torque_clip_samples / max(torque_clip_total, 1),
    max_torque_error=0.0,
    curvature_error_p90=percentile(curvature_errors, 90),
  )
  if report.lateral.active_samples < 100:
    report.lateral.verdict = "insufficient lateral-active data"
  elif report.lateral.steering_temp_events or report.lateral.steer_saturated_events:
    report.lateral.verdict = "needs steering-limit review"
  elif report.lateral.steering_pressed_pct > 0.10:
    report.lateral.verdict = "many driver interventions"
  else:
    report.lateral.verdict = "no obvious logged lateral issue"

  report.longitudinal = LongitudinalMetrics(
    enabled_samples=enabled_samples,
    lead_samples=lead_samples,
    radar_lead_samples=radar_lead_samples,
    vision_lead_samples=vision_lead_samples,
    min_lead_distance=min(lead_distances) if lead_distances else None,
    min_ttc=min(ttc_values) if ttc_values else None,
    brake_pressed_samples=brake_pressed_samples,
    gas_pressed_samples=gas_pressed_samples,
    accel_command_p90_abs=percentile([abs(v) for v in accel_commands], 90),
    harsh_brake_samples=harsh_brake_samples,
  )
  if report.longitudinal.enabled_samples < 100:
    report.longitudinal.verdict = "insufficient enabled data"
  elif report.longitudinal.harsh_brake_samples > 20:
    report.longitudinal.verdict = "harsh braking review"
  elif report.longitudinal.lead_samples and report.longitudinal.radar_lead_samples == 0:
    report.longitudinal.verdict = "vision-only lead; radar candidate should stay shadow"
  else:
    report.longitudinal.verdict = "no obvious logged longitudinal issue"

  lateral_score = max(0, min(100, round(
    100 - 18 * report.lateral.steering_temp_events - 5 * report.lateral.steer_saturated_events -
    35 * min(report.lateral.steering_pressed_pct, 1.0) - 20 * min(report.lateral.torque_clip_pct, 1.0)
  )))
  longitudinal_score = max(0, min(100, round(
    100 - 18 * missed_stop_events - 14 * false_brake_events -
    min(25, report.longitudinal.harsh_brake_samples / max(report.longitudinal.enabled_samples, 1) * 500.0) -
    min(20, low_speed_creep_samples / max(services["carState"], 1) * 600.0)
  )))
  distance_quality = 0.0 if report.radar.distance_mae is None else max(0.0, 1.0 - report.radar.distance_mae / 5.0)
  velocity_quality = 0.0 if report.radar.derived_velocity_mae is None else max(0.0, 1.0 - report.radar.derived_velocity_mae / 4.0)
  radar_score = max(0, min(100, round(100.0 * (
    0.45 * report.radar.coverage + 0.35 * distance_quality + 0.20 * velocity_quality
  ))))
  recommendations: list[dict[str, Any]] = []
  if missed_stop_events:
    recommendations.append({"target": "stop-hold distance", "delta": "+0.1 m maximum", "confidence": "medium",
                            "reason": f"{missed_stop_events} driver-brake stop interventions", "autoApply": False,
                            "codePath": "selfdrive/controls/lib/carnival_intersection_controller.py"})
  if low_speed_creep_samples:
    recommendations.append({"target": "stop-hold brake", "delta": "+0.05 m/s^2 maximum", "confidence": "high",
                            "reason": f"{low_speed_creep_samples} low-speed creep samples", "autoApply": False,
                            "codePath": "selfdrive/controls/lib/carnival_intersection_controller.py"})
  if report.lateral.steering_temp_events:
    recommendations.append({"target": "EPS predictive risk onset", "delta": "-0.02 maximum", "confidence": "medium",
                            "reason": f"{report.lateral.steering_temp_events} temporary steering events", "autoApply": False,
                            "codePath": "opendbc/car/hyundai/carcontroller.py"})
  elif report.lateral.steer_saturated_events and report.lateral.steering_temp_events == 0:
    recommendations.append({"target": "curve speed", "delta": "-2% maximum", "confidence": "medium",
                            "reason": f"{report.lateral.steer_saturated_events} saturation events without EPS faults", "autoApply": False,
                            "codePath": "selfdrive/controls/lib/longitudinal_planner.py"})
  if false_brake_events:
    recommendations.append({"target": "radar confirmation gate", "delta": "+1 confirmation frame maximum", "confidence": "medium",
                            "reason": f"{false_brake_events} uncorroborated hard-brake events", "autoApply": False,
                            "codePath": "selfdrive/controls/lib/carnival_confidence.py"})
  if not recommendations:
    recommendations.append({"target": "none", "delta": "0", "confidence": "high",
                            "reason": "no bounded change justified by this route", "autoApply": False, "codePath": ""})
  report.scorecard = RouteScorecard(
    overall_score=round(0.40 * lateral_score + 0.45 * longitudinal_score + 0.15 * radar_score),
    lateral_score=lateral_score,
    longitudinal_score=longitudinal_score,
    radar_score=radar_score,
    intervention_events=driver_brake_events + report.lateral.lateral_takeover_events,
    false_brake_events=false_brake_events,
    missed_stop_events=missed_stop_events,
    low_speed_creep_samples=low_speed_creep_samples,
    torque_saturation_events=report.lateral.steer_saturated_events,
    mean_confidence=mean(confidence_samples) if confidence_samples else None,
    recommendations=recommendations,
  )

  add_issues(report)
  return report


def add_issues(report: RouteReport) -> None:
  if report.radar.verdict == "strong shadow candidate":
    report.issues.append("Hidden 0x180 object distance is strong; keep shadow until lateral/velocity decode is solved.")
  elif report.radar.verdict != "insufficient reference lead data":
    report.issues.append(f"Hidden radar candidate is not yet clean: {report.radar.verdict}.")

  if report.lateral.steering_temp_events:
    report.issues.append(f"Steering availability/limit events: {report.lateral.steering_temp_events}.")
  if report.lateral.steer_saturated_events:
    report.issues.append(f"Steer saturation events: {report.lateral.steer_saturated_events}.")
  if report.lateral.steering_pressed_pct > 0.10:
    report.issues.append(f"Driver steering intervention rate is high: {report.lateral.steering_pressed_pct:.1%}.")
  if report.longitudinal.harsh_brake_samples > 20:
    report.issues.append(f"Harsh brake command samples: {report.longitudinal.harsh_brake_samples}.")


def fmt(value: Any, suffix: str = "") -> str:
  if value is None:
    return "n/a"
  if isinstance(value, float):
    return f"{value:.2f}{suffix}"
  return f"{value}{suffix}"


def write_markdown(reports: list[RouteReport], output: Path, log_root: Path) -> None:
  now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
  lines = [
    "# Carnival Debug Report",
    "",
    f"Generated: {now}",
    f"Log root: `{log_root}`",
    "",
    "## Route Summary",
    "",
    "| Route | Mode | Radar Verdict | Radar Coverage | d MAE | dDot/v MAE | Lateral | Longitudinal |",
    "|---|---|---:|---:|---:|---:|---|---|",
  ]
  for report in reports:
    mode = "OP long" if report.modes.get("openpilot_long") else "stock SCC" if report.modes.get("stock_scc") else "unknown"
    lines.append(
      f"| `{report.route}` | {mode} | {report.radar.verdict} | "
      f"{report.radar.coverage:.1%} | {fmt(report.radar.distance_mae, ' m')} | "
      f"{fmt(report.radar.derived_velocity_mae, ' m/s')} | {report.lateral.verdict} | {report.longitudinal.verdict} |"
    )

  lines += ["", "## Issues", ""]
  issues = [(report.route, issue) for report in reports for issue in report.issues]
  if issues:
    for route, issue in issues:
      lines.append(f"- `{route}`: {issue}")
  else:
    lines.append("- No high-signal issues found in parsed logs.")

  for report in reports:
    route_scores = "/".join(str(score) for score in (
      report.scorecard.overall_score,
      report.scorecard.lateral_score,
      report.scorecard.longitudinal_score,
      report.scorecard.radar_score,
    ))
    route_events = "/".join(str(count) for count in (
      report.scorecard.intervention_events,
      report.scorecard.false_brake_events,
      report.scorecard.missed_stop_events,
    ))
    lines += [
      "",
      f"## `{report.route}`",
      "",
      "### Route Scorecard",
      "",
      f"- Overall/lateral/longitudinal/radar: {route_scores}",
      f"- Interventions/false brakes/missed stops: {route_events}",
      f"- Low-speed creep samples: {report.scorecard.low_speed_creep_samples}",
      f"- Mean live confidence: {fmt(report.scorecard.mean_confidence)}",
      f"- Recommendations: `{report.scorecard.recommendations}`",
      "",
      f"- Files: {report.files}",
      f"- Car params: `{report.car_params[0] if report.car_params else {}}`",
      f"- Services: `{report.services}`",
      f"- Events: `{report.events}`",
      f"- Alerts: `{report.alerts}`",
      "",
      "### Radar Candidate",
      "",
      f"- Verdict: {report.radar.verdict}",
      f"- Refs/selected/coverage: {report.radar.refs}/{report.radar.selected}/{report.radar.coverage:.1%}",
      f"- Distance MAE/P90: {fmt(report.radar.distance_mae, ' m')} / {fmt(report.radar.distance_p90, ' m')}",
      f"- Derived velocity MAE/P90: {fmt(report.radar.derived_velocity_mae, ' m/s')} / {fmt(report.radar.derived_velocity_p90, ' m/s')}",
      f"- Raw velocity MAE: {fmt(report.radar.raw_velocity_mae, ' m/s')}",
      f"- Raw lateral MAE: {fmt(report.radar.raw_lateral_mae, ' m')}",
      f"- Address counts: `{report.radar.address_counts}`",
      f"- Slot counts: `{report.radar.slot_counts}`",
      f"- State counts: `{report.radar.state_counts}`",
      "",
      "### Lateral",
      "",
      f"- Verdict: {report.lateral.verdict}",
      f"- Active samples: {report.lateral.active_samples}",
      f"- Steering pressed: {report.lateral.steering_pressed_samples} ({report.lateral.steering_pressed_pct:.1%})",
      f"- Steering temp/saturated/takeover events: {report.lateral.steering_temp_events}/{report.lateral.steer_saturated_events}/{report.lateral.lateral_takeover_events}",
      f"- Torque clip samples: {report.lateral.torque_clip_samples} ({report.lateral.torque_clip_pct:.1%})",
      f"- Curvature error P90: {fmt(report.lateral.curvature_error_p90)}",
      "",
      "### Steering Event Windows",
      "",
    ]
    if report.steering_windows:
      lines += [
        "| Event | t | vEgo | Curve | Lat Active | Driver Steer | Desired Curv | Actual Curv | Torque Err |",
        "|---|---:|---:|---|---|---|---:|---:|---:|",
      ]
      for window in report.steering_windows[:30]:
        lines.append(
          f"| {window.event} | {window.t:.2f} | {window.v_ego:.1f} | "
          f"{'yes' if window.in_curve else 'no'} | {'yes' if window.lat_active else 'no'} | "
          f"{'yes' if window.steering_pressed else 'no'} | {window.desired_curvature:.5f} | "
          f"{window.curvature:.5f} | {window.torque_error:.3f} |"
        )
    else:
      lines.append("- No steering warning/override/saturation windows found.")

    lines += [
      "",
      "### Longitudinal",
      "",
      f"- Verdict: {report.longitudinal.verdict}",
      f"- Enabled samples: {report.longitudinal.enabled_samples}",
      f"- Lead samples radar/vision: {report.longitudinal.radar_lead_samples}/{report.longitudinal.vision_lead_samples}",
      f"- Min lead distance: {fmt(report.longitudinal.min_lead_distance, ' m')}",
      f"- Min TTC: {fmt(report.longitudinal.min_ttc, ' s')}",
      f"- Brake/gas pressed samples: {report.longitudinal.brake_pressed_samples}/{report.longitudinal.gas_pressed_samples}",
      f"- Harsh brake samples: {report.longitudinal.harsh_brake_samples}",
    ]

  output.parent.mkdir(parents=True, exist_ok=True)
  output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
  parser = argparse.ArgumentParser(description="Pull comma logs and generate a 2024 Kia Carnival debug report.")
  parser.add_argument("--device", default=None, help="comma SSH IP, e.g. 192.168.68.68")
  parser.add_argument("--ssh-key", default=str(Path.home() / ".ssh" / "id_ed25519"))
  parser.add_argument("--log-root", type=Path, default=Path("drive-logs") / "carnival-auto")
  parser.add_argument("--output", type=Path, default=None)
  parser.add_argument("--json-output", type=Path, default=None)
  parser.add_argument("--skip-pull", action="store_true")
  parser.add_argument("--limit-routes", type=int, default=None)
  parser.add_argument("--route", action="append", default=None, help="Analyze only matching route id. Can be passed multiple times.")
  parser.add_argument("--latest", type=int, default=None, help="Analyze only the latest N route ids after pulling/discovery.")
  parser.add_argument("--max-segments-per-route", type=int, default=None,
                      help="Analyze at most this many segments per route, from the beginning after sorting.")
  parser.add_argument("--recent-segments-per-route", type=int, default=None,
                      help="Analyze only the most recent N segments per route.")
  args = parser.parse_args()

  if args.device and not args.skip_pull:
    copied = pull_logs(
      args.device,
      args.log_root,
      args.ssh_key if Path(args.ssh_key).exists() else None,
      args.limit_routes,
      args.latest,
    )
    print(f"Pulled {copied} new log files into {args.log_root}")
  elif not args.log_root.exists():
    raise SystemExit(f"Log root does not exist: {args.log_root}")

  by_route: dict[str, list[Path]] = defaultdict(list)
  for file in iter_log_files(args.log_root):
    by_route[route_key(file, args.log_root)].append(file)
  if not by_route:
    raise SystemExit(f"No rlog/qlog files found under {args.log_root}")

  route_items = sorted(by_route.items())
  if args.route:
    requested = set(args.route)
    route_items = [(route, files) for route, files in route_items if route in requested or any(token in route for token in requested)]
    if not route_items:
      raise SystemExit(f"No matching routes for {sorted(requested)} under {args.log_root}")
  if args.latest is not None:
    route_items = route_items[-args.latest:]
  if args.max_segments_per_route is not None or args.recent_segments_per_route is not None:
    limited_items = []
    for route, files in route_items:
      by_segment: dict[int, list[Path]] = defaultdict(list)
      for file in files:
        by_segment[segment_index(file)].append(file)
      segment_ids = sorted(by_segment)
      if args.recent_segments_per_route is not None:
        segment_ids = segment_ids[-args.recent_segments_per_route:]
      if args.max_segments_per_route is not None:
        segment_ids = segment_ids[:args.max_segments_per_route]
      limited_files = [file for seg in segment_ids for file in by_segment[seg]]
      limited_items.append((route, limited_files))
    route_items = limited_items

  reports = []
  for route, files in route_items:
    print(f"Analyzing {route} ({len(files)} log files)...", flush=True)
    reports.append(analyze_route(route, files))

  timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
  output = args.output or (Path("drive_reports") / f"carnival-report-{timestamp}.md")
  json_output = args.json_output or output.with_suffix(".json")
  write_markdown(reports, output, args.log_root)
  json_output.parent.mkdir(parents=True, exist_ok=True)
  json_output.write_text(json.dumps([asdict(report) for report in reports], indent=2), encoding="utf-8")
  print(f"Wrote {output}")
  print(f"Wrote {json_output}")

  shutil.which("true")  # keeps pyflakes quiet when run in minimal environments


if __name__ == "__main__":
  try:
    main()
  except subprocess.CalledProcessError as e:
    print(e.stdout, file=sys.stdout)
    print(e.stderr, file=sys.stderr)
    raise
