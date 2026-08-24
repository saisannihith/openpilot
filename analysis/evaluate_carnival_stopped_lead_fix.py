#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if (ROOT / "tools").is_dir() and (ROOT / "common").is_dir():
  OPENPILOT_ROOT = ROOT
else:
  OPENPILOT_ROOT = ROOT / "openpilot"
sys.path.insert(0, str(OPENPILOT_ROOT))


def install_openpilot_namespace() -> None:
  namespace = types.ModuleType("openpilot")
  namespace.__path__ = [str(OPENPILOT_ROOT)]  # type: ignore[attr-defined]
  sys.modules["openpilot"] = namespace


try:
  from openpilot.tools.lib.logreader import LogReader, ReadMode
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
except ModuleNotFoundError:
  install_openpilot_namespace()
  from openpilot.tools.lib.logreader import LogReader, ReadMode
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner


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
      except Exception:
        pass
  return -1


def route_name(path: Path) -> str:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 2:
      return "--".join(parts[:2])
  return path.parent.name


def make_cp(car_fingerprint: str = "KIA_CARNIVAL_4TH_GEN") -> SimpleNamespace:
  return SimpleNamespace(
    carFingerprint=car_fingerprint,
    longitudinalActuatorDelay=0.2,
    radarDelay=0.0,
    brand="hyundai",
    openpilotLongitudinalControl=True,
    experimentalLongitudinalAvailable=True,
    flags=0,
  )


@dataclass
class EvalSample:
  route: str
  segment: int
  route_t: float
  long_active: bool
  v_ego: float
  standstill: bool
  brake_pressed: bool
  gas_pressed: bool
  lead_index: int
  lead_radar: bool
  lead_track_id: int
  lead_d_rel: float
  lead_y_rel: float
  lead_v_lead: float
  lead_v_rel: float
  lead_a_lead_k: float
  source: str
  should_stop: bool
  plan_accel: float
  cmd_accel: float
  guard_cap: float | None
  stop_hold_cap: float | None


def expand_paths(patterns: list[str]) -> list[Path]:
  out: list[Path] = []
  for pattern in patterns:
    matched = sorted(Path("/").glob(pattern.lstrip("/")) if pattern.startswith("/") else Path().glob(pattern))
    out.extend(path for path in matched if path.is_file())
  return sorted(set(out))


def evaluate_file(path: Path, mode: ReadMode, planner: LongitudinalPlanner) -> list[EvalSample]:
  latest: dict[str, Any] = {}
  samples: list[EvalSample] = []
  start_ns: int | None = None
  seg = segment_number(path)
  stop_distance = STOP_DISTANCE - 0.5

  for msg in LogReader(str(path), default_mode=mode, sort_by_time=True):
    mono_time = int(msg.logMonoTime)
    which = msg.which()
    if start_ns is None and which in ("carState", "longitudinalPlan", "radarState"):
      start_ns = mono_time
    if which in ("carState", "carControl", "longitudinalPlan", "radarState"):
      latest[which] = getattr(msg, which)
    if which != "longitudinalPlan" or start_ns is None:
      continue
    if not all(key in latest for key in ("carState", "carControl", "longitudinalPlan", "radarState")):
      continue

    car_state = latest["carState"]
    car_control = latest["carControl"]
    long_plan = latest["longitudinalPlan"]
    radar_state = latest["radarState"]
    actuators = safe_attr(car_control, "actuators")
    route_t = max(seg, 0) * 60.0 + (mono_time - start_ns) / 1e9
    v_ego = safe_float(safe_attr(car_state, "vEgo", 0.0))
    for lead_index, lead in enumerate((radar_state.leadOne, radar_state.leadTwo)):
      if not bool(safe_attr(lead, "status", False)):
        continue
      cap = planner.get_standstill_stopped_lead_guard_cap(
        lead,
        v_ego,
        accel_min=-2.0,
        stop_distance=stop_distance,
        release_ready=False,
        confident_depart_ready=False,
      )
      stop_hold_cap = planner.get_carnival_radar_stop_hold_cap(
        (radar_state.leadOne, radar_state.leadTwo),
        v_ego,
        accel_min=-2.0,
        driver_gas=bool(safe_attr(car_state, "gasPressed", False)),
      )
      samples.append(EvalSample(
        route=route_name(path),
        segment=seg,
        route_t=route_t,
        long_active=bool(safe_attr(car_control, "longActive", False)),
        v_ego=v_ego,
        standstill=bool(safe_attr(car_state, "standstill", False)),
        brake_pressed=bool(safe_attr(car_state, "brakePressed", False)),
        gas_pressed=bool(safe_attr(car_state, "gasPressed", False)),
        lead_index=lead_index,
        lead_radar=bool(safe_attr(lead, "radar", False)),
        lead_track_id=int(safe_attr(lead, "radarTrackId", -1)),
        lead_d_rel=safe_float(safe_attr(lead, "dRel", 0.0)),
        lead_y_rel=safe_float(safe_attr(lead, "yRel", 0.0)),
        lead_v_lead=safe_float(safe_attr(lead, "vLead", 0.0)),
        lead_v_rel=safe_float(safe_attr(lead, "vRel", 0.0)),
        lead_a_lead_k=safe_float(safe_attr(lead, "aLeadK", 0.0)),
        source=str(safe_attr(long_plan, "longitudinalPlanSource", "unknown")),
        should_stop=bool(safe_attr(long_plan, "shouldStop", False)),
        plan_accel=safe_float(safe_attr(long_plan, "aTarget", 0.0)),
        cmd_accel=safe_float(safe_attr(actuators, "accel", 0.0)),
        guard_cap=cap,
        stop_hold_cap=stop_hold_cap,
      ))
  return samples


def group_close_events(samples: list[EvalSample]) -> list[list[EvalSample]]:
  close = [
    sample for sample in sorted(samples, key=lambda s: (s.route, s.route_t, s.lead_index))
    if sample.lead_d_rel <= 12.0 and abs(sample.lead_y_rel) <= 1.75 and sample.v_ego <= 5.8
  ]
  events: list[list[EvalSample]] = []
  cur: list[EvalSample] = []
  prev: EvalSample | None = None
  for sample in close:
    if cur and (prev is None or sample.route != prev.route or sample.route_t - prev.route_t > 3.0):
      events.append(cur)
      cur = []
    cur.append(sample)
    prev = sample
  if cur:
    events.append(cur)
  return events


def summarize_event(event: list[EvalSample]) -> dict[str, Any]:
  first = event[0]
  guard = [sample for sample in event if sample.guard_cap is not None or sample.stop_hold_cap is not None]
  stop_hold = [sample for sample in event if sample.stop_hold_cap is not None]
  manual = [sample for sample in event if sample.brake_pressed or sample.gas_pressed]
  min_d = min(sample.lead_d_rel for sample in event)
  cap_values = [
    cap
    for sample in guard
    for cap in (sample.guard_cap, sample.stop_hold_cap)
    if cap is not None
  ]
  return {
    "route": first.route,
    "segments": sorted(set(sample.segment for sample in event)),
    "startRouteT": round(first.route_t, 3),
    "endRouteT": round(event[-1].route_t, 3),
    "minLeadDRel": round(min_d, 3),
    "minSpeedMps": round(min(sample.v_ego for sample in event), 3),
    "maxSpeedMps": round(max(sample.v_ego for sample in event), 3),
    "radarFrames": sum(sample.lead_radar for sample in event),
    "trackIds": sorted(set(sample.lead_track_id for sample in event if sample.lead_track_id >= 0)),
    "manualRouteT": None if not manual else round(manual[0].route_t, 3),
    "firstGuardRouteT": None if not guard else round(guard[0].route_t, 3),
    "firstGuardLeadDRel": None if not guard else round(guard[0].lead_d_rel, 3),
    "firstGuardSpeedMps": None if not guard else round(guard[0].v_ego, 3),
    "firstStopHoldRouteT": None if not stop_hold else round(stop_hold[0].route_t, 3),
    "firstStopHoldLeadDRel": None if not stop_hold else round(stop_hold[0].lead_d_rel, 3),
    "firstStopHoldSpeedMps": None if not stop_hold else round(stop_hold[0].v_ego, 3),
    "strongestGuardCap": None if not cap_values else round(min(cap_values), 3),
    "guardBeforeManual": bool(guard and (not manual or guard[0].route_t < manual[0].route_t)),
    "stopHoldBeforeManual": bool(stop_hold and (not manual or stop_hold[0].route_t < manual[0].route_t)),
    "tail": [
      {
        "routeT": round(sample.route_t, 3),
        "vEgo": round(sample.v_ego, 3),
        "dRel": round(sample.lead_d_rel, 3),
        "vLead": round(sample.lead_v_lead, 3),
        "radar": sample.lead_radar,
        "trackId": sample.lead_track_id,
        "brake": sample.brake_pressed,
        "gas": sample.gas_pressed,
        "shouldStop": sample.should_stop,
        "guardCap": None if sample.guard_cap is None else round(sample.guard_cap, 3),
        "stopHoldCap": None if sample.stop_hold_cap is None else round(sample.stop_hold_cap, 3),
      }
      for sample in event[-12:]
    ],
  }


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--mode", choices=("qlog", "rlog"), default="qlog")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  mode = ReadMode.QLOG if args.mode == "qlog" else ReadMode.RLOG
  planner = LongitudinalPlanner(make_cp(), init_v=0.0)
  samples: list[EvalSample] = []
  failures = []
  for path in expand_paths(args.logs):
    try:
      samples.extend(evaluate_file(path, mode, planner))
    except Exception as exc:
      failures.append({"path": str(path), "error": str(exc)})

  events = group_close_events(samples)
  payload = {
    "samples": len(samples),
    "events": [summarize_event(event) for event in events],
    "readFailures": failures,
  }
  text = json.dumps(payload, indent=2, sort_keys=True)
  print(text)
  if args.out:
    args.out.write_text(text + "\n")


if __name__ == "__main__":
  main()
