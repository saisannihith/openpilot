#!/usr/bin/env python3
from __future__ import annotations

import sys
import types
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
if (ROOT / "tools").is_dir() and (ROOT / "common").is_dir():
  OPENPILOT_ROOT = ROOT
else:
  OPENPILOT_ROOT = ROOT / "openpilot"
sys.path.insert(0, str(OPENPILOT_ROOT))

try:
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
except ModuleNotFoundError:
  namespace = types.ModuleType("openpilot")
  namespace.__path__ = [str(OPENPILOT_ROOT)]  # type: ignore[attr-defined]
  sys.modules["openpilot"] = namespace
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner


def make_cp(car_fingerprint: str):
  return SimpleNamespace(
    carFingerprint=car_fingerprint,
    longitudinalActuatorDelay=0.2,
    radarDelay=0.0,
    brand="hyundai",
    openpilotLongitudinalControl=True,
    experimentalLongitudinalAvailable=True,
    flags=0,
  )


def make_lead(**kwargs):
  defaults = dict(
    status=True,
    dRel=7.4,
    vLead=0.08,
    aLeadK=-0.04,
    radar=True,
    modelProb=1.0,
    yRel=-0.12,
  )
  defaults.update(kwargs)
  return SimpleNamespace(**defaults)


def main() -> None:
  carnival = LongitudinalPlanner(make_cp("KIA_CARNIVAL_4TH_GEN"), init_v=1.9)
  confirmation_lead = make_lead()
  confirmation_lead.radarTrackId = 0xC4101

  stop_hold_cap = carnival.get_carnival_radar_stop_hold_cap(
    (confirmation_lead, make_lead(status=False)),
    v_ego=1.9,
    accel_min=-2.0,
    driver_gas=False,
    release_ready=False,
  )
  assert carnival.carnival_radar_stop_hold_active, stop_hold_cap
  assert stop_hold_cap is not None and stop_hold_cap <= -0.55, stop_hold_cap

  vision_lead = make_lead(dRel=6.2, vLead=0.2, aLeadK=0.0, radar=False, modelProb=0.99)
  latched_cap = carnival.get_carnival_radar_stop_hold_cap(
    (vision_lead, make_lead(status=False)),
    v_ego=0.4,
    accel_min=-2.0,
    driver_gas=False,
    release_ready=False,
  )
  assert carnival.carnival_radar_stop_hold_active, latched_cap
  assert latched_cap is not None, latched_cap

  released_cap = carnival.get_carnival_radar_stop_hold_cap(
    (vision_lead, make_lead(status=False)),
    v_ego=0.4,
    accel_min=-2.0,
    driver_gas=True,
    release_ready=False,
  )
  assert released_cap is None, released_cap
  assert not carnival.carnival_radar_stop_hold_active

  carnival_cap = carnival.get_standstill_stopped_lead_guard_cap(
    confirmation_lead,
    v_ego=1.9,
    accel_min=-2.0,
    stop_distance=5.5,
    release_ready=False,
    confident_depart_ready=False,
  )
  assert carnival_cap is not None, carnival_cap
  assert carnival_cap <= -0.45, carnival_cap

  generic = LongitudinalPlanner(make_cp("HONDA_CIVIC"), init_v=1.9)
  generic_cap = generic.get_standstill_stopped_lead_guard_cap(
    make_lead(),
    v_ego=1.9,
    accel_min=-2.0,
    stop_distance=5.5,
    release_ready=False,
    confident_depart_ready=False,
  )
  assert generic_cap is None, generic_cap

  reject_cases = (
    dict(name="highway-speed", v_ego=12.0, lead=make_lead()),
    dict(name="far-lead", v_ego=1.9, lead=make_lead(dRel=14.0)),
    dict(name="moving-lead", v_ego=1.9, lead=make_lead(vLead=2.5)),
    dict(name="adjacent-lane", v_ego=1.9, lead=make_lead(yRel=2.2)),
    dict(name="wrong-track", v_ego=1.9, lead=make_lead()),
  )
  reject_cases[-1]["lead"].radarTrackId = 0x401
  for case in reject_cases:
    carnival.carnival_radar_stop_hold_active = False
    cap = carnival.get_carnival_radar_stop_hold_cap(
      (case["lead"], make_lead(status=False)),
      v_ego=case["v_ego"],
      accel_min=-2.0,
      driver_gas=False,
      release_ready=False,
    )
    assert cap is None, (case["name"], cap)
    assert not carnival.carnival_radar_stop_hold_active, case["name"]

  carnival.carnival_radar_stop_hold_active = False
  assert carnival.get_carnival_radar_stop_hold_cap(
    (confirmation_lead, make_lead(status=False)),
    v_ego=1.9,
    accel_min=-2.0,
    driver_gas=False,
    release_ready=False,
  ) is not None
  departed_cap = carnival.get_carnival_radar_stop_hold_cap(
    (make_lead(dRel=11.5, vLead=2.2, aLeadK=0.4, radar=False, modelProb=0.99, yRel=-0.1), make_lead(status=False)),
    v_ego=2.0,
    accel_min=-2.0,
    driver_gas=False,
    release_ready=False,
  )
  assert departed_cap is None, departed_cap
  assert not carnival.carnival_radar_stop_hold_active

  carnival.carnival_radar_stop_hold_active = False
  assert carnival.get_carnival_radar_stop_hold_cap(
    (confirmation_lead, make_lead(status=False)),
    v_ego=1.9,
    accel_min=-2.0,
    driver_gas=False,
    release_ready=False,
  ) is not None
  release_ready_cap = carnival.get_carnival_radar_stop_hold_cap(
    (make_lead(dRel=7.2, vLead=0.4, aLeadK=0.2, radar=True, modelProb=1.0, yRel=-0.1), make_lead(status=False)),
    v_ego=0.2,
    accel_min=-2.0,
    driver_gas=False,
    release_ready=True,
  )
  assert release_ready_cap is None, release_ready_cap
  assert not carnival.carnival_radar_stop_hold_active

  print(
    "PASS "
    f"stop_hold_cap={stop_hold_cap:.3f} "
    f"latched_cap={latched_cap:.3f} "
    f"carnival_cap={carnival_cap:.3f} "
    f"generic_cap={generic_cap} "
    "reject_cases=5 departure_release=1"
    " release_ready_without_gas=1"
  )


if __name__ == "__main__":
  main()
