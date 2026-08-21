#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import numpy as np


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
  from cereal import car, log
  from opendbc.car.hyundai.interface import CarInterface as HyundaiCarInterface
  from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.lib.longitudinal_planner import (
    CARNIVAL_LEAD_DEPART_ACCEL_HOLD_MIN_ACCEL,
    CARNIVAL_LEAD_DEPART_MIN_ACCEL,
    LongitudinalPlanner,
  )
  from openpilot.selfdrive.modeld.constants import ModelConstants
except ModuleNotFoundError:
  install_openpilot_namespace()
  from cereal import car, log
  from opendbc.car.hyundai.interface import CarInterface as HyundaiCarInterface
  from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.lib.longitudinal_planner import (
    CARNIVAL_LEAD_DEPART_ACCEL_HOLD_MIN_ACCEL,
    CARNIVAL_LEAD_DEPART_MIN_ACCEL,
    LongitudinalPlanner,
  )
  from openpilot.selfdrive.modeld.constants import ModelConstants


def make_lead(*, status: bool, d_rel: float, v_lead: float, a_lead: float, model_prob: float = 1.0):
  lead = log.RadarState.LeadData.new_message()
  lead.status = status
  lead.dRel = d_rel
  lead.vLead = v_lead
  lead.vLeadK = v_lead
  lead.vRel = v_lead
  lead.aLeadK = a_lead
  lead.yRel = 0.0
  lead.modelProb = model_prob
  lead.radar = False
  return lead


def make_model(v_ego: float, desired_accel: float):
  model = log.ModelDataV2.new_message()
  model.init("leadsV3", 3)
  t_idxs = ModelConstants.T_IDXS
  model.position.x = [float(v_ego * t) for t in t_idxs]
  model.position.y = [0.0] * len(t_idxs)
  model.position.z = [0.0] * len(t_idxs)
  model.position.t = [float(t) for t in t_idxs]
  model.velocity.x = [float(v_ego)] * len(t_idxs)
  model.velocity.y = [0.0] * len(t_idxs)
  model.velocity.z = [0.0] * len(t_idxs)
  model.velocity.t = [float(t) for t in t_idxs]
  model.acceleration.x = [0.0] * len(t_idxs)
  model.acceleration.y = [0.0] * len(t_idxs)
  model.acceleration.z = [0.0] * len(t_idxs)
  model.acceleration.t = [float(t) for t in t_idxs]
  model.meta.disengagePredictions.gasPressProbs = [1.0] * 6
  model.meta.disengagePredictions.brakePressProbs = [0.0] * 6
  model.action.desiredAcceleration = desired_accel
  model.action.shouldStop = False
  return model


def make_sm(frame: dict, long_control_state):
  v_ego = float(frame["v_ego"])
  lead = make_lead(
    status=True,
    d_rel=float(frame["d_rel"]),
    v_lead=float(frame["v_lead"]),
    a_lead=float(frame["a_lead"]),
  )
  return {
    "carControl": SimpleNamespace(orientationNED=[0.0, 0.0, 0.0]),
    "carState": SimpleNamespace(
      vEgo=v_ego,
      vEgoCluster=v_ego,
      aEgo=float(frame.get("a_ego", 0.0)),
      vCruise=100.0,
      standstill=bool(frame["standstill"]),
      steeringAngleDeg=0.0,
      brakePressed=False,
      gasPressed=False,
      cruiseState=SimpleNamespace(standstill=False),
    ),
    "controlsState": SimpleNamespace(
      longControlState=long_control_state,
      forceDecel=False,
    ),
    "liveParameters": SimpleNamespace(angleOffsetDeg=0.0),
    "modelV2": make_model(v_ego, float(frame["model_accel"])),
    "radarState": SimpleNamespace(
      leadOne=lead,
      leadTwo=make_lead(status=False, d_rel=200.0, v_lead=0.0, a_lead=0.0, model_prob=0.0),
    ),
    "selfdriveState": SimpleNamespace(enabled=True, experimentalMode=True, personality=0),
    "starpilotCarState": SimpleNamespace(accelPressed=False),
    "starpilotPlan": SimpleNamespace(
      vCruise=10.0,
      minAcceleration=-0.5,
      maxAcceleration=2.0,
      disableThrottle=False,
      trackingLead=True,
      accelerationJerk=5.0,
      dangerJerk=5.0,
      speedJerk=5.0,
      dangerFactor=1.0,
      tFollow=1.45,
      forcingStop=False,
      redLight=False,
      forcingStopLength=2.0,
    ),
  }


def make_planner_toggles():
  return SimpleNamespace(
    taco_tune=False,
    classic_model=False,
    tinygrad_model=True,
    model_version="v15",
    vEgoStopping=0.5,
    radar_takeoffs=False,
  )


def make_longcontrol_toggles(CP):
  return SimpleNamespace(
    custom_accel_profile=False,
    startAccel=float(CP.startAccel),
    stopAccel=float(CP.stopAccel),
    stoppingDecelRate=float(CP.stoppingDecelRate),
    vEgoStarting=float(CP.vEgoStarting),
    vEgoStopping=float(CP.vEgoStopping),
  )


def main() -> int:
  CP = HyundaiCarInterface.get_non_essential_params(HYUNDAI_CAR.KIA_CARNIVAL_4TH_GEN)
  planner = LongitudinalPlanner(CP, init_v=0.0)
  long_control = LongControl(CP)
  long_control.long_control_state = LongCtrlState.stopping
  planner_toggles = make_planner_toggles()
  long_toggles = make_longcontrol_toggles(CP)

  # Route 00000013--69e0a3742d segment 15. The stale log released at 47.82s,
  # then dipped to a weak planner target during the first PID handoff.
  frames = [
    {"t": 45.32, "v_ego": 0.00, "standstill": True, "d_rel": 4.15, "v_lead": -0.01, "a_lead": 0.00, "model_accel": -0.23},
    {"t": 45.82, "v_ego": 0.00, "standstill": True, "d_rel": 4.15, "v_lead": -0.02, "a_lead": -0.01, "model_accel": -0.23},
    {"t": 46.31, "v_ego": 0.00, "standstill": True, "d_rel": 3.86, "v_lead": -0.03, "a_lead": 0.03, "model_accel": -0.24},
    {"t": 46.82, "v_ego": 0.00, "standstill": True, "d_rel": 3.76, "v_lead": 0.11, "a_lead": 0.43, "model_accel": -0.23},
    {"t": 47.32, "v_ego": 0.00, "standstill": True, "d_rel": 3.86, "v_lead": 0.25, "a_lead": 0.82, "model_accel": -0.23},
    {"t": 47.82, "v_ego": 0.00, "standstill": True, "d_rel": 4.408, "v_lead": 1.081, "a_lead": 1.52, "model_accel": 0.436},
    {"t": 48.33, "v_ego": 0.17, "standstill": False, "d_rel": 5.59, "v_lead": 1.73, "a_lead": 2.03, "model_accel": 0.24},
    {"t": 48.82, "v_ego": 0.52, "standstill": False, "d_rel": 7.05, "v_lead": 3.43, "a_lead": 0.02, "model_accel": 1.50},
  ]

  rows = []
  last_output = 0.0
  for frame in frames:
    sm = make_sm(frame, long_control.long_control_state)
    sm["carState"].aEgo = float(last_output)
    planner.update(sm, planner_toggles)

    CS = car.CarState.new_message(vEgo=float(frame["v_ego"]), aEgo=float(last_output), brakePressed=False)
    CS.cruiseState.standstill = False
    output = long_control.update(
      active=True,
      CS=CS,
      a_target=planner.output_a_target,
      should_stop=planner.output_should_stop,
      accel_limits=(-3.0, 2.0),
      starpilot_toggles=long_toggles,
      has_lead=True,
    )
    last_output = output
    rows.append({
      "t": frame["t"],
      "vEgo": round(float(frame["v_ego"]), 3),
      "plannerAccel": round(float(planner.output_a_target), 3),
      "shouldStop": bool(planner.output_should_stop),
      "longControlState": int(long_control.long_control_state),
      "commandAccel": round(float(output), 3),
    })

  release = next(row for row in rows if row["t"] == 47.82)
  handoff = next(row for row in rows if row["t"] == 48.33)
  assert release["plannerAccel"] >= CARNIVAL_LEAD_DEPART_MIN_ACCEL, rows
  assert handoff["plannerAccel"] >= CARNIVAL_LEAD_DEPART_ACCEL_HOLD_MIN_ACCEL, rows
  assert handoff["commandAccel"] >= 0.25, rows

  payload = {
    "status": "pass",
    "minExpectedPlannerFloor": CARNIVAL_LEAD_DEPART_ACCEL_HOLD_MIN_ACCEL,
    "rows": rows,
  }
  print(json.dumps(payload, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
