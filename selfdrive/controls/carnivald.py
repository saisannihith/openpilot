#!/usr/bin/env python3
from __future__ import annotations

import math
import numpy as np

from cereal import car, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.selfdrive.controls.lib.carnival_confidence import CarnivalConfidenceGovernor, CarnivalConfidenceInput


CARNIVAL = "KIA_CARNIVAL_4TH_GEN"


def _lane_confidence(model) -> float:
  probs = list(getattr(model, "laneLineProbs", []))
  inner = probs[1:3] if len(probs) >= 4 else probs
  return float(np.mean(inner)) if inner else 0.0


def _road_edge_confidence(model) -> float:
  stds = [float(value) for value in getattr(model, "roadEdgeStds", []) if math.isfinite(float(value))]
  return float(np.clip(1.0 - np.mean(stds) / 1.5, 0.0, 1.0)) if stds else 0.0


def _path_valid(model) -> bool:
  xs = list(getattr(getattr(model, "position", None), "x", []))
  ys = list(getattr(getattr(model, "position", None), "y", []))
  return len(xs) >= 10 and len(xs) == len(ys) and all(math.isfinite(float(v)) for v in (xs[0], xs[-1], ys[0], ys[-1]))


def _lead(radar_state):
  leads = [lead for lead in (radar_state.leadOne, radar_state.leadTwo) if bool(getattr(lead, "status", False))]
  return min(leads, key=lambda lead: float(getattr(lead, "dRel", float("inf")))) if leads else None


def _stop_state(sm, lead) -> tuple[str, bool]:
  stopped_lead = bool(lead is not None and float(getattr(lead, "vLead", 99.0)) < 0.8 and float(getattr(lead, "dRel", 999.0)) < 18.0)
  red = bool(getattr(sm["starpilotPlan"], "redLight", False))
  forcing = bool(getattr(sm["starpilotPlan"], "forcingStop", False))
  should_stop = bool(getattr(sm["longitudinalPlan"], "shouldStop", False))
  standstill = bool(getattr(sm["carState"], "standstill", False))
  if standstill and (stopped_lead or red or forcing or should_stop):
    return "hold", True
  if stopped_lead:
    return "lead-approach", False
  if red or forcing or should_stop:
    return "intersection-approach", False
  return "cruise", False


def _cut_in_candidates(live_tracks) -> int:
  count = 0
  for point in getattr(live_tracks, "points", []):
    d_rel = float(getattr(point, "dRel", 0.0))
    y_rel = float(getattr(point, "yRel", 0.0))
    v_lat = float(getattr(point, "vLat", 0.0))
    if 0.0 < d_rel < 80.0 and 1.5 < abs(y_rel) < 4.0 and y_rel * v_lat < -0.15:
      count += 1
  return min(count, 255)


def _message_valid(active_car: bool, sm) -> bool:
  # Radar health is payload data. It must not suppress the message that tells
  # the UI and route logs radar has gone stale.
  return active_car and sm.all_checks(["modelV2", "carState", "carControl"])


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  sm = messaging.SubMaster([
    "modelV2", "radarState", "carState", "carControl", "carOutput",
    "controlsState", "selfdriveState", "longitudinalPlan", "starpilotPlan", "liveTracks",
  ], poll="modelV2")
  pm = messaging.PubMaster(["carnivalState"])
  governor = CarnivalConfidenceGovernor()
  frame = 0
  features_enabled = params.get_bool("CarnivalFeaturesEnabled")
  governor_enabled = features_enabled and params.get_bool("CarnivalConfidenceGovernor")

  while True:
    sm.update()
    if not sm.updated["modelV2"]:
      continue
    frame += 1
    if frame % 20 == 1:
      features_enabled = params.get_bool("CarnivalFeaturesEnabled")
      governor_enabled = features_enabled and params.get_bool("CarnivalConfidenceGovernor")

    active_car = str(CP.carFingerprint) == CARNIVAL
    lead = _lead(sm["radarState"])
    torque = float(getattr(sm["carControl"].actuators, "torque", 0.0))
    output = governor.update(CarnivalConfidenceInput(
      active=active_car and bool(getattr(sm["carControl"], "latActive", False) or getattr(sm["carControl"], "longActive", False)),
      lane_confidence=_lane_confidence(sm["modelV2"]),
      road_edge_confidence=_road_edge_confidence(sm["modelV2"]),
      path_valid=_path_valid(sm["modelV2"]),
      lead_present=lead is not None,
      radar_confirmed=bool(lead is not None and getattr(lead, "radar", False)),
      radar_alive=bool(sm.alive.get("radarState", False) and sm.valid.get("radarState", False)),
      lead_model_prob=float(getattr(lead, "modelProb", 0.0)) if lead is not None else 0.0,
      steering_torque_fraction=abs(torque),
      steering_angle_deg=float(getattr(sm["carState"], "steeringAngleDeg", 0.0)),
      v_ego=float(getattr(sm["carState"], "vEgo", 0.0)),
      steering_pressed=bool(getattr(sm["carState"], "steeringPressed", False)),
      brake_pressed=bool(getattr(sm["carState"], "brakePressed", False)),
      gas_pressed=bool(getattr(sm["carState"], "gasPressed", False)),
      steer_fault_temporary=bool(getattr(sm["carState"], "steerFaultTemporary", False)),
    ))
    stop_state, stop_hold = _stop_state(sm, lead)
    msg = messaging.new_message("carnivalState")
    msg.valid = _message_valid(active_car, sm)
    state = msg.carnivalState
    state.active = active_car and features_enabled
    state.overallConfidence = output.overall
    state.lateralConfidence = output.lateral
    state.longitudinalConfidence = output.longitudinal
    state.pathConfidence = output.path
    state.laneConfidence = output.lane
    state.roadEdgeConfidence = output.road_edge
    state.radarConfidence = output.radar
    state.steeringSaturation = output.steering_saturation
    state.epsRisk = output.eps_risk
    state.interventionRisk = output.intervention_risk
    state.leadSource = "none" if lead is None else "radar+vision" if bool(getattr(lead, "radar", False)) else "vision"
    state.leadDistance = float(getattr(lead, "dRel", 0.0)) if lead is not None else 0.0
    state.leadRelativeSpeed = float(getattr(lead, "vRel", 0.0)) if lead is not None else 0.0
    state.radarTrackId = int(getattr(lead, "radarTrackId", -1)) if lead is not None else -1
    state.radarStale = not bool(sm.alive.get("radarState", False) and sm.valid.get("radarState", False))
    state.stopState = stop_state
    state.stopHoldActive = stop_hold
    state.governorState = output.governor_state if governor_enabled else "monitor"
    state.recommendedSpeedScale = output.recommended_speed_scale
    state.torqueScale = output.torque_scale
    state.reason = output.reason if governor_enabled else "governor disabled; logging only" if features_enabled else "Carnival enhancements disabled"
    state.visionLeadPresent = lead is not None
    state.radarLeadPresent = bool(lead is not None and getattr(lead, "radar", False))
    state.cutInCandidateCount = _cut_in_candidates(sm["liveTracks"])
    pm.send("carnivalState", msg)


if __name__ == "__main__":
  main()
