from __future__ import annotations

from dataclasses import dataclass
import math


def clip01(value: float) -> float:
  return max(0.0, min(1.0, float(value)))


def weighted_mean(values: tuple[tuple[float, float], ...], default: float = 0.0) -> float:
  finite = [(clip01(value), weight) for value, weight in values if math.isfinite(value) and weight > 0.0]
  total = sum(weight for _, weight in finite)
  return clip01(sum(value * weight for value, weight in finite) / total) if total > 0.0 else default


@dataclass(frozen=True)
class CarnivalConfidenceInput:
  active: bool = False
  lane_confidence: float = 0.0
  road_edge_confidence: float = 0.0
  path_valid: bool = False
  lead_present: bool = False
  radar_confirmed: bool = False
  radar_alive: bool = True
  lead_model_prob: float = 0.0
  steering_torque_fraction: float = 0.0
  steering_angle_deg: float = 0.0
  v_ego: float = 0.0
  steering_pressed: bool = False
  brake_pressed: bool = False
  gas_pressed: bool = False
  steer_fault_temporary: bool = False


@dataclass(frozen=True)
class CarnivalConfidenceOutput:
  overall: float
  lateral: float
  longitudinal: float
  path: float
  lane: float
  road_edge: float
  radar: float
  steering_saturation: float
  eps_risk: float
  intervention_risk: float
  recommended_speed_scale: float
  torque_scale: float
  governor_state: str
  reason: str


class CarnivalConfidenceMonitor:
  """Deterministic, bounded confidence estimator for the 4th-gen Carnival.

  This process has no actuator authority. The scale fields remain neutral for
  schema compatibility; control belongs to the platform controllers.
  """

  def __init__(self, dt: float = 0.05):
    self.dt = dt
    self._high_torque_time = 0.0
    self._intervention = 0.0
    self._overall = 1.0

  def update(self, data: CarnivalConfidenceInput) -> CarnivalConfidenceOutput:
    lane = clip01(data.lane_confidence)
    edge = clip01(data.road_edge_confidence)
    path = weighted_mean(((1.0 if data.path_valid else 0.0, 0.45), (lane, 0.35), (edge, 0.20)))

    saturation = clip01((abs(data.steering_torque_fraction) - 0.68) / 0.30)
    high_torque = saturation >= 0.65 and abs(data.steering_angle_deg) >= 3.0 and data.v_ego >= 8.0
    self._high_torque_time = min(2.0, self._high_torque_time + self.dt) if high_torque else max(0.0, self._high_torque_time - 2.0 * self.dt)
    duration_risk = clip01(self._high_torque_time / 0.8)
    angle_risk = clip01((abs(data.steering_angle_deg) - 5.0) / 35.0)
    speed_risk = clip01((data.v_ego - 8.0) / 14.0)
    eps_risk = weighted_mean(((saturation, 0.45), (duration_risk, 0.30), (angle_risk, 0.15), (speed_risk, 0.10)))
    if data.steer_fault_temporary:
      eps_risk = 1.0

    intervention_now = data.steering_pressed or data.brake_pressed or data.gas_pressed
    target_intervention = 1.0 if intervention_now else 0.0
    rate = self.dt / (0.25 if intervention_now else 2.5)
    self._intervention += clip01(rate) * (target_intervention - self._intervention)
    intervention = clip01(self._intervention)

    if not data.radar_alive:
      radar = 0.0
    elif not data.lead_present:
      radar = 1.0
    elif data.radar_confirmed:
      radar = 1.0
    else:
      radar = clip01(data.lead_model_prob) * 0.72

    lateral = weighted_mean(((path, 0.48), (1.0 - eps_risk, 0.34), (1.0 - saturation, 0.12), (1.0 - intervention, 0.06)))
    longitudinal = weighted_mean(((path, 0.30), (radar, 0.50), (1.0 - intervention, 0.20)))
    target_overall = min(lateral, longitudinal)
    alpha = clip01(self.dt / (0.25 if target_overall < self._overall else 1.0))
    self._overall += alpha * (target_overall - self._overall)
    overall = clip01(self._overall)

    if not data.active:
      state, reason = "inactive", "controls inactive"
    elif data.steer_fault_temporary:
      state, reason = "fault", "temporary EPS fault"
    elif eps_risk >= 0.70:
      state, reason = "protect", "sustained steering load"
    elif path < 0.42:
      state, reason = "caution", "weak road geometry"
    elif data.lead_present and radar < 0.55:
      state, reason = "caution", "vision lead not radar-confirmed"
    else:
      state, reason = "confident", "signals agree"

    return CarnivalConfidenceOutput(
      overall=overall, lateral=lateral, longitudinal=longitudinal, path=path,
      lane=lane, road_edge=edge, radar=radar, steering_saturation=saturation,
      eps_risk=eps_risk, intervention_risk=intervention,
      recommended_speed_scale=1.0, torque_scale=1.0,
      governor_state=state, reason=reason,
    )
