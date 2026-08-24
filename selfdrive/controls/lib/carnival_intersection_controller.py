from __future__ import annotations

from dataclasses import dataclass


CARNIVAL = "KIA_CARNIVAL_4TH_GEN"
HOLD_ENTRY_MAX_SPEED = 1.2
HOLD_EXIT_MAX_SPEED = 2.0
HOLD_BRAKE = 0.55
HOLD_CLEAR_CONFIRM_TIME = 0.8
STOPPED_LEAD_MAX_SPEED = 0.8
STOPPED_LEAD_MAX_DISTANCE = 18.0
STOPPED_LEAD_MAX_LATERAL = 1.75
LEAD_RELEASE_MIN_SPEED = 1.2
LEAD_RELEASE_MIN_DELTA = 0.8


@dataclass(frozen=True)
class CarnivalIntersectionOutput:
  state: str = "idle"
  accel_cap: float | None = None
  should_stop: bool = False
  reason: str = ""


class CarnivalIntersectionController:
  """Own the final low-speed hold/release phase after model-led stopping."""

  def __init__(self, car_fingerprint: str, dt: float):
    self.enabled = str(car_fingerprint) == CARNIVAL
    self.dt = dt
    self.state = "idle"
    self._clear_time = 0.0

  @staticmethod
  def _stopped_lead(lead) -> bool:
    return bool(
      lead is not None and getattr(lead, "status", False) and
      float(getattr(lead, "dRel", 999.0)) <= STOPPED_LEAD_MAX_DISTANCE and
      abs(float(getattr(lead, "yRel", 99.0))) <= STOPPED_LEAD_MAX_LATERAL and
      float(getattr(lead, "vLead", 99.0)) <= STOPPED_LEAD_MAX_SPEED and
      (bool(getattr(lead, "radar", False)) or float(getattr(lead, "modelProb", 0.0)) >= 0.90)
    )

  @staticmethod
  def _lead_departed(lead, v_ego: float) -> bool:
    return bool(
      lead is not None and getattr(lead, "status", False) and
      float(getattr(lead, "vLead", 0.0)) >= LEAD_RELEASE_MIN_SPEED and
      float(getattr(lead, "vLead", 0.0)) - float(v_ego) >= LEAD_RELEASE_MIN_DELTA and
      (bool(getattr(lead, "radar", False)) or float(getattr(lead, "modelProb", 0.0)) >= 0.95)
    )

  def update(self, *, v_ego: float, lead, red_light: bool, model_should_stop: bool,
             forcing_stop: bool, driver_gas: bool, feature_enabled: bool = True) -> CarnivalIntersectionOutput:
    if not self.enabled or not feature_enabled:
      self.state = "idle"
      self._clear_time = 0.0
      return CarnivalIntersectionOutput()
    if driver_gas:
      self.state = "release"
      self._clear_time = 0.0
      return CarnivalIntersectionOutput(state=self.state, reason="driver accelerator")

    stopped_lead = self._stopped_lead(lead)
    stop_evidence = bool(stopped_lead or red_light or model_should_stop or forcing_stop)
    if self.state in ("idle", "release"):
      if stop_evidence and float(v_ego) <= HOLD_ENTRY_MAX_SPEED:
        self.state = "hold"
        self._clear_time = 0.0
      elif stop_evidence:
        self.state = "approach"
      elif self.state == "release" and float(v_ego) > HOLD_EXIT_MAX_SPEED:
        self.state = "idle"
    elif self.state == "approach":
      if not stop_evidence:
        self.state = "idle"
      elif float(v_ego) <= HOLD_ENTRY_MAX_SPEED:
        self.state = "hold"
        self._clear_time = 0.0
    elif self.state == "hold":
      if self._lead_departed(lead, v_ego):
        self.state = "release"
        self._clear_time = 0.0
      elif stop_evidence:
        self._clear_time = 0.0
      else:
        self._clear_time += self.dt
        if self._clear_time + 1e-6 >= HOLD_CLEAR_CONFIRM_TIME:
          self.state = "release"
          self._clear_time = 0.0

    if self.state == "hold":
      reason = "stopped lead" if stopped_lead else "intersection stop"
      return CarnivalIntersectionOutput(state=self.state, accel_cap=-HOLD_BRAKE, should_stop=True, reason=reason)
    return CarnivalIntersectionOutput(state=self.state, reason="stop evidence" if stop_evidence else "clear")
