from openpilot.selfdrive.controls.lib.carnival_confidence import (
  CarnivalConfidenceInput,
  CarnivalConfidenceMonitor,
)
from openpilot.selfdrive.controls.carnivald import _message_valid


class _FakeSubMaster:
  def __init__(self, valid_services: set[str]):
    self.valid_services = valid_services
    self.checked: list[str] = []

  def all_checks(self, services: list[str]) -> bool:
    self.checked = services
    return all(service in self.valid_services for service in services)


def test_confidence_monitor_confident_signals_stay_unrestricted():
  monitor = CarnivalConfidenceMonitor()
  output = monitor.update(CarnivalConfidenceInput(
    active=True, lane_confidence=0.9, road_edge_confidence=0.9, path_valid=True,
    lead_present=True, radar_confirmed=True, lead_model_prob=0.99,
    steering_torque_fraction=0.35, steering_angle_deg=3.0, v_ego=20.0,
  ))
  assert output.overall > 0.75
  assert output.torque_scale == 1.0
  assert output.recommended_speed_scale == 1.0
  assert output.governor_state == "confident"


def test_confidence_monitor_reports_sustained_eps_risk_without_control_authority():
  monitor = CarnivalConfidenceMonitor()
  output = None
  for _ in range(24):
    output = monitor.update(CarnivalConfidenceInput(
      active=True, lane_confidence=0.9, road_edge_confidence=0.9, path_valid=True,
      steering_torque_fraction=0.97, steering_angle_deg=24.0, v_ego=24.0,
    ))
  assert output is not None
  assert output.eps_risk > 0.65
  assert output.torque_scale == 1.0
  assert output.recommended_speed_scale == 1.0
  assert output.governor_state == "protect"


def test_confidence_monitor_does_not_penalize_no_lead_cruise():
  monitor = CarnivalConfidenceMonitor()
  output = monitor.update(CarnivalConfidenceInput(
    active=True, lane_confidence=0.85, road_edge_confidence=0.85, path_valid=True,
    lead_present=False, radar_alive=True,
  ))
  assert output.radar == 1.0
  assert output.longitudinal > 0.8


def test_carnival_state_stays_valid_when_only_radar_is_stale():
  sm = _FakeSubMaster({"modelV2", "carState", "carControl"})
  assert _message_valid(True, sm)
  assert "radarState" not in sm.checked
