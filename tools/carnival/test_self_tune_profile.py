from openpilot.tools.carnival.self_tune_profile import (
  apply_resolved_plan,
  build_delta_plan,
  resolve_values,
  revert_snapshot,
)


class FakeParams:
  def __init__(self):
    self.values = {}

  def put(self, key, value):
    self.values[key] = str(value)


def test_profile_is_empty_without_route_evidence():
  assert build_delta_plan({"scorecard": {}, "longitudinal": {}}) == {}


def test_profile_uses_tiny_allowlisted_deltas_for_missed_stop():
  plan = build_delta_plan({
    "scorecard": {"missed_stop_events": 1},
    "longitudinal": {"min_ttc": 1.8},
  })
  assert plan["StandardFollow"] == 0.05
  assert plan["TrafficFollow"] == 0.05
  assert plan["ForceStopDistanceOffset"] == 1.0


def test_profile_uses_follow_only_for_short_ttc():
  plan = build_delta_plan({
    "scorecard": {"missed_stop_events": 0},
    "longitudinal": {"min_ttc": 1.5},
  })
  assert plan["StandardFollow"] == 0.05
  assert "ForceStopDistanceOffset" not in plan


def test_profile_resolution_clips_and_ignores_unknown_keys():
  resolved = resolve_values(
    {"TrafficFollow": 2.49, "ForceStopDistanceOffset": 20.0, "UnsafeTorque": 1.0},
    {"TrafficFollow": 0.05, "ForceStopDistanceOffset": 1.0, "UnsafeTorque": 10.0},
  )
  assert resolved["TrafficFollow"]["after"] == 2.5
  assert "ForceStopDistanceOffset" not in resolved
  assert "UnsafeTorque" not in resolved


def test_profile_apply_and_revert_are_allowlisted_transactions():
  params = FakeParams()
  resolved = {
    "StandardFollow": {"before": 1.45, "after": 1.5},
    "UnsafeTorque": {"before": 1, "after": 99},
  }
  snapshot = apply_resolved_plan(params, resolved)
  assert params.values == {"StandardFollow": "1.5"}
  assert snapshot["values"] == {"StandardFollow": 1.45}
  assert revert_snapshot(params, snapshot)
  assert params.values["StandardFollow"] == "1.45"
