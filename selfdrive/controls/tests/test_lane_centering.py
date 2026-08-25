from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController


_V_EGO = 20.0
_XS = np.linspace(0.0, 50.0, 52)


class _IndexOnlyList:
  def __init__(self, values):
    self._values = values

  def __len__(self):
    return len(self._values)

  def __getitem__(self, index):
    if not isinstance(index, int):
      raise TypeError("integer index required")
    return self._values[index]


def _path(y, y_std=0.1):
  return SimpleNamespace(
    x=_XS.copy(),
    y=np.full_like(_XS, float(y)),
    yStd=np.full_like(_XS, float(y_std)),
  )


def _model(left=-1.8, right=1.8, model_y=0.0, lane_prob=0.9, lane_std=0.1, path_std=0.1, lane_change=0,
           frame_id=0, outer_left=-5.4, outer_right=5.4, outer_left_prob=0.9, outer_right_prob=0.9,
           road_edge_left=-7.0, road_edge_right=7.0, road_edge_std=0.5):
  return SimpleNamespace(
    frameId=frame_id,
    laneLines=[_path(outer_left), _path(left), _path(right), _path(outer_right)],
    laneLineProbs=[outer_left_prob, lane_prob, lane_prob, outer_right_prob],
    laneLineStds=[0.0, lane_std, lane_std, 0.0],
    roadEdges=[_path(road_edge_left), _path(road_edge_right)],
    roadEdgeStds=[road_edge_std, road_edge_std],
    position=_path(model_y, path_std),
    meta=SimpleNamespace(laneChangeState=lane_change),
  )


def _update(controller, model, *, offset=0.0, authority=1.0, enabled=True, active=True, valid=True, speed=_V_EGO,
            pause_on_signal=False, turn_signal_active=False, model_curvature=0.0, steering_pressed=False,
            road_aware=False, road_edge_offset=0.15):
  return controller.update(model_curvature, model, speed, enabled, offset, authority, active, valid,
                           pause_on_signal, turn_signal_active, steering_pressed, road_aware, road_edge_offset)


def _converge(model, *, offset=0.0, authority=1.0, model_curvature=0.0, road_aware=False, road_edge_offset=0.15):
  controller = LaneCenteringController()
  output = 0.0
  for i in range(300):
    if i % 5 == 0:
      model.frameId += 1
    output = _update(controller, model, offset=offset, authority=authority, model_curvature=model_curvature,
                     road_aware=road_aware, road_edge_offset=road_edge_offset)
  return controller, output


@pytest.mark.parametrize(
  "kwargs",
  [
    {"enabled": False},
    {"active": False},
    {"valid": False},
    {"speed": 4.9},
  ],
)
def test_hard_gates_are_noop(kwargs):
  assert _update(LaneCenteringController(), _model(left=-1.5, right=2.1), **kwargs) == 0.0


def test_lane_change_is_noop():
  assert _update(LaneCenteringController(), _model(left=-1.5, right=2.1, lane_change=1)) == 0.0


def test_turn_signal_fades_lane_centering_correction():
  model = _model(left=-1.5, right=2.1)
  controller, centered = _converge(model, authority=0.0)
  fading = _update(controller, model, authority=0.0, pause_on_signal=True, turn_signal_active=True)
  assert 0.0 < fading < centered

  for _ in range(300):
    fading = _update(controller, model, authority=0.0, pause_on_signal=True, turn_signal_active=True)
  assert abs(fading) < 1e-6


def test_turn_signal_pause_can_be_disabled():
  model = _model(left=-1.5, right=2.1)
  _, output = _converge(model, authority=0.0)
  controller, _ = _converge(model, authority=0.0)
  signaled = _update(controller, model, authority=0.0, turn_signal_active=True)
  assert signaled == pytest.approx(output, abs=1e-7)


def test_driver_steering_always_fades_correction():
  model = _model(left=-1.5, right=2.1)
  controller, centered = _converge(model, authority=0.0)
  fading = _update(controller, model, authority=0.0, steering_pressed=True)
  assert 0.0 < fading < centered

  for _ in range(300):
    fading = _update(controller, model, authority=0.0, steering_pressed=True)
  assert abs(fading) < 1e-6


@pytest.mark.parametrize(
  "field,value",
  [
    ("prob", np.nan),
    ("prob", 1.1),
    ("std", np.nan),
    ("std", -0.1),
  ],
)
def test_invalid_lane_confidence_is_rejected(field, value):
  model = _model(left=-1.5, right=2.1)
  values = model.laneLineProbs if field == "prob" else model.laneLineStds
  values[1] = value
  assert _update(LaneCenteringController(), model) == 0.0


def test_input_must_cover_lookahead():
  model = _model(left=-1.5, right=2.1)
  model.laneLines[1].x = model.laneLines[1].x[:10]
  model.laneLines[1].y = model.laneLines[1].y[:10]
  assert _update(LaneCenteringController(), model) == 0.0


def test_missing_or_untrusted_path_uncertainty_is_rejected():
  missing = _model(left=-1.5, right=2.1)
  del missing.position.yStd
  assert _update(LaneCenteringController(), missing) == 0.0

  uncertain = _model(left=-1.5, right=2.1, path_std=0.36)
  assert _update(LaneCenteringController(), uncertain) == 0.0


def test_missing_model_frame_id_is_rejected():
  model = _model(left=-1.5, right=2.1)
  del model.frameId
  assert _update(LaneCenteringController(), model, model_curvature=0.003) == 0.003


@pytest.mark.parametrize(
  "model",
  [
    _model(left=-1.5, right=2.1, lane_prob=0.69),
    _model(left=-1.5, right=2.1, lane_std=0.26),
    SimpleNamespace(
      frameId=0,
      laneLines=[],
      laneLineProbs=[],
      laneLineStds=[],
      position=_path(0.0),
      meta=SimpleNamespace(laneChangeState=0),
    ),
  ],
)
def test_unmarked_or_untrusted_road_defers_to_model(model):
  model_curvature = 0.003
  assert _update(LaneCenteringController(), model, model_curvature=model_curvature) == model_curvature


def test_lane_center_error_steers_toward_center():
  _, right = _converge(_model(left=-1.5, right=2.1), authority=0.0)
  _, left = _converge(_model(left=-2.1, right=1.5), authority=0.0)
  assert left < 0.0
  assert right > 0.0


def test_small_center_error_does_not_chatter():
  _, output = _converge(_model(left=-1.75, right=1.85), authority=0.0)
  assert output == 0.0


def test_offset_direction():
  _, right = _converge(_model(), offset=0.2, authority=0.0)
  _, left = _converge(_model(), offset=-0.2, authority=0.0)
  assert right > 0.0
  assert left < 0.0


def test_road_topology_classifies_confirmed_outer_lanes():
  controller = LaneCenteringController()
  right_outer = _model(
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  left_outer = _model(
    outer_left_prob=0.05,
    road_edge_left=-2.8,
    road_edge_right=6.0,
  )
  assert controller._classify_road_topology(right_outer, _V_EGO) == 1
  assert controller._classify_road_topology(left_outer, _V_EGO) == -1


def test_road_topology_supports_capnp_style_index_only_lists():
  controller = LaneCenteringController()
  model = _model(
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  model.laneLines = _IndexOnlyList(model.laneLines)
  model.roadEdges = _IndexOnlyList(model.roadEdges)
  assert controller._classify_road_topology(model, _V_EGO) == 1


def test_road_topology_rejects_interior_and_conflicting_geometry():
  controller = LaneCenteringController()
  assert controller._classify_road_topology(_model(), _V_EGO) == 0
  conflicting = _model(
    outer_right_prob=0.05,
    road_edge_left=-2.5,
    road_edge_right=6.0,
  )
  assert controller._classify_road_topology(conflicting, _V_EGO) == 0


def test_road_topology_rejects_close_or_inconsistent_outer_edges():
  controller = LaneCenteringController()
  close_edge = _model(
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.4,
  )
  assert controller._classify_road_topology(close_edge, _V_EGO) == 0

  inconsistent = _model(
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  inconsistent.roadEdges[1].y = np.linspace(2.8, 1.6, _XS.size)
  assert controller._classify_road_topology(inconsistent, _V_EGO) == 0


def test_road_topology_requires_sustained_evidence():
  controller = LaneCenteringController()
  for _ in range(19):
    controller._update_road_topology(1, 0.15)
  assert controller._road_topology_state == 0
  controller._update_road_topology(1, 0.15)
  assert controller._road_topology_state == 1

  for _ in range(9):
    controller._update_road_topology(0, 0.15)
  assert controller._road_topology_state == 1
  controller._update_road_topology(0, 0.15)
  assert controller._road_topology_state == 0


def test_road_aware_bias_moves_toward_confirmed_outer_edge():
  right_outer = _model(
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  left_outer = _model(
    outer_left_prob=0.05,
    road_edge_left=-2.8,
    road_edge_right=6.0,
  )
  _, right = _converge(right_outer, authority=0.0, road_aware=True)
  _, left = _converge(left_outer, authority=0.0, road_aware=True)
  assert left < 0.0 < right
  assert abs(left) == pytest.approx(abs(right), rel=1e-6)


def test_manual_offset_disables_road_aware_bias():
  model = _model(
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  _, manual_only = _converge(model, offset=0.15, authority=0.0)
  controller, road_aware = _converge(model, offset=0.15, authority=0.0, road_aware=True)
  assert controller._road_topology_state == 0
  assert controller._road_topology_reason == "manual_offset_active"
  assert road_aware == pytest.approx(manual_only, abs=1e-9)


def test_road_aware_bias_preserves_narrow_lane_clearance():
  model = _model(
    left=-1.3,
    right=1.3,
    outer_left=-4.5,
    outer_right=4.5,
    outer_right_prob=0.05,
    road_edge_left=-5.5,
    road_edge_right=1.9,
  )
  _, output = _converge(model, authority=0.0, road_aware=True, road_edge_offset=0.2)
  assert output == pytest.approx(0.0, abs=1e-9)


def test_road_aware_bias_yields_when_model_is_near_outer_boundary():
  model = _model(
    model_y=0.55,
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  _, centered = _converge(model, authority=0.0)
  controller, road_aware = _converge(model, authority=0.0, road_aware=True)
  assert controller._road_topology_state == 1
  assert controller._effective_road_topology_bias == 0.0
  assert road_aware == pytest.approx(centered, abs=1e-9)


def test_road_aware_bias_uses_only_available_outer_clearance():
  model = _model(
    model_y=0.3,
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  controller, _ = _converge(model, authority=0.0, road_aware=True, road_edge_offset=0.15)
  assert controller._road_topology_state == 1
  assert 0.0 < controller._effective_road_topology_bias <= 0.1


def test_offset_is_reduced_in_narrow_lane():
  narrow = _model(left=-1.3, right=1.3)
  _, at_safe_limit = _converge(narrow, offset=0.2, authority=0.0)
  _, above_safe_limit = _converge(narrow, offset=0.3, authority=0.0)
  assert np.isclose(at_safe_limit, above_safe_limit)


def test_large_lane_offset_keeps_envelope_authority():
  model = _model(left=-1.4, right=2.2, model_y=0.0, path_std=0.1)
  _, lane_authority = _converge(model, authority=0.0)
  _, e2e_authority = _converge(model, authority=1.0)
  assert lane_authority > 0.0
  assert abs(lane_authority) > abs(e2e_authority) > 0.0


def test_uncertain_e2e_path_disables_lane_correction():
  model = _model(left=-1.0, right=2.6, model_y=0.0, path_std=0.6)
  _, output = _converge(model, authority=1.0)
  assert output == 0.0


def test_e2e_authority_blends_lane_correction():
  model = _model(left=-1.55, right=2.05, model_y=0.0, path_std=0.1)
  _, lane_only = _converge(model, authority=0.0)
  _, blended = _converge(model, authority=0.5)
  _, e2e = _converge(model, authority=1.0)
  assert lane_only > blended > e2e >= 0.0


def test_confident_e2e_authority_starts_before_large_offset():
  model = _model(left=-1.7, right=2.1, model_y=0.0, path_std=0.1)
  _, lane_only = _converge(model, authority=0.0)
  _, e2e = _converge(model, authority=1.0)
  assert lane_only > e2e > 0.0


def test_confidence_loss_drops_filtered_correction():
  controller, output = _converge(_model(left=-1.5, right=2.1), authority=0.0)
  assert output > 0.0
  low_confidence = _model(left=-1.5, right=2.1, lane_prob=0.2, frame_id=1)
  fading = _update(controller, low_confidence, authority=0.0)
  assert 0.0 < fading < output

  for _ in range(300):
    fading = _update(controller, low_confidence, authority=0.0)
  assert abs(fading) < 1e-6


def test_lane_geometry_requires_three_model_frames_to_acquire():
  controller = LaneCenteringController()
  model = _model(left=-1.5, right=2.1)
  for frame_id in (1, 2):
    model.frameId = frame_id
    assert _update(controller, model, authority=0.0) == 0.0

  model.frameId = 3
  assert _update(controller, model, authority=0.0) > 0.0


def test_uncertain_markings_have_less_authority():
  _, strong = _converge(_model(left=-1.5, right=2.1, lane_prob=0.95, lane_std=0.1), authority=0.0)
  _, uncertain = _converge(_model(left=-1.5, right=2.1, lane_prob=0.75, lane_std=0.20), authority=0.0)
  assert 0.0 < uncertain < strong


def test_diverging_lane_boundaries_are_rejected():
  model = _model()
  widths = 3.0 + 0.08 * np.minimum(_XS, 20.0)
  center = np.full_like(_XS, 0.3)
  model.laneLines[1].y = center - widths * 0.5
  model.laneLines[2].y = center + widths * 0.5
  _, output = _converge(model, authority=0.0)
  assert output == 0.0


def test_conflicting_multi_horizon_error_has_less_authority():
  coherent_model = _model(model_y=0.3)
  conflicting_model = _model()
  conflicting_model.position.y = np.where(_XS < 14.5, -0.3, 0.3)
  _, coherent = _converge(coherent_model, authority=0.0)
  _, conflicting = _converge(conflicting_model, authority=0.0)
  assert abs(conflicting) < abs(coherent)


def test_driver_release_resumes_with_smoothed_correction():
  model = _model(left=-1.5, right=2.1)
  controller, output = _converge(model, authority=0.0)
  assert output > 0.0
  for _ in range(300):
    output = _update(controller, model, authority=0.0, steering_pressed=True)
  assert abs(output) < 1e-6

  for _ in range(2):
    model.frameId += 1
    assert _update(controller, model, authority=0.0) == pytest.approx(0.0, abs=1e-9)
  model.frameId += 1
  assert _update(controller, model, authority=0.0) > 0.0


def test_driver_override_clears_road_topology_and_requires_reacquisition():
  model = _model(
    outer_right_prob=0.05,
    road_edge_left=-6.0,
    road_edge_right=2.8,
  )
  controller, output = _converge(model, authority=0.0, road_aware=True)
  assert output > 0.0
  assert controller._road_topology_state == 1

  _update(controller, model, authority=0.0, road_aware=True, steering_pressed=True)
  assert controller._road_topology_state == 0
  assert controller._valid_model_frames == 0

  for _ in range(19):
    model.frameId += 1
    _update(controller, model, authority=0.0, road_aware=True)
  assert controller._road_topology_state == 0
  model.frameId += 1
  _update(controller, model, authority=0.0, road_aware=True)
  assert controller._road_topology_state == 1


def test_geometry_target_refreshes_once_per_model_frame():
  controller = LaneCenteringController()
  model = _model(left=-1.5, right=2.1, frame_id=10)
  for frame_id in (10, 11, 12):
    model.frameId = frame_id
    active = _update(controller, model, authority=0.0)
  assert active > 0.0

  model.laneLines[1].y = np.full_like(_XS, -1.8)
  model.laneLines[2].y = np.full_like(_XS, 1.8)
  cached = _update(controller, model, authority=0.0)
  assert active < cached

  model.frameId += 1
  refreshed = _update(controller, model, authority=0.0)
  assert 0.0 < refreshed < cached


def test_correction_is_smoothed_and_capped():
  controller = LaneCenteringController()
  model = _model(left=0.0, right=3.0, path_std=0.1)
  first = _update(controller, model, authority=0.0)
  _, steady = _converge(model, authority=0.0)
  assert first == 0.0
  assert steady > 0.0
  assert np.isclose(abs(steady), 0.6 / _V_EGO ** 2, atol=2e-6)


def test_multi_horizon_fit_rejects_near_point_bias():
  model = _model()
  model.position.y = np.zeros_like(_XS)
  model.laneLines[1].y = np.where(_XS < 7.0, -1.2, -1.8)
  model.laneLines[2].y = np.where(_XS < 7.0, 2.4, 1.8)
  _, output = _converge(model, authority=0.0)
  assert abs(output) < 1e-9


def test_curved_lane_geometry_corrects_lane_relative_offset():
  model = _model()
  curve_y = 0.001 * _XS ** 2
  model.laneLines[1].y = curve_y - 1.8
  model.laneLines[2].y = curve_y + 1.8
  model.position.y = curve_y + 0.3
  _, output = _converge(model, authority=0.0, model_curvature=0.002)
  assert output < 0.002


def test_curve_restores_more_envelope_authority():
  model = _model(left=-1.4, right=2.2, model_y=0.0, path_std=0.1)
  _, straight = _converge(model, authority=1.0)
  _, curve = _converge(model, authority=1.0, model_curvature=0.002)
  assert abs(curve - 0.002) > abs(straight) > 0.0


def test_correction_lateral_acceleration_is_speed_bounded():
  model = _model(left=0.0, right=3.0, path_std=0.1)
  _, at_20 = _converge(model, authority=0.0)
  controller = LaneCenteringController()
  at_30 = 0.0
  for i in range(300):
    if i % 5 == 0:
      model.frameId += 1
    at_30 = _update(controller, model, authority=0.0, speed=30.0)
  assert 0.599 < abs(at_20) * 20.0 ** 2 <= 0.6
  assert 0.599 < abs(at_30) * 30.0 ** 2 <= 0.6


def test_speed_change_immediately_reapplies_lateral_acceleration_limit():
  model = _model(left=0.0, right=3.0, path_std=0.1)
  controller, at_20 = _converge(model, authority=0.0)
  assert 0.599 < abs(at_20) * 20.0 ** 2 <= 0.6

  at_35 = _update(controller, model, authority=0.0, speed=35.0)
  assert abs(at_35) * 35.0 ** 2 <= 0.6

  model.frameId += 1
  model.laneLineProbs[1] = model.laneLineProbs[2] = 0.1
  fading = _update(controller, model, authority=0.0, speed=40.0)
  assert abs(fading) * 40.0 ** 2 <= 0.6


def test_hostile_random_transitions_remain_finite_and_bounded():
  rng = np.random.default_rng(20260825)
  controller = LaneCenteringController()
  model = _model()

  for frame_id in range(5000):
    speed = float(rng.uniform(5.0, 40.0))
    model_curvature = float(rng.uniform(-0.02, 0.02))
    lane_width = float(rng.uniform(2.4, 5.0))
    lane_center = float(rng.uniform(-0.8, 0.8))
    model.frameId = frame_id
    model.laneLines[1].y.fill(lane_center - lane_width * 0.5)
    model.laneLines[2].y.fill(lane_center + lane_width * 0.5)
    model.position.y.fill(float(rng.uniform(-0.8, 0.8)))
    model.position.yStd.fill(float(rng.uniform(0.0, 0.5)))
    model.laneLineProbs[1] = float(rng.uniform(0.0, 1.0))
    model.laneLineProbs[2] = float(rng.uniform(0.0, 1.0))
    model.laneLineStds[1] = float(rng.uniform(0.0, 0.5))
    model.laneLineStds[2] = float(rng.uniform(0.0, 0.5))
    if frame_id % 127 == 0:
      model.position.yStd[10] = np.nan

    output = _update(
      controller,
      model,
      offset=float(rng.uniform(-0.5, 0.5)),
      authority=float(rng.uniform(-1.0, 2.0)),
      speed=speed,
      model_curvature=model_curvature,
      steering_pressed=bool(rng.integers(0, 20) == 0),
      road_aware=True,
      road_edge_offset=float(rng.uniform(-0.5, 0.5)),
    )
    assert np.isfinite(output)
    assert abs(output - model_curvature) * speed ** 2 <= 0.600001


def test_diagnostic_events_are_consumed_once():
  controller = LaneCenteringController()
  model = _model(left=-1.5, right=2.1)
  for frame_id in range(3):
    model.frameId = frame_id
    _update(controller, model, authority=0.0)
  event = controller.consume_diagnostic_event()
  assert event is not None
  assert event[0] is True
  assert event[3] == "active"
  assert controller.consume_diagnostic_event() is None
