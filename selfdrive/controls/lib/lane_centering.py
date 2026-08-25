from cereal import log
import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value


_MIN_V_EGO = 5.0
_MIN_LANE_PROB = 0.7
_FULL_LANE_PROB = 0.80
_MAX_LANE_STD = 0.25
_FULL_LANE_STD = 0.20
_MIN_LANE_WIDTH = 2.6
_MAX_LANE_WIDTH = 4.8
_FULL_LANE_WIDTH_SPREAD = 0.15
_MAX_LANE_WIDTH_SPREAD = 0.45
_MAX_OFFSET = 0.3
_MIN_CENTER_TO_LINE = 1.4
_LOOKAHEAD_MIN = 10.0
_LOOKAHEAD_MAX = 35.0
_LOOKAHEAD_NEAR_MIN = 6.0
_LOOKAHEAD_NEAR_FRAC = 0.45
_FIT_POINT_COUNT = 5
_MAX_CORRECTION = 0.002
_MAX_CORRECTION_LAT_ACCEL = 0.6
_CENTER_GAIN = 0.75
_SMOOTH_TAU = 0.4
_SIGNAL_RELEASE_TAU = 0.20
_DRIVER_RELEASE_TAU = 0.15
_CONFIDENCE_RELEASE_TAU = 0.20
_CENTER_ERROR_DEADBAND = 0.08
_ACQUIRE_MODEL_FRAMES = 3
_MIN_CORRECTION_COHERENCE = 0.4
_FULL_CORRECTION_COHERENCE = 0.8
_ROAD_EDGE_MAX_STD = 1.0
_OUTER_LINE_ABSENT_MAX = 0.35
_ADJACENT_LINE_PRESENT_MIN = 0.55
_ADJACENT_LANE_WIDTH_MIN = 2.4
_ADJACENT_LANE_WIDTH_MAX = 5.0
_ROAD_EDGE_ASYMMETRY_MIN = 2.0
_ROAD_EDGE_CLEARANCE_MIN = 0.75
_ROAD_EDGE_CLEARANCE_MAX = 3.5
_ROAD_TOPOLOGY_LOOKAHEAD_MIN = 15.0
_ROAD_TOPOLOGY_LOOKAHEAD_MAX = 30.0
_ROAD_TOPOLOGY_ACQUIRE_FRAMES = 20
_ROAD_TOPOLOGY_RELEASE_FRAMES = 10
_ROAD_TOPOLOGY_SMOOTH_TAU = 2.0
_MAX_ROAD_EDGE_OFFSET = 0.2
_MANUAL_OFFSET_EPSILON = 0.005

_E2E_MAX_PATH_STD = 0.35
_E2E_BREAK_IN_START = 0.15
_E2E_BREAK_IN_FULL = 0.50

# A confident model may choose a human-like apex near a lane boundary. Preserve that
# preference while it remains modest, but restore lane authority as the planned path
# approaches the edge of the lane. This keeps the feature model-led in normal driving
# without allowing E2E authority to suppress a large, confidently measured offset.
_ENVELOPE_START = 0.12
_ENVELOPE_FULL = 0.30
_ENVELOPE_MIN_SCALE = 0.65
_CURVE_LAT_ACCEL_START = 0.20
_CURVE_LAT_ACCEL_FULL = 0.80


class LaneCenteringController:
  def __init__(self) -> None:
    self._correction = 0.0
    self._last_model_frame_id = None
    self._raw_valid = False
    self._raw_correction = 0.0
    self._valid_model_frames = 0
    self._road_topology_candidate = 0
    self._road_topology_candidate_frames = 0
    self._road_topology_state = 0
    self._road_topology_bias = 0.0
    self._effective_road_topology_bias = 0.0
    self._road_topology_offset_limit = 0.0
    self._road_topology_reason = "initializing"
    self._lane_centering_reason = "initializing"
    self._driver_override_active = False
    self._diagnostic_raw_valid = False
    self._diagnostic_event = None

  def reset(self) -> None:
    self._correction = 0.0
    self._last_model_frame_id = None
    self._raw_valid = False
    self._raw_correction = 0.0
    self._valid_model_frames = 0
    self._driver_override_active = False
    self._reset_road_topology()
    self._set_lane_state(False, "inactive")

  def _release_lane_lock(self) -> None:
    self._last_model_frame_id = None
    self._raw_valid = False
    self._raw_correction = 0.0
    self._valid_model_frames = 0
    self._reset_road_topology()

  def _reset_road_topology(self) -> None:
    self._road_topology_candidate = 0
    self._road_topology_candidate_frames = 0
    self._road_topology_state = 0
    self._road_topology_bias = 0.0
    self._effective_road_topology_bias = 0.0
    self._road_topology_offset_limit = 0.0

  def _set_lane_state(self, active: bool, reason: str) -> None:
    previous_active = self._diagnostic_raw_valid
    previous_reason = self._lane_centering_reason
    self._lane_centering_reason = reason
    if active != previous_active or reason != previous_reason:
      self._diagnostic_event = (active, self._road_topology_state, self._effective_road_topology_bias,
                                reason, self._road_topology_reason)
    self._diagnostic_raw_valid = active

  def consume_diagnostic_event(self):
    event = self._diagnostic_event
    self._diagnostic_event = None
    return event

  def _apply_correction_limit(self, v_ego: float) -> None:
    max_correction = min(_MAX_CORRECTION, _MAX_CORRECTION_LAT_ACCEL / max(v_ego ** 2, 1.0))
    self._correction = float(np.clip(self._correction, -max_correction, max_correction))

  def update(self, model_curvature, model_v2, v_ego, enabled, offset, e2e_authority, lat_active, model_valid,
             pause_on_signal=False, turn_signal_active=False, steering_pressed=False,
             road_aware=False, road_edge_offset=0.15) -> float:
    model_curvature = float(model_curvature)

    try:
      v_ego = float(v_ego)
      offset = float(offset)
      e2e_authority = float(e2e_authority)
      road_edge_offset = float(road_edge_offset)
    except (TypeError, ValueError):
      self.reset()
      return model_curvature

    if not np.isfinite([v_ego, model_curvature, offset, e2e_authority, road_edge_offset]).all():
      self.reset()
      return model_curvature

    if not model_valid or not enabled or not lat_active or v_ego < _MIN_V_EGO:
      self.reset()
      return model_curvature

    if steering_pressed:
      if not self._driver_override_active:
        self._release_lane_lock()
        self._driver_override_active = True
        self._set_lane_state(False, "driver_override")
      self._correction = float(smooth_value(0.0, self._correction, _DRIVER_RELEASE_TAU, dt=DT_CTRL))
      self._apply_correction_limit(v_ego)
      return model_curvature + self._correction
    self._driver_override_active = False

    if pause_on_signal and turn_signal_active:
      self._release_lane_lock()
      self._set_lane_state(False, "turn_signal_pause")
      self._correction = float(smooth_value(0.0, self._correction, _SIGNAL_RELEASE_TAU, dt=DT_CTRL))
      self._apply_correction_limit(v_ego)
      return model_curvature + self._correction

    try:
      if model_v2.meta.laneChangeState != log.LaneChangeState.off:
        self.reset()
        return model_curvature
    except (AttributeError, TypeError, ValueError):
      self.reset()
      return model_curvature

    model_frame_id = getattr(model_v2, "frameId", None)
    if model_frame_id is None:
      self._release_lane_lock()
      self._set_lane_state(False, "missing_model_frame_id")
      self._correction = float(smooth_value(0.0, self._correction, _CONFIDENCE_RELEASE_TAU, dt=DT_CTRL))
      self._apply_correction_limit(v_ego)
      return model_curvature + self._correction

    try:
      model_frame_id = int(model_frame_id)
    except (TypeError, ValueError, OverflowError):
      self._release_lane_lock()
      self._set_lane_state(False, "invalid_model_frame_id")
      return model_curvature

    if self._last_model_frame_id is not None and model_frame_id < self._last_model_frame_id:
      self._release_lane_lock()

    if model_frame_id != self._last_model_frame_id:
      manual_offset_active = abs(offset) > _MANUAL_OFFSET_EPSILON
      topology_candidate = 0
      topology_offset = 0.0
      if road_aware and manual_offset_active:
        self._road_topology_reason = "manual_offset_active"
      elif road_aware:
        topology_candidate = self._classify_road_topology(model_v2, v_ego)
        topology_offset = min(
          float(np.clip(abs(road_edge_offset), 0.0, _MAX_ROAD_EDGE_OFFSET)),
          self._road_topology_offset_limit,
        )
      else:
        self._road_topology_reason = "disabled"
      self._update_road_topology(
        topology_candidate,
        topology_offset,
      )
      calculated_valid, calculated_correction = self._calculate_raw_correction(
        model_v2,
        v_ego,
        model_curvature,
        float(np.clip(offset, -_MAX_OFFSET, _MAX_OFFSET)),
        float(np.clip(e2e_authority, 0.0, 1.0)),
        self._road_topology_bias,
      )
      self._valid_model_frames = min(self._valid_model_frames + 1, _ACQUIRE_MODEL_FRAMES) if calculated_valid else 0
      self._raw_valid = calculated_valid and self._valid_model_frames >= _ACQUIRE_MODEL_FRAMES
      self._raw_correction = calculated_correction if calculated_valid else 0.0
      self._last_model_frame_id = model_frame_id
      state_reason = self._lane_centering_reason if not calculated_valid else ("active" if self._raw_valid else "acquiring")
      self._set_lane_state(self._raw_valid, state_reason)

    valid = self._raw_valid
    raw_correction = self._raw_correction
    if not valid:
      self._correction = float(smooth_value(0.0, self._correction, _CONFIDENCE_RELEASE_TAU, dt=DT_CTRL))
      self._apply_correction_limit(v_ego)
      return model_curvature + self._correction

    max_correction = min(_MAX_CORRECTION, _MAX_CORRECTION_LAT_ACCEL / max(v_ego ** 2, 1.0))
    target = float(np.clip(raw_correction * _CENTER_GAIN, -max_correction, max_correction))
    self._correction = float(smooth_value(target, self._correction, _SMOOTH_TAU, dt=DT_CTRL))
    self._apply_correction_limit(v_ego)
    return model_curvature + self._correction

  @staticmethod
  def _valid_path(x, y) -> bool:
    return x.size >= 2 and x.size == y.size and np.isfinite(x).all() and np.isfinite(y).all() and np.all(np.diff(x) > 0)

  @staticmethod
  def _covers(x, distance: float) -> bool:
    return bool(x[0] <= distance <= x[-1])

  def _update_road_topology(self, candidate: int, offset: float) -> None:
    previous_state = self._road_topology_state
    if candidate == self._road_topology_candidate:
      self._road_topology_candidate_frames += 1
    else:
      self._road_topology_candidate = candidate
      self._road_topology_candidate_frames = 1

    required_frames = _ROAD_TOPOLOGY_ACQUIRE_FRAMES if candidate else _ROAD_TOPOLOGY_RELEASE_FRAMES
    if self._road_topology_candidate_frames >= required_frames:
      self._road_topology_state = candidate

    target = float(self._road_topology_state) * offset
    self._road_topology_bias = float(smooth_value(
      target,
      self._road_topology_bias,
      _ROAD_TOPOLOGY_SMOOTH_TAU,
      dt=5.0 * DT_CTRL,
    ))
    if self._road_topology_state != previous_state:
      self._diagnostic_event = (self._raw_valid, self._road_topology_state, self._effective_road_topology_bias,
                                self._lane_centering_reason, self._road_topology_reason)

  def _classify_road_topology(self, model_v2, v_ego: float) -> int:
    self._road_topology_offset_limit = 0.0
    try:
      lane_lines = model_v2.laneLines
      road_edges = model_v2.roadEdges
      probs = np.asarray(model_v2.laneLineProbs, dtype=float)
      lane_stds = np.asarray(model_v2.laneLineStds, dtype=float)
      edge_stds = np.asarray(model_v2.roadEdgeStds, dtype=float)
      if len(lane_lines) < 4 or len(road_edges) < 2 or probs.size < 4 or lane_stds.size < 3 or edge_stds.size < 2:
        self._road_topology_reason = "missing_geometry"
        return 0
      if not np.isfinite(probs[:4]).all() or not np.isfinite(lane_stds[[1, 2]]).all() or not np.isfinite(edge_stds[:2]).all():
        self._road_topology_reason = "invalid_uncertainty"
        return 0
      if np.any(probs[:4] < 0.0) or np.any(probs[:4] > 1.0):
        self._road_topology_reason = "invalid_probability"
        return 0
      if np.min(probs[[1, 2]]) < _MIN_LANE_PROB or np.max(lane_stds[[1, 2]]) > _MAX_LANE_STD:
        self._road_topology_reason = "untrusted_inner_lanes"
        return 0
      if np.any(edge_stds[:2] < 0.0) or np.max(edge_stds[:2]) > _ROAD_EDGE_MAX_STD:
        self._road_topology_reason = "untrusted_road_edges"
        return 0

      paths = []
      selected_paths = [lane_lines[i] for i in range(4)] + [road_edges[i] for i in range(2)]
      for path in selected_paths:
        x = np.asarray(path.x, dtype=float)
        y = np.asarray(path.y, dtype=float)
        if not self._valid_path(x, y):
          self._road_topology_reason = "invalid_geometry"
          return 0
        paths.append((x, y))

      lookahead_far = float(np.clip(v_ego, _ROAD_TOPOLOGY_LOOKAHEAD_MIN, _ROAD_TOPOLOGY_LOOKAHEAD_MAX))
      lookaheads = np.linspace(max(10.0, lookahead_far * 0.5), lookahead_far, 4)
      if not all(self._covers(x, lookahead_far) for x, _ in paths):
        self._road_topology_reason = "short_geometry"
        return 0
      line_y = np.asarray([np.interp(lookaheads, x, y) for x, y in paths[:4]])
      edge_y = np.asarray([np.interp(lookaheads, x, y) for x, y in paths[4:]])
      lane_width = line_y[2] - line_y[1]
      left_adjacent_width = line_y[1] - line_y[0]
      right_adjacent_width = line_y[3] - line_y[2]
      left_clearance = line_y[1] - edge_y[0]
      right_clearance = edge_y[1] - line_y[2]
      if (np.any(lane_width < _MIN_LANE_WIDTH) or np.any(lane_width > _MAX_LANE_WIDTH)
          or np.ptp(lane_width) > _MAX_LANE_WIDTH_SPREAD
          or np.any(left_clearance < -0.3) or np.any(right_clearance < -0.3)):
        self._road_topology_reason = "inconsistent_geometry"
        return 0

      left_adjacent = (
        probs[0] >= _ADJACENT_LINE_PRESENT_MIN
        and np.all(left_adjacent_width >= _ADJACENT_LANE_WIDTH_MIN)
        and np.all(left_adjacent_width <= _ADJACENT_LANE_WIDTH_MAX)
        and np.ptp(left_adjacent_width) <= _MAX_LANE_WIDTH_SPREAD
      )
      right_adjacent = (
        probs[3] >= _ADJACENT_LINE_PRESENT_MIN
        and np.all(right_adjacent_width >= _ADJACENT_LANE_WIDTH_MIN)
        and np.all(right_adjacent_width <= _ADJACENT_LANE_WIDTH_MAX)
        and np.ptp(right_adjacent_width) <= _MAX_LANE_WIDTH_SPREAD
      )
      clearance_delta = left_clearance - right_clearance
      if (
        probs[3] <= _OUTER_LINE_ABSENT_MAX
        and left_adjacent
        and np.all(clearance_delta >= _ROAD_EDGE_ASYMMETRY_MIN)
        and np.all(right_clearance >= _ROAD_EDGE_CLEARANCE_MIN)
        and np.all(right_clearance <= _ROAD_EDGE_CLEARANCE_MAX)
      ):
        self._road_topology_offset_limit = min(_MAX_ROAD_EDGE_OFFSET, float(np.min(right_clearance)) - _ROAD_EDGE_CLEARANCE_MIN)
        self._road_topology_reason = "confirmed_right_outer"
        return 1
      if (
        probs[0] <= _OUTER_LINE_ABSENT_MAX
        and right_adjacent
        and np.all(clearance_delta <= -_ROAD_EDGE_ASYMMETRY_MIN)
        and np.all(left_clearance >= _ROAD_EDGE_CLEARANCE_MIN)
        and np.all(left_clearance <= _ROAD_EDGE_CLEARANCE_MAX)
      ):
        self._road_topology_offset_limit = min(_MAX_ROAD_EDGE_OFFSET, float(np.min(left_clearance)) - _ROAD_EDGE_CLEARANCE_MIN)
        self._road_topology_reason = "confirmed_left_outer"
        return -1
      self._road_topology_reason = "ambiguous_topology"
      return 0
    except (AttributeError, IndexError, TypeError, ValueError):
      self._road_topology_reason = "topology_exception"
      return 0

  def _calculate_raw_correction(self, model_v2, v_ego: float, model_curvature: float,
                                offset: float, e2e_authority: float, road_topology_bias: float = 0.0) -> tuple[bool, float]:
    try:
      self._effective_road_topology_bias = 0.0
      lane_lines = model_v2.laneLines
      probs = np.asarray(model_v2.laneLineProbs, dtype=float)
      stds = np.asarray(model_v2.laneLineStds, dtype=float)
      if len(lane_lines) < 3 or probs.size < 3 or stds.size < 3:
        self._lane_centering_reason = "missing_lane_geometry"
        return False, 0.0
      if not np.isfinite(probs[[1, 2]]).all() or not np.isfinite(stds[[1, 2]]).all():
        self._lane_centering_reason = "invalid_lane_uncertainty"
        return False, 0.0
      if np.any(probs[[1, 2]] < _MIN_LANE_PROB) or np.any(probs[[1, 2]] > 1.0):
        self._lane_centering_reason = "untrusted_lane_probability"
        return False, 0.0
      if np.any(stds[[1, 2]] < 0.0) or np.any(stds[[1, 2]] > _MAX_LANE_STD):
        self._lane_centering_reason = "untrusted_lane_uncertainty"
        return False, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      pos_y_std = np.asarray(model_v2.position.yStd, dtype=float)
      if not (self._valid_path(left_x, left_y) and self._valid_path(right_x, right_y) and self._valid_path(pos_x, pos_y)):
        self._lane_centering_reason = "invalid_path_geometry"
        return False, 0.0
      if not self._valid_path(pos_x, pos_y_std):
        self._lane_centering_reason = "invalid_path_uncertainty"
        return False, 0.0

      lookahead_far = float(np.clip(v_ego, _LOOKAHEAD_MIN, _LOOKAHEAD_MAX))
      lookahead_near = max(_LOOKAHEAD_NEAR_MIN, lookahead_far * _LOOKAHEAD_NEAR_FRAC)
      lookaheads = np.linspace(lookahead_near, lookahead_far, _FIT_POINT_COUNT)
      if not all(self._covers(x, lookahead_far) for x in (left_x, right_x, pos_x)):
        self._lane_centering_reason = "short_path_geometry"
        return False, 0.0
      if not self._covers(pos_x, lookahead_far):
        self._lane_centering_reason = "short_path_uncertainty"
        return False, 0.0

      left = np.interp(lookaheads, left_x, left_y)
      right = np.interp(lookaheads, right_x, right_y)
      widths = right - left
      if np.any(widths < _MIN_LANE_WIDTH) or np.any(widths > _MAX_LANE_WIDTH):
        self._lane_centering_reason = "invalid_lane_width"
        return False, 0.0
      width_spread = float(np.max(widths) - np.min(widths))
      if width_spread > _MAX_LANE_WIDTH_SPREAD:
        self._lane_centering_reason = "diverging_lane_geometry"
        return False, 0.0

      path_stds = np.interp(lookaheads, pos_x, pos_y_std)
      if np.any(path_stds < 0.0) or np.any(path_stds > _E2E_MAX_PATH_STD):
        self._lane_centering_reason = "untrusted_path_uncertainty"
        return False, 0.0

      max_safe_offset = min(_MAX_OFFSET, max(0.0, float(np.min(widths)) * 0.5 - _MIN_CENTER_TO_LINE))
      # modelV2 paths use device coordinates (+y right), and desired curvature is
      # positive for right turns. Keep the user-facing offset convention positive-right.
      model_y = np.interp(lookaheads, pos_x, pos_y)
      if road_topology_bias > 0.0:
        available_outer_clearance = float(np.min(right - model_y)) - _MIN_CENTER_TO_LINE
        road_topology_bias = min(road_topology_bias, max(0.0, available_outer_clearance))
      elif road_topology_bias < 0.0:
        available_outer_clearance = float(np.min(model_y - left)) - _MIN_CENTER_TO_LINE
        road_topology_bias = -min(abs(road_topology_bias), max(0.0, available_outer_clearance))
      self._effective_road_topology_bias = float(road_topology_bias)
      target_offset = float(np.clip(offset + road_topology_bias, -max_safe_offset, max_safe_offset))
      target_y = 0.5 * (left + right) + target_offset
      errors = target_y - model_y
      error_abs = np.abs(errors)
      far_error_abs = float(error_abs[-1])
      errors = np.copysign(np.maximum(error_abs - _CENTER_ERROR_DEADBAND, 0.0), errors)

      # Fit one bounded curvature correction to the lane-relative error at several
      # distances. For small angles in openpilot's sign convention, an added positive
      # curvature k moves the path toward +y by approximately 0.5*k*x^2. A multi-point
      # least-squares fit is less sensitive to one noisy lane-line sample and follows
      # curved lane geometry better than a single pure-pursuit point.
      weights = np.linspace(0.7, 1.0, _FIT_POINT_COUNT)
      x2 = lookaheads ** 2
      correction_terms = weights * x2 * errors
      raw_correction = float(2.0 * np.sum(correction_terms) / np.sum(weights * x2 ** 2))
      term_magnitude = float(np.sum(np.abs(correction_terms)))
      correction_coherence = 1.0 if term_magnitude < 1e-9 else abs(float(np.sum(correction_terms))) / term_magnitude

      probability_scale = float(np.clip(
        (float(np.min(probs[[1, 2]])) - _MIN_LANE_PROB) / (_FULL_LANE_PROB - _MIN_LANE_PROB),
        0.0,
        1.0,
      ))
      uncertainty_scale = float(np.clip(
        (_MAX_LANE_STD - float(np.max(stds[[1, 2]]))) / (_MAX_LANE_STD - _FULL_LANE_STD),
        0.0,
        1.0,
      ))
      width_scale = float(np.clip(
        (_MAX_LANE_WIDTH_SPREAD - width_spread) / (_MAX_LANE_WIDTH_SPREAD - _FULL_LANE_WIDTH_SPREAD),
        0.0,
        1.0,
      ))
      coherence_scale = float(np.clip(
        (correction_coherence - _MIN_CORRECTION_COHERENCE) /
        (_FULL_CORRECTION_COHERENCE - _MIN_CORRECTION_COHERENCE),
        0.0,
        1.0,
      ))
      raw_correction *= min(probability_scale, uncertainty_scale, width_scale, coherence_scale)

      max_error_abs = float(np.max(error_abs))
      break_in = np.clip(
        (far_error_abs - _E2E_BREAK_IN_START) / (_E2E_BREAK_IN_FULL - _E2E_BREAK_IN_START),
        0.0,
        1.0,
      )
      model_scale = 1.0 - e2e_authority * float(break_in)
      envelope_scale = float(np.clip(
        (max_error_abs - _ENVELOPE_START) / (_ENVELOPE_FULL - _ENVELOPE_START),
        0.0,
        1.0,
      ))
      curve_scale = float(np.clip(
        (abs(model_curvature) * v_ego ** 2 - _CURVE_LAT_ACCEL_START) /
        (_CURVE_LAT_ACCEL_FULL - _CURVE_LAT_ACCEL_START),
        0.0,
        1.0,
      ))
      envelope_scale *= _ENVELOPE_MIN_SCALE + (1.0 - _ENVELOPE_MIN_SCALE) * curve_scale
      raw_correction *= max(model_scale, envelope_scale)

      self._lane_centering_reason = "active"
      return True, raw_correction
    except (AttributeError, IndexError, TypeError, ValueError):
      self._lane_centering_reason = "lane_geometry_exception"
      return False, 0.0
