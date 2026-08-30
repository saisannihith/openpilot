#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from types import SimpleNamespace
from typing import Any

import capnp
from cereal import messaging, log, car, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D
from openpilot.selfdrive.controls.lib.desire_helper import LaneChangeDirection, LaneChangeState
from openpilot.starpilot.common.starpilot_variables import get_starpilot_toggles
from opendbc.car.honda.radar_interface import BOSCH_A_FREQ_HZ
from opendbc.car.honda.values import HONDA_BOSCH_A


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 0.6

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame
G90_RADAR_LOW_SPEED_MAX_DIST = 12.0
G90_RADAR_LOW_SPEED_MAX_Y = 0.6
HONDA_BOSCH_A_RADAR_TS = 1.0 / BOSCH_A_FREQ_HZ
HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT = 3
HONDA_BOSCH_A_CHALLENGER_STALE_CYCLES = 2
HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES = 3
HONDA_BOSCH_A_GROSS_DISTANCE_M = 25.0


def is_bosch_a_radar_car(CP) -> bool:
  return CP.brand == "honda" and CP.carFingerprint in HONDA_BOSCH_A and not CP.radarUnavailable


# Adjacent-lane stopped-vehicle detector, used as a stop-line hint on red-light
# approaches. The qualifier is the DECELERATION HISTORY, not the current speed: roadside
# furniture and curb-parked cars never show a moving -> stopped transition, so testing
# for "anything slow in the next lane" instead would brake us early for parked cars.
ADJACENT_STOP_MOVING_V = 5.0      # m/s — must have genuinely been moving
ADJACENT_STOP_REST_V = 1.5        # m/s — and then genuinely at rest
ADJACENT_STOP_MOVING_FRAMES = 15  # 0.75 s at 20 Hz, both ways: rejects speed noise
ADJACENT_STOP_REST_FRAMES = 15
ADJACENT_STOP_MIN_Y = 1.8         # m — inside this is our own lane
ADJACENT_STOP_MAX_Y = 7.5         # m — beyond this is roadside, not an adjacent lane
ADJACENT_STOP_MAX_D = 110.0       # m
CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN = 0xC4100
CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MAX = 0xC41FF
CARNIVAL_4TH_GEN_PRIMARY_TRACK_ID_MIN = 0xC4200
CARNIVAL_4TH_GEN_PRIMARY_TRACK_ID_MAX = 0xC42FF
CARNIVAL_4TH_GEN_CONFIRMATION_DIST_SCALE = 0.22
CARNIVAL_4TH_GEN_CONFIRMATION_DIST_FLOOR = 4.0
CARNIVAL_4TH_GEN_CONFIRMATION_Y_STD_SCALE = 1.5
CARNIVAL_4TH_GEN_CONFIRMATION_Y_FLOOR = 1.2
CARNIVAL_4TH_GEN_CONFIRMATION_V_STD_SCALE = 3.0
CARNIVAL_4TH_GEN_CONFIRMATION_V_FLOOR = 4.0
CARNIVAL_4TH_GEN_CONFIRMATION_NIS_GATE = 11.345
CARNIVAL_4TH_GEN_CONFIRMATION_PREFERRED_NIS_GATE = 14.156
CARNIVAL_4TH_GEN_CONFIRMATION_SWITCH_SCORE_MARGIN = 3.841
CARNIVAL_4TH_GEN_CONFIRMATION_RADAR_D_STD = 0.25
CARNIVAL_4TH_GEN_CONFIRMATION_RADAR_Y_STD = 0.25
CARNIVAL_4TH_GEN_CONFIRMATION_RADAR_V_STD = 0.35
CARNIVAL_4TH_GEN_VELOCITY_MIN_FRAMES = 8
CARNIVAL_4TH_GEN_VELOCITY_MAX_DISTANCE = 40.0
CARNIVAL_4TH_GEN_VELOCITY_MAX_ABS_Y = 0.75
CARNIVAL_4TH_GEN_VELOCITY_MAX_PATH_OFFSET = 0.35
CARNIVAL_4TH_GEN_VELOCITY_MAX_NIS = 5.991
CARNIVAL_4TH_GEN_VELOCITY_MAX_RATE_RESIDUAL = 0.75
CARNIVAL_4TH_GEN_VELOCITY_MAX_CONSENSUS = 0.45
CARNIVAL_4TH_GEN_FAR_VELOCITY_MIN_DISTANCE = 40.0
CARNIVAL_4TH_GEN_FAR_VELOCITY_MAX_DISTANCE = 60.0
CARNIVAL_4TH_GEN_FAR_VELOCITY_MIN_SELECTED_FRAMES = 6
CARNIVAL_4TH_GEN_FAR_VELOCITY_MAX_CONSENSUS = 1.0
CARNIVAL_4TH_GEN_FAR_VELOCITY_BLEND_WEIGHT = 0.35
CARNIVAL_4TH_GEN_FAR_VELOCITY_MAX_CORRECTION = 0.35
CARNIVAL_4TH_GEN_REACQUIRE_MIN_FRAMES = 8
CARNIVAL_4TH_GEN_REACQUIRE_MAX_DISTANCE = 25.0
CARNIVAL_4TH_GEN_REACQUIRE_PATH_OFFSET = 1.1
CARNIVAL_4TH_GEN_RADAR_ONLY_MIN_FRAMES = 20
CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_V_EGO = 3.0
CARNIVAL_4TH_GEN_RADAR_ONLY_MIN_DISTANCE = 1.5
CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_DISTANCE = 18.0
CARNIVAL_4TH_GEN_RADAR_ONLY_PATH_OFFSET = 0.65
CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_ABS_Y = 1.25
CARNIVAL_4TH_GEN_RADAR_ONLY_PATH_MIN_FRAMES = 8
CARNIVAL_4TH_GEN_RADAR_ONLY_MIN_V_LEAD = -1.5
CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_V_LEAD = 6.0
CARNIVAL_4TH_GEN_RADAR_ONLY_DISTANCE_RATE_FRAMES = 8
CARNIVAL_4TH_GEN_RADAR_ONLY_DISTANCE_RATE_MAX_RESIDUAL = 1.5


def is_carnival_confirmation_track(identifier: int) -> bool:
  return CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN <= identifier <= CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MAX


ADJACENT_STOP_QUEUE_GAP_M = 5.0   # m — anything stopped beyond the furthest qualifier means
                                  # the bar is past it too, so the hint would stop us short


def is_carnival_primary_track(identifier: int) -> bool:
  return CARNIVAL_4TH_GEN_PRIMARY_TRACK_ID_MIN <= identifier <= CARNIVAL_4TH_GEN_PRIMARY_TRACK_ID_MAX


def is_carnival_r0100_track(identifier: int) -> bool:
  return is_carnival_confirmation_track(identifier) or is_carnival_primary_track(identifier)


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.carnivalR0100 = is_carnival_r0100_track(identifier)
    self.carnivalPrimary = is_carnival_primary_track(identifier)
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

    self.leadTrackID = 0

    # deceleration history for the adjacent-lane stopped-vehicle detector
    self.moving_frames = 0
    self.rest_frames = 0
    self.seen_moving = False
    self.radar_only_path_frames = 0
    self.radar_only_distance_history = deque(maxlen=CARNIVAL_4TH_GEN_RADAR_ONLY_DISTANCE_RATE_FRAMES)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: bool,
             measurement_update: bool | None = None):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate
    self.radar_only_distance_history.append(d_rel)

    # `measurement_update` is separate from the published measured bit so legacy radar sources keep
    # their existing behaviour. Civic Bosch emits real measurements at ~15 Hz while radard is driven
    # at the ~20 Hz model rate; duplicate liveTracks payloads must not be absorbed twice.
    if measurement_update is None:
      # Preserve the historical Track.update behaviour for direct/legacy callers. The radar source
      # adapter supplies an explicit False only for a duplicate Civic Bosch payload.
      measurement_update = True
    # computed velocity and accelerations
    if measurement_update and self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    if measurement_update:
      # Learn if constant acceleration
      if abs(self.aLeadK) < 0.5:
        self.aLeadTau.x = min(max(self.aLeadTau.x, 1e-2) * 1.1, _LEAD_ACCEL_TAU)
      else:
        self.aLeadTau.update(0.0)

      # Track the moving -> stopped transition. Only sustained runs count, so one noisy
      # speed sample can neither arm nor trip the detector.
      if self.vLead > ADJACENT_STOP_MOVING_V:
        self.moving_frames += 1
        self.rest_frames = 0
        if self.moving_frames >= ADJACENT_STOP_MOVING_FRAMES:
          self.seen_moving = True
      elif abs(self.vLead) < ADJACENT_STOP_REST_V:
        self.moving_frames = 0
        self.rest_frames += 1
      else:
        # coasting between the two bands: hold state, restart both runs
        self.moving_frames = 0
        self.rest_frames = 0

      self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_adjacent_lead(self, left: bool, standstill: bool, model_data: capnp._DynamicStructReader):
    if self.carnivalR0100:
      return False

    if standstill or self.vLead < 1 or self.leadTrackID == self.identifier:
      return False

    if left:
      left_lane = np.interp(self.dRel, model_data.laneLines[1].x, model_data.laneLines[1].y)
      return -self.yRel < left_lane
    right_lane = np.interp(self.dRel, model_data.laneLines[2].x, model_data.laneLines[2].y)
    return -self.yRel > right_lane

  def is_adjacent_stopped(self, model_data: capnp._DynamicStructReader):
    """A neighbouring-lane vehicle that was seen moving and has now come to rest.

    Deliberately not potential_adjacent_lead, which is moving-target-only and would have
    to be loosened to "anything slow" to catch these. Lane geometry mirrors it (model
    y == -yRel, laneLines[1] left boundary and [2] right), plus an outer bound so
    roadside returns past the neighbouring lane don't qualify.
    """
    if self.carnivalR0100:
      return False

    if not (self.seen_moving and self.rest_frames >= ADJACENT_STOP_REST_FRAMES):
      return False

    if self.leadTrackID == self.identifier:
      return False

    return self.in_adjacent_lane(model_data)

  def in_adjacent_lane(self, model_data: capnp._DynamicStructReader):
    """Lane geometry only, no deceleration history — also used to spot a queue ahead."""
    if not (ADJACENT_STOP_MIN_Y < abs(self.yRel) < ADJACENT_STOP_MAX_Y):
      return False

    if not (0.0 < self.dRel < ADJACENT_STOP_MAX_D):
      return False

    model_y = -self.yRel
    left_lane = np.interp(self.dRel, model_data.laneLines[1].x, model_data.laneLines[1].y)
    right_lane = np.interp(self.dRel, model_data.laneLines[2].x, model_data.laneLines[2].y)
    return bool(model_y < left_lane or model_y > right_lane)

  def potential_low_speed_lead(self, v_ego: float, model_data: capnp._DynamicStructReader | None = None,
                               previously_selected: bool = False,
                               model_path: tuple[list[float], list[float]] | None = None):
    if self.carnivalR0100:
      return carnival_low_speed_radar_lead_sane(self, v_ego, model_data, previously_selected, model_path)

    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def vision_track_probability(track: Track, lead: capnp._DynamicStructReader, v_ego: float) -> float:
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  prob_d = laplacian_pdf(track.dRel, offset_vision_dist, lead.xStd[0])
  prob_y = laplacian_pdf(track.yRel, -lead.y[0], lead.yStd[0])
  prob_v = laplacian_pdf(track.vRel + v_ego, lead.v[0], lead.vStd[0])
  return prob_d * prob_y * prob_v


def g90_radar_lead_lateral_sane(track: Track) -> bool:
  # The G90 extended radar channels can report close side ghosts in tight turns.
  # Keep the gate tight at close range, then widen gradually with distance.
  max_y = min(6.0, 1.5 + 0.08 * max(track.dRel, 0.0))
  return abs(track.yRel) <= max_y


def g90_low_speed_radar_lead_sane(track: Track, v_ego: float) -> bool:
  return (track.cnt >= 3 and v_ego < 3.0 and
          0.75 < track.dRel < G90_RADAR_LOW_SPEED_MAX_DIST and
          abs(track.yRel) < G90_RADAR_LOW_SPEED_MAX_Y)


def carnival_low_speed_radar_lead_sane(track: Track, v_ego: float, model_data: capnp._DynamicStructReader | None,
                                       previously_selected: bool,
                                       model_path: tuple[list[float], list[float]] | None = None) -> bool:
  if model_data is None or not track.measured:
    track.radar_only_path_frames = 0
    return False

  if model_path is None:
    position = getattr(model_data, "position", None)
    path_x = list(getattr(position, "x", []))
    path_y = list(getattr(position, "y", []))
  else:
    path_x, path_y = model_path
  if len(path_x) < 2 or len(path_x) != len(path_y):
    track.radar_only_path_frames = 0
    return False

  object_x = track.dRel + RADAR_TO_CAMERA
  if object_x < path_x[0] or object_x > path_x[-1]:
    track.radar_only_path_frames = 0
    return False
  path_offset = abs(-track.yRel - float(np.interp(object_x, path_x, path_y)))
  if len(track.radar_only_distance_history) < CARNIVAL_4TH_GEN_RADAR_ONLY_DISTANCE_RATE_FRAMES:
    track.radar_only_path_frames = 0
    return False
  observed_v_rel = ((track.radar_only_distance_history[-1] - track.radar_only_distance_history[0]) /
                    ((len(track.radar_only_distance_history) - 1) * DT_MDL))
  distance_rate_sane = abs(observed_v_rel - track.vRel) <= CARNIVAL_4TH_GEN_RADAR_ONLY_DISTANCE_RATE_MAX_RESIDUAL

  if previously_selected:
    track.radar_only_path_frames = 0
    return (track.cnt >= CARNIVAL_4TH_GEN_REACQUIRE_MIN_FRAMES and
            v_ego < V_EGO_STATIONARY and
            CARNIVAL_4TH_GEN_RADAR_ONLY_MIN_DISTANCE < track.dRel < CARNIVAL_4TH_GEN_REACQUIRE_MAX_DISTANCE and
            path_offset < CARNIVAL_4TH_GEN_REACQUIRE_PATH_OFFSET and
            distance_rate_sane)

  path_aligned = (track.cnt >= CARNIVAL_4TH_GEN_RADAR_ONLY_MIN_FRAMES and
                  v_ego < CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_V_EGO and
                  CARNIVAL_4TH_GEN_RADAR_ONLY_MIN_DISTANCE < track.dRel < CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_DISTANCE and
                  abs(track.yRel) < CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_ABS_Y and
                  path_offset < CARNIVAL_4TH_GEN_RADAR_ONLY_PATH_OFFSET and
                  CARNIVAL_4TH_GEN_RADAR_ONLY_MIN_V_LEAD < track.vLead < CARNIVAL_4TH_GEN_RADAR_ONLY_MAX_V_LEAD)
  track.radar_only_path_frames = track.radar_only_path_frames + 1 if path_aligned else 0
  return track.radar_only_path_frames >= CARNIVAL_4TH_GEN_RADAR_ONLY_PATH_MIN_FRAMES and distance_rate_sane


def honda_bosch_a_low_speed_radar_lead_sane(track: Track, v_ego: float) -> bool:
  """Require a few real Bosch sweeps before a radar-only low-speed takeover."""
  return track.cnt >= HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT and track.potential_low_speed_lead(v_ego)


def track_matches_vision(track: Track, lead: capnp._DynamicStructReader, v_ego: float, *,
                         dist_scale: float, dist_floor: float, vel_limit: float,
                         y_std_scale: float, y_floor: float) -> bool:
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  dist_sane = abs(track.dRel - offset_vision_dist) < max(abs(offset_vision_dist) * dist_scale, dist_floor)
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < vel_limit) or (v_ego + track.vRel > 3)
  lat_sane = abs(track.yRel + lead.y[0]) < max(y_floor, y_std_scale * max(float(lead.yStd[0]), 0.2))
  return dist_sane and vel_sane and lat_sane


def carnival_confirmation_matches_vision(track: Track, lead: capnp._DynamicStructReader, v_ego: float,
                                         model_v_ego: float | None = None) -> bool:
  if not getattr(track, "carnivalR0100", False) or track.cnt < 1:
    return False

  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  dist_sane = abs(track.dRel - offset_vision_dist) < max(abs(offset_vision_dist) * CARNIVAL_4TH_GEN_CONFIRMATION_DIST_SCALE,
                                                         CARNIVAL_4TH_GEN_CONFIRMATION_DIST_FLOOR)
  lat_sane = abs(track.yRel + lead.y[0]) < max(CARNIVAL_4TH_GEN_CONFIRMATION_Y_FLOOR,
                                               CARNIVAL_4TH_GEN_CONFIRMATION_Y_STD_SCALE * max(float(lead.yStd[0]), 0.2))
  association_v_ego = v_ego if model_v_ego is None else model_v_ego
  vel_sane = abs(track.vRel - (lead.v[0] - association_v_ego)) < max(
    CARNIVAL_4TH_GEN_CONFIRMATION_V_FLOOR,
    CARNIVAL_4TH_GEN_CONFIRMATION_V_STD_SCALE * max(float(lead.vStd[0]), 0.5),
  )
  physical_speed_sane = track.vRel + v_ego > -2.0
  return dist_sane and lat_sane and vel_sane and physical_speed_sane


def carnival_confirmation_innovation_score(track: Track, lead: capnp._DynamicStructReader,
                                            model_v_ego: float) -> float:
  """Three-dimensional normalized innovation score for R0100 association.

  The decoded radar velocity is reliable only after the radar object has been
  associated with the same physical lead as modelV2. Normalize each residual by
  the combined model/radar uncertainty so distance, lateral position, and
  velocity contribute in comparable units.
  """
  if not getattr(track, "carnivalR0100", False) or track.cnt < 1:
    return math.inf

  try:
    residual = np.asarray([
      float(track.dRel) - (float(lead.x[0]) - RADAR_TO_CAMERA),
      float(track.yRel) + float(lead.y[0]),
      float(track.vRel) - (float(lead.v[0]) - float(model_v_ego)),
    ])
    model_std = np.asarray([
      np.clip(float(lead.xStd[0]), 0.75, 6.0),
      np.clip(float(lead.yStd[0]), 0.25, 1.5),
      np.clip(float(lead.vStd[0]), 0.5, 3.0),
    ])
    radar_std = np.asarray([
      CARNIVAL_4TH_GEN_CONFIRMATION_RADAR_D_STD,
      CARNIVAL_4TH_GEN_CONFIRMATION_RADAR_Y_STD,
      CARNIVAL_4TH_GEN_CONFIRMATION_RADAR_V_STD,
    ])
    if not np.isfinite(residual).all() or not np.isfinite(model_std).all():
      return math.inf
    return float(np.sum(residual ** 2 / (model_std ** 2 + radar_std ** 2)))
  except (AttributeError, IndexError, TypeError, ValueError):
    return math.inf


def carnival_primary_velocity_refinement(track: Track, lead_msg: capnp._DynamicStructReader,
                                         model_data: capnp._DynamicStructReader, model_v_ego: float,
                                         selected_frames: int = 0) -> tuple[float, bool] | None:
  """Return a bounded R0100 velocity refinement for an already model-matched lead."""
  if (not track.carnivalPrimary or not track.measured or
      track.cnt < CARNIVAL_4TH_GEN_VELOCITY_MIN_FRAMES or
      not (0.0 < track.dRel <= CARNIVAL_4TH_GEN_FAR_VELOCITY_MAX_DISTANCE) or
      abs(track.yRel) > CARNIVAL_4TH_GEN_VELOCITY_MAX_ABS_Y or
      carnival_confirmation_innovation_score(track, lead_msg, model_v_ego) > CARNIVAL_4TH_GEN_VELOCITY_MAX_NIS):
    return None

  position = getattr(model_data, "position", None)
  path_x = list(getattr(position, "x", []))
  path_y = list(getattr(position, "y", []))
  object_x = track.dRel + RADAR_TO_CAMERA
  if len(path_x) < 2 or len(path_x) != len(path_y) or object_x < path_x[0] or object_x > path_x[-1]:
    return None
  path_offset = abs(-track.yRel - float(np.interp(object_x, path_x, path_y)))
  if path_offset > CARNIVAL_4TH_GEN_VELOCITY_MAX_PATH_OFFSET:
    return None

  history = track.radar_only_distance_history
  if len(history) < CARNIVAL_4TH_GEN_VELOCITY_MIN_FRAMES:
    return None
  samples = np.asarray(history, dtype=float)
  sample_indices = np.arange(len(samples), dtype=float)
  centered_indices = sample_indices - np.mean(sample_indices)
  denominator = float(np.dot(centered_indices, centered_indices)) * DT_MDL
  if denominator <= 0.0:
    return None
  observed_v_rel = float(np.dot(centered_indices, samples - np.mean(samples)) / denominator)
  if not math.isfinite(observed_v_rel) or abs(track.vRel - observed_v_rel) > CARNIVAL_4TH_GEN_VELOCITY_MAX_RATE_RESIDUAL:
    return None

  model_v_rel = float(lead_msg.v[0] - model_v_ego)
  radar_v_rel = float(track.vRel)
  velocity_residual = radar_v_rel - model_v_rel
  if track.dRel <= CARNIVAL_4TH_GEN_VELOCITY_MAX_DISTANCE:
    if abs(velocity_residual) > CARNIVAL_4TH_GEN_VELOCITY_MAX_CONSENSUS:
      return None
    return radar_v_rel, True

  if (track.dRel <= CARNIVAL_4TH_GEN_FAR_VELOCITY_MIN_DISTANCE or
      selected_frames < CARNIVAL_4TH_GEN_FAR_VELOCITY_MIN_SELECTED_FRAMES or
      abs(velocity_residual) > CARNIVAL_4TH_GEN_FAR_VELOCITY_MAX_CONSENSUS):
    return None
  correction = float(np.clip(
    CARNIVAL_4TH_GEN_FAR_VELOCITY_BLEND_WEIGHT * velocity_residual,
    -CARNIVAL_4TH_GEN_FAR_VELOCITY_MAX_CORRECTION,
    CARNIVAL_4TH_GEN_FAR_VELOCITY_MAX_CORRECTION,
  ))
  return model_v_rel + correction, False


def get_RadarState_from_carnival_confirmation(track: Track, lead_msg: capnp._DynamicStructReader,
                                              model_data: capnp._DynamicStructReader, v_ego: float,
                                              model_v_ego: float, model_prob: float, selected_frames: int = 0):
  model_v_rel = float(lead_msg.v[0] - model_v_ego)
  refinement = carnival_primary_velocity_refinement(track, lead_msg, model_data, model_v_ego, selected_frames)
  v_rel, direct_radar_velocity = (model_v_rel, False) if refinement is None else refinement
  v_lead = float(v_ego + v_rel)
  v_lead_k = float(track.vLeadK) if direct_radar_velocity else v_lead
  a_lead_k = float(track.aLeadK) if direct_radar_velocity else float(lead_msg.a[0])
  a_lead_tau = float(track.aLeadTau.x) if direct_radar_velocity else 0.3
  return {
    "dRel": float(track.dRel),
    "yRel": float(track.yRel),
    "vRel": v_rel,
    "vLead": v_lead,
    "vLeadK": v_lead_k,
    "aLeadK": a_lead_k,
    "aLeadTau": a_lead_tau,
    "status": True,
    "fcw": track.is_potential_fcw(model_prob),
    "modelProb": model_prob,
    "radar": True,
    "radarTrackId": track.identifier,
  }


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, model_data: capnp._DynamicStructReader, tracks: dict[int, Track],
                          starpilot_toggles: SimpleNamespace, g90_radar_filter: bool = False,
                          preferred_track_id: int = -1, model_v_ego: float | None = None):
  if model_data.meta.laneChangeState == LaneChangeState.laneChangeStarting and getattr(starpilot_toggles, "human_lane_changes", False):
    direction = model_data.meta.laneChangeDirection
    if direction == LaneChangeDirection.left:
      tracks = {k: v for k, v in tracks.items() if v.yRel > 0}
    elif direction == LaneChangeDirection.right:
      tracks = {k: v for k, v in tracks.items() if v.yRel < 0}

  if g90_radar_filter:
    tracks = {k: v for k, v in tracks.items() if g90_radar_lead_lateral_sane(v)}

  if not tracks:
    return None

  association_v_ego = v_ego if model_v_ego is None else model_v_ego
  confirmation_candidates = []
  for identifier, track in tracks.items():
    if not carnival_confirmation_matches_vision(track, lead, v_ego, association_v_ego):
      continue
    score = carnival_confirmation_innovation_score(track, lead, association_v_ego)
    gate = (CARNIVAL_4TH_GEN_CONFIRMATION_PREFERRED_NIS_GATE
            if identifier == preferred_track_id and track.cnt >= 3
            else CARNIVAL_4TH_GEN_CONFIRMATION_NIS_GATE)
    if score <= gate and track.vRel + v_ego > -2.0:
      confirmation_candidates.append((score, identifier, track))
  if confirmation_candidates:
    best_score, _, best_track = min(confirmation_candidates, key=lambda candidate: candidate[0])
    primary = min((candidate for candidate in confirmation_candidates if candidate[2].carnivalPrimary),
                  default=None, key=lambda candidate: candidate[0])
    if primary is not None:
      return primary[2]
    preferred = next((candidate for candidate in confirmation_candidates
                      if candidate[1] == preferred_track_id), None)
    # A new object must improve the normalized innovation by a chi-square 95%
    # one-degree margin before replacing a still-plausible previous target.
    if preferred is not None and preferred[0] <= best_score + CARNIVAL_4TH_GEN_CONFIRMATION_SWITCH_SCORE_MARGIN:
      return preferred[2]
    return best_track

  # R0100 tracks have their own model-first contract above. Do not let a failed
  # qualification fall through to the looser generic matcher.
  tracks = {identifier: track for identifier, track in tracks.items()
            if not getattr(track, "carnivalR0100", False)}
  if not tracks:
    return None

  track = max(tracks.values(), key=lambda candidate: vision_track_probability(candidate, lead, v_ego))

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  if track_matches_vision(track, lead, v_ego,
                          dist_scale=0.25, dist_floor=5.0,
                          vel_limit=10.0, y_std_scale=1.0, y_floor=1.0):
    return track

  # Some vehicles intermittently drop a good radar match on large leads (semis are
  # a common offender). If the same track is still present and only missed the
  # strict vision gate by a small margin, keep the previous radar match instead of
  # oscillating between radar and vision estimates.
  preferred_track = tracks.get(preferred_track_id)
  if preferred_track is not None and preferred_track.cnt >= 3:
    if track_matches_vision(preferred_track, lead, v_ego,
                            dist_scale=0.40, dist_floor=8.0,
                            vel_limit=13.0, y_std_scale=2.0, y_floor=1.5):
      return preferred_track
  return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, model_prob: float):
  prev_aLeadK = getattr(get_RadarState_from_vision, "prev_aLeadK", 0.0)
  blended_aLeadK = 0.8 * float(lead_msg.a[0]) + 0.2 * prev_aLeadK
  get_RadarState_from_vision.prev_aLeadK = blended_aLeadK
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_msg.v[0] - model_v_ego),
    "vLead": float(v_ego + (lead_msg.v[0] - model_v_ego)),
    "vLeadK": float(v_ego + (lead_msg.v[0] - model_v_ego)),
    "aLeadK": blended_aLeadK,
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(model_prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, model_data: capnp._DynamicStructReader, standstill: bool,
             starpilot_plan: capnp._DynamicStructReader, starpilot_toggles: SimpleNamespace,
             low_speed_override: bool = True, g90_radar_filter: bool = False, lead_prob: float | None = None,
             preferred_track_id: int = -1, preferred_track_frames: int = 0,
             honda_bosch_a_radar: bool = False) -> dict[str, Any]:
  lead_detection_probability = float(getattr(starpilot_toggles, "lead_detection_probability", 0.35))
  filtered_lead_prob = float(lead_msg.prob if lead_prob is None else lead_prob)

  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and filtered_lead_prob > lead_detection_probability:
    track = match_vision_to_track(v_ego, lead_msg, model_data, tracks, starpilot_toggles, g90_radar_filter,
                                  preferred_track_id=preferred_track_id, model_v_ego=model_v_ego)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    if track.carnivalR0100:
      lead_dict = get_RadarState_from_carnival_confirmation(track, lead_msg, model_data, v_ego,
                                                            model_v_ego, filtered_lead_prob,
                                                            preferred_track_frames + 1 if track.identifier == preferred_track_id else 1)
    else:
      lead_dict = track.get_RadarState(filtered_lead_prob)
  elif (track is None) and ready and (filtered_lead_prob > lead_detection_probability):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, filtered_lead_prob)

  if low_speed_override:
    if g90_radar_filter:
      low_speed_tracks = [c for c in tracks.values() if g90_low_speed_radar_lead_sane(c, v_ego)]
    elif honda_bosch_a_radar:
      low_speed_tracks = [c for c in tracks.values() if honda_bosch_a_low_speed_radar_lead_sane(c, v_ego)]
    else:
      position = getattr(model_data, "position", None)
      model_path = (list(getattr(position, "x", [])), list(getattr(position, "y", [])))
      low_speed_tracks = [
        c for c in tracks.values()
        if c.potential_low_speed_lead(v_ego, model_data, c.identifier == preferred_track_id, model_path)
      ]

    model_lead_available = ready and filtered_lead_prob > lead_detection_probability

    # Keep a previously selected Bosch radar track through ordinary model-probability fluctuations
    # when it is still coherent. If the model has a valid lead, the old track must still agree with
    # that lead; no model lead leaves the mature radar track eligible for continuity.
    if honda_bosch_a_radar:
      preferred_track = tracks.get(preferred_track_id)
      if (preferred_track is not None and honda_bosch_a_low_speed_radar_lead_sane(preferred_track, v_ego)):
        preferred_matches_model = (not model_lead_available or
                                   track_matches_vision(preferred_track, lead_msg, v_ego,
                                                        dist_scale=0.25, dist_floor=5.0,
                                                        vel_limit=10.0, y_std_scale=1.0, y_floor=1.0))
        preferred_is_current = (not lead_dict.get('status', False) or
                                lead_dict.get('radarTrackId', -1) == preferred_track_id or
                                (lead_dict.get('status', False) and not lead_dict.get('radar', False)))
        if preferred_is_current and preferred_matches_model:
          lead_dict = preferred_track.get_RadarState(filtered_lead_prob)

    def candidate_is_established(candidate: Track) -> bool:
      if not honda_bosch_a_radar:
        return True
      if candidate.cnt < HONDA_BOSCH_A_LOW_SPEED_MIN_COUNT:
        return False
      if not lead_dict.get('status', False):
        # A mature centered Bosch point may provide the radar-only low-speed lead.
        return True
      if lead_dict.get('radarTrackId', -1) == candidate.identifier:
        return True
      # Do not replace an established lead with an unrelated closer point when there is no model
      # evidence to arbitrate them. A different candidate may take over only after it agrees with
      # the available model lead; radar-only takeover remains possible when lead_dict is invalid.
      return (model_lead_available and
              track_matches_vision(candidate, lead_msg, v_ego,
                                   dist_scale=0.25, dist_floor=5.0,
                                   vel_limit=10.0, y_std_scale=1.0, y_floor=1.0))

    low_speed_tracks = [c for c in low_speed_tracks if candidate_is_established(c)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  for track in tracks.values():
    track.leadTrackID = lead_dict.get('radarTrackId', -1)

  if 'dRel' in lead_dict:
    lead_dict['dRel'] -= starpilot_plan.increasedStoppedDistance

  return lead_dict


def get_adjacent_lead(tracks: dict[int, Track], standstill: bool, model_data: capnp._DynamicStructReader, left: bool = True) -> dict[str, Any]:
  lead_dict = {'status': False}

  adjacent_tracks = [c for c in tracks.values() if c.potential_adjacent_lead(left, standstill, model_data)]
  if len(adjacent_tracks) > 0:
    closest_track = min(adjacent_tracks, key=lambda c: c.dRel)
    lead_dict = closest_track.get_RadarState()

  return lead_dict


def get_adjacent_stopped(tracks: dict[int, Track], model_data: capnp._DynamicStructReader) -> dict[str, Any]:
  """Stop-line hint: a vehicle that decelerated to a stop in a neighbouring lane.

  Takes the FARTHEST qualifying vehicle, then drops the hint entirely if a queue reaches
  past it. Cars already stopped when we acquire them never show the moving -> stopped
  transition, so the qualifying set is biased toward the back of a line; without this the
  hint marks a mid-queue bumper and stops us short of the bar.
  """
  if len(model_data.laneLines) < 4:
    return {'status': False}

  candidates = [c for c in tracks.values() if c.is_adjacent_stopped(model_data)]
  if not candidates:
    return {'status': False}

  furthest = max(candidates, key=lambda c: c.dRel)
  for c in tracks.values():
    if (c.dRel > furthest.dRel + ADJACENT_STOP_QUEUE_GAP_M and
        abs(c.vLead) < ADJACENT_STOP_REST_V and
        c.in_adjacent_lane(model_data)):
      return {'status': False}
  return {
    'status': True,
    'dRel': float(furthest.dRel),
    'yRel': float(furthest.yRel),
    'radarTrackId': int(furthest.identifier),
  }


class RadarD:
  def __init__(self, radar_ts: float = DT_MDL, delay: float = 0.0, g90_radar_filter: bool = False,
               honda_bosch_a_radar: bool = False):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.honda_bosch_a_radar = honda_bosch_a_radar
    # The lead KF consumes Bosch measurements at the physical radar cadence. Lead probability
    # filters, however, consume modelV2 leads every model cycle and must retain model-loop timing.
    kf_dt = HONDA_BOSCH_A_RADAR_TS if self.honda_bosch_a_radar else radar_ts
    self.kalman_params = KalmanParams(kf_dt)
    self.g90_radar_filter = g90_radar_filter
    lead_prob_dt = DT_MDL if self.honda_bosch_a_radar else radar_ts
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, lead_prob_dt) for _ in range(2)]
    self.prev_lead_track_ids = [-1, -1]
    self.prev_lead_track_frames = [0, 0]
    self.preferred_stale_track_ids = [-1, -1]
    self.preferred_challenger_stale_counts = [0, 0]
    self.preferred_gross_distance_stale_counts = [0, 0]

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL)) + 1)
    self.last_v_ego_frame = -1
    self._last_tracks_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    self.starpilot_radar_state = custom.StarPilotRadarState.new_message()
    self.starpilot_toggles = get_starpilot_toggles()

  def _reset_preferred_stale_evidence(self, lead_index: int, track_id: int = -1) -> None:
    self.preferred_stale_track_ids[lead_index] = track_id
    self.preferred_challenger_stale_counts[lead_index] = 0
    self.preferred_gross_distance_stale_counts[lead_index] = 0

  def _update_honda_bosch_a_preferred_staleness(self, lead_index: int, lead: capnp._DynamicStructReader,
                                               lead_prob: float) -> None:
    if not self.honda_bosch_a_radar:
      return

    preferred_id = self.prev_lead_track_ids[lead_index]
    if self.preferred_stale_track_ids[lead_index] != preferred_id:
      self._reset_preferred_stale_evidence(lead_index, preferred_id)

    lead_detection_probability = float(getattr(self.starpilot_toggles, "lead_detection_probability", 0.35))
    preferred_track = self.tracks.get(preferred_id)
    if preferred_id < 0 or preferred_track is None or not self.ready or lead_prob <= lead_detection_probability:
      self._reset_preferred_stale_evidence(lead_index, preferred_id)
      return

    strict_match = track_matches_vision(preferred_track, lead, self.v_ego,
                                        dist_scale=0.25, dist_floor=5.0,
                                        vel_limit=10.0, y_std_scale=1.0, y_floor=1.0)
    relaxed_match = track_matches_vision(preferred_track, lead, self.v_ego,
                                         dist_scale=0.40, dist_floor=8.0,
                                         vel_limit=13.0, y_std_scale=2.0, y_floor=1.5)

    # Arm A: a preferred track that no longer passes continuity may be stale when another live
    # track has a better association score. Clearing preference never selects that challenger;
    # the unchanged strict match path below remains the only way it can become a radar lead.
    if relaxed_match:
      self.preferred_challenger_stale_counts[lead_index] = 0
    else:
      best_track = max(self.tracks.values(), key=lambda candidate: vision_track_probability(candidate, lead, self.v_ego))
      preferred_score = vision_track_probability(preferred_track, lead, self.v_ego)
      best_score = vision_track_probability(best_track, lead, self.v_ego)
      if best_track.identifier != preferred_id and best_score > preferred_score:
        self.preferred_challenger_stale_counts[lead_index] += 1
      else:
        self.preferred_challenger_stale_counts[lead_index] = 0

    # Arm B: gross absolute range disagreement is independent evidence of staleness, but a strict
    # match is authoritative and resets the streak even when model uncertainty permits >25 m error.
    distance_mismatch = abs(preferred_track.dRel - (lead.x[0] - RADAR_TO_CAMERA))
    if strict_match:
      self.preferred_gross_distance_stale_counts[lead_index] = 0
    elif distance_mismatch > HONDA_BOSCH_A_GROSS_DISTANCE_M:
      self.preferred_gross_distance_stale_counts[lead_index] += 1
    else:
      self.preferred_gross_distance_stale_counts[lead_index] = 0

    challenger_stale = self.preferred_challenger_stale_counts[lead_index] >= HONDA_BOSCH_A_CHALLENGER_STALE_CYCLES
    distance_stale = self.preferred_gross_distance_stale_counts[lead_index] >= HONDA_BOSCH_A_GROSS_DISTANCE_STALE_CYCLES
    if challenger_stale or distance_stale:
      self.prev_lead_track_ids[lead_index] = -1
      self._reset_preferred_stale_evidence(lead_index)

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9 * max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    radar_fresh = True
    if self.honda_bosch_a_radar:
      radar_fresh = sm.recv_frame['liveTracks'] != self._last_tracks_frame
      self._last_tracks_frame = sm.recv_frame['liveTracks']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids, rpt in ar_pts.items():
      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      measured = rpt[3] if not self.honda_bosch_a_radar else bool(rpt[3] and radar_fresh)
      # Non-Bosch sources retain the historical per-model-cycle update semantics. Only Civic Bosch
      # suppresses duplicate measurement updates when liveTracks has not advanced.
      measurement_update = True if not self.honda_bosch_a_radar else measured
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, measured, measurement_update)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    self.starpilot_radar_state = custom.StarPilotRadarState.new_message()

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego

    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        lead_prob = float(leads_v3[i].prob)
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

        self._update_honda_bosch_a_preferred_staleness(i, leads_v3[i], self.lead_prob_filters[i].x)

      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, sm['modelV2'],
                                          sm['carState'].standstill, sm['starpilotPlan'], self.starpilot_toggles, low_speed_override=True,
                                          g90_radar_filter=self.g90_radar_filter, lead_prob=self.lead_prob_filters[0].x,
                                          preferred_track_id=self.prev_lead_track_ids[0],
                                          preferred_track_frames=self.prev_lead_track_frames[0],
                                          honda_bosch_a_radar=self.honda_bosch_a_radar)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, sm['modelV2'],
                                          sm['carState'].standstill, sm['starpilotPlan'], self.starpilot_toggles, low_speed_override=False,
                                          g90_radar_filter=self.g90_radar_filter, lead_prob=self.lead_prob_filters[1].x,
                                          preferred_track_id=self.prev_lead_track_ids[1],
                                          preferred_track_frames=self.prev_lead_track_frames[1],
                                          honda_bosch_a_radar=self.honda_bosch_a_radar)

      for i, lead in enumerate((self.radar_state.leadOne, self.radar_state.leadTwo)):
        if lead.status and getattr(lead, "radar", False):
          track_id = int(getattr(lead, "radarTrackId", -1))
          self.prev_lead_track_frames[i] = self.prev_lead_track_frames[i] + 1 if track_id == self.prev_lead_track_ids[i] else 1
          if track_id != self.prev_lead_track_ids[i]:
            self._reset_preferred_stale_evidence(i, track_id)
          self.prev_lead_track_ids[i] = track_id
        elif (not lead.status) or (self.prev_lead_track_ids[i] not in self.tracks):
          self.prev_lead_track_ids[i] = -1
          self.prev_lead_track_frames[i] = 0
          self._reset_preferred_stale_evidence(i)
        else:
          # Retain the preferred ID for normal reacquisition, but require a new
          # consecutive radar-match streak before far-range velocity blending.
          self.prev_lead_track_frames[i] = 0

    if self.ready and (self.starpilot_toggles.adjacent_lead_tracking or self.starpilot_toggles.human_lane_changes):
      self.starpilot_radar_state.leadLeft = get_adjacent_lead(self.tracks, sm['carState'].standstill, sm['modelV2'], left=True)
      self.starpilot_radar_state.leadRight = get_adjacent_lead(self.tracks, sm['carState'].standstill, sm['modelV2'], left=False)

    # Not gated on the adjacent-lead toggles: this is a separate signal with a separate
    # consumer (Force Stop), and leaving leadLeft/leadRight untouched keeps existing
    # lane-change and UI behaviour unchanged.
    if self.ready:
      self.starpilot_radar_state.adjacentStopped = get_adjacent_stopped(self.tracks, sm['modelV2'])

    self.starpilot_toggles = get_starpilot_toggles(sm)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)

    starpilot_radar_msg = messaging.new_message("starpilotRadarState")
    starpilot_radar_msg.valid = self.radar_state_valid
    starpilot_radar_msg.starpilotRadarState = self.starpilot_radar_state
    pm.send("starpilotRadarState", starpilot_radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2',
                           ignore_valid=['starpilotPlan'])
  pm = messaging.PubMaster(['radarState'])

  radar_ts = float(getattr(CP, "radarTimeStepDEPRECATED", DT_MDL) or DT_MDL)
  if not 0.01 < radar_ts < 0.2:
    radar_ts = DT_MDL

  g90_radar_filter = CP.brand == "hyundai" and CP.carFingerprint == "GENESIS_G90"
  honda_bosch_a_radar = is_bosch_a_radar_car(CP)
  RD = RadarD(radar_ts=radar_ts, delay=CP.radarDelay, g90_radar_filter=g90_radar_filter,
              honda_bosch_a_radar=honda_bosch_a_radar)

  sm = sm.extend(['starpilotPlan'])
  pm = pm.extend(['starpilotRadarState'])

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
