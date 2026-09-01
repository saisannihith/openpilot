from types import SimpleNamespace

from openpilot.selfdrive.controls.radard import (
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN,
  KalmanParams,
  Track,
  carnival_confirmation_innovation_score,
  carnival_model_first_velocity,
  carnival_trailing_relative_velocity,
  get_lead,
  is_carnival_confirmation_track,
  is_carnival_r0100_track,
)


def make_track(identifier: int, d_rel: float = 12.0, y_rel: float = 0.0, v_rel: float = 0.0, v_ego: float = 2.0) -> Track:
  track = Track(identifier, v_ego + v_rel, KalmanParams(0.05))
  track.update(d_rel, y_rel, v_rel, v_ego + v_rel, True)
  return track


def mature_track(track: Track, frames: int = 5) -> Track:
  for _ in range(frames - track.cnt):
    track.update(track.dRel, track.yRel, track.vRel, track.vLead, True)
  return track


def make_model_data(path_y: float = 0.0):
  return SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
    position=SimpleNamespace(
      x=[0.0, 10.0, 20.0, 40.0, 80.0],
      y=[path_y, path_y, path_y, path_y, path_y],
    ),
  )


def make_velocity_track(relative_velocity: float = -1.0, raw_velocity: float | None = None) -> Track:
  track = Track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, 9.0, KalmanParams(0.05))
  for frame in range(25):
    time_s = frame * 0.05
    distance = 40.0 + relative_velocity * time_s
    measured_velocity = relative_velocity if raw_velocity is None else raw_velocity
    track.update(distance, 0.0, measured_velocity, 10.0 + measured_velocity, True,
                 measurement_time=time_s)
  return track


def test_carnival_confirmation_track_id_range():
  assert is_carnival_confirmation_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN)
  assert is_carnival_confirmation_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1)
  assert not is_carnival_confirmation_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN - 1)
  assert not is_carnival_confirmation_track(42)
  assert is_carnival_r0100_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN)
  assert not is_carnival_r0100_track(0xC4200)
  assert not is_carnival_r0100_track(42)


def test_carnival_confirmation_track_requires_path_and_maturity_for_radar_only_lead():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=10.0, y_rel=0.0, v_ego=1.0)

  assert track.carnivalR0100
  assert not hasattr(track, "confirmationOnly")
  assert not track.potential_low_speed_lead(1.0)
  assert not track.potential_low_speed_lead(1.0, make_model_data())


def test_carnival_confirmation_track_can_create_strict_low_speed_radar_only_lead():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=10.0, y_rel=0.0, v_rel=-1.0, v_ego=1.0), frames=20)
  model_data = make_model_data()
  for _ in range(7):
    assert not track.potential_low_speed_lead(1.0, model_data)
  lead = SimpleNamespace(
    prob=0.1,
    x=[11.52],
    y=[0.0],
    v=[0.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=1.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=1.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["status"]
  assert lead_state["radar"]
  assert lead_state["radarTrackId"] == track.identifier
  assert lead_state["vRel"] == -1.0
  assert lead_state["vLead"] == 0.0


def test_carnival_radar_only_lead_rejects_object_outside_model_path():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=10.0, y_rel=1.4, v_rel=-1.0, v_ego=1.0), frames=30)

  assert not track.potential_low_speed_lead(1.0, make_model_data())


def test_carnival_radar_only_lead_rejects_sub_bumper_return():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=0.8, y_rel=0.0, v_rel=-0.75, v_ego=0.0), frames=30)
  model_data = make_model_data()

  for _ in range(8):
    assert not track.potential_low_speed_lead(0.0, model_data)
  assert not track.potential_low_speed_lead(0.0, model_data, previously_selected=True)


def test_carnival_radar_only_lead_follows_curved_model_path():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=10.0, y_rel=-0.8, v_rel=-1.0, v_ego=1.0), frames=20)

  curved_path = make_model_data(path_y=0.8)
  for _ in range(7):
    assert not track.potential_low_speed_lead(1.0, curved_path)
  assert track.potential_low_speed_lead(1.0, curved_path)
  assert not track.potential_low_speed_lead(1.0, make_model_data(path_y=-0.8))


def test_carnival_radar_only_lead_rejects_kinematically_inconsistent_side_object():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                     d_rel=10.0, y_rel=0.0, v_rel=-1.0, v_ego=1.0)
  for frame in range(1, 30):
    distance = 10.0 + 0.05 * frame
    track.update(distance, 0.0, -1.0, 0.0, True)

  model_data = make_model_data()
  for _ in range(8):
    assert not track.potential_low_speed_lead(1.0, model_data)
  assert not track.potential_low_speed_lead(1.0, model_data, previously_selected=True)


def test_carnival_previously_selected_track_reacquires_with_shorter_history():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=22.0, y_rel=0.0, v_rel=1.0, v_ego=2.0), frames=8)

  assert track.potential_low_speed_lead(2.0, make_model_data(), previously_selected=True)
  assert not track.potential_low_speed_lead(2.0, make_model_data(), previously_selected=False)


def test_normal_track_can_still_be_low_speed_radar_only_lead():
  track = make_track(42, d_rel=10.0, y_rel=0.0, v_ego=1.0)

  assert not hasattr(track, "confirmationOnly")
  assert not track.carnivalR0100
  assert track.potential_low_speed_lead(1.0)


def test_carnival_confirmation_track_cannot_be_adjacent_lead_or_stop_hint():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=18.0, y_rel=4.0, v_ego=6.0)
  track.seen_moving = True
  track.rest_frames = 30

  assert not track.potential_adjacent_lead(left=True, standstill=False, model_data=None)
  assert not track.potential_adjacent_lead(left=False, standstill=False, model_data=None)
  assert not track.is_adjacent_stopped(model_data=None)


def test_carnival_confirmation_track_can_confirm_matching_model_lead():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=20.0, y_rel=0.0, v_rel=0.8, v_ego=10.0)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.8,
    x=[21.52],
    y=[0.0],
    v=[11.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  confirmed = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert confirmed["status"]
  assert confirmed["radar"]
  assert confirmed["radarTrackId"] == track.identifier
  assert confirmed["dRel"] == track.dRel
  assert confirmed["vRel"] == 1.0
  assert confirmed["vLead"] == 11.0


def test_carnival_confirmation_moving_velocity_mismatch_keeps_geometry_association():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=20.0, y_rel=0.0, v_rel=-5.0, v_ego=10.0)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.8,
    x=[21.52],
    y=[0.0],
    v=[11.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["status"]
  assert lead_state["radar"]
  assert lead_state["radarTrackId"] == track.identifier
  assert lead_state["vRel"] == 1.0


def test_carnival_confirmation_mature_consensus_keeps_velocity_model_led():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=20.0, y_rel=0.0, v_rel=0.8, v_ego=10.0))
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.8,
    x=[21.52],
    y=[0.0],
    v=[11.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["radar"]
  assert lead_state["vRel"] == 1.0
  assert lead_state["vLead"] == 11.0


def test_carnival_associated_lead_velocity_stays_model_led():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                     d_rel=20.0, y_rel=0.0, v_rel=-0.6, v_ego=10.0)
  for frame in range(1, 8):
    track.update(20.0 - 0.6 * 0.05 * frame, 0.0, -0.6, 9.4, True)
  model_data = make_model_data()
  lead = SimpleNamespace(
    prob=0.8,
    x=[track.dRel + 1.52],
    y=[0.0],
    v=[9.8],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["radar"]
  assert abs(lead_state["vRel"] + 0.2) < 1e-6


def test_carnival_confirmation_velocity_disagreement_stays_model_led():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=20.0, y_rel=0.0, v_rel=3.5, v_ego=10.0))
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.8,
    x=[21.52],
    y=[0.0],
    v=[11.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["radar"]
  assert lead_state["vRel"] == 1.0
  assert lead_state["vLead"] == 11.0


def test_carnival_confirmation_distance_rate_keeps_geometry_but_model_velocity():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                     d_rel=30.0, y_rel=0.0, v_rel=-14.0, v_ego=22.0)
  for frame in range(1, 8):
    distance = 30.0 - 12.0 * 0.05 * frame
    track.update(distance, 0.0, -14.0, 8.0, True)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.9,
    x=[track.dRel + 1.52],
    y=[0.0],
    v=[21.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=22.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=22.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["radar"]
  assert lead_state["radarTrackId"] == track.identifier
  assert lead_state["vRel"] == -1.0

  track.update(track.dRel, 0.0, -14.0, 8.0, True)
  held_state = get_lead(
    v_ego=22.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=22.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )
  assert held_state["radar"]
  assert held_state["radarTrackId"] == track.identifier
  assert held_state["vRel"] == -1.0


def test_carnival_confirmation_inconsistent_distance_rate_stays_model_led():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                     d_rel=30.0, y_rel=0.0, v_rel=-14.0, v_ego=22.0)
  for frame in range(1, 8):
    track.update(30.0 + 0.05 * frame, 0.0, -14.0, 8.0, True)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.9,
    x=[track.dRel + 1.52],
    y=[0.0],
    v=[21.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=22.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=22.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["radar"]
  assert lead_state["radarTrackId"] == track.identifier
  assert lead_state["vRel"] == -1.0


def test_carnival_confirmation_track_cannot_create_lead_without_model_confidence():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=20.0, y_rel=0.0, v_rel=1.0, v_ego=10.0)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.2,
    x=[21.52],
    y=[0.0],
    v=[11.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert not lead_state["status"]


def test_carnival_confirmation_track_falls_back_to_vision_when_model_does_not_match():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=80.0, y_rel=0.0, v_rel=1.0, v_ego=10.0)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.8,
    x=[21.52],
    y=[0.0],
    v=[11.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["status"]
  assert not lead_state["radar"]
  assert lead_state["radarTrackId"] == -1


def test_carnival_confirmation_track_is_preferred_over_generic_track_for_matching_model_lead():
  confirmation_track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=20.0, y_rel=0.0, v_rel=1.0, v_ego=10.0)
  generic_track = make_track(42, d_rel=20.2, y_rel=0.0, v_rel=1.0, v_ego=10.0)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.8,
    x=[21.52],
    y=[0.0],
    v=[11.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0,
    ready=True,
    tracks={generic_track.identifier: generic_track, confirmation_track.identifier: confirmation_track},
    lead_msg=lead,
    model_v_ego=10.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )

  assert lead_state["status"]
  assert lead_state["radar"]
  assert lead_state["radarTrackId"] == confirmation_track.identifier
  assert lead_state["vRel"] == 1.0


def test_carnival_model_association_uses_best_innovation() -> None:
  less_likely = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 7,
                           d_rel=20.3, y_rel=0.1, v_rel=1.1, v_ego=10.0)
  secondary = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 8,
                         d_rel=20.0, y_rel=0.0, v_rel=1.0, v_ego=10.0)
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.8, x=[21.52], y=[0.0], v=[11.0], a=[0.0],
    xStd=[2.0], yStd=[0.3], vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0, ready=True,
    tracks={secondary.identifier: secondary, less_likely.identifier: less_likely},
    lead_msg=lead, model_v_ego=10.0, model_data=model_data,
    standstill=False, starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )
  assert lead_state["radarTrackId"] == secondary.identifier

  less_likely.update(20.3, 0.1, -8.0, 2.0, True)
  lead_state = get_lead(
    v_ego=10.0, ready=True,
    tracks={secondary.identifier: secondary, less_likely.identifier: less_likely},
    lead_msg=lead, model_v_ego=10.0, model_data=model_data,
    standstill=False, starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
  )
  assert lead_state["radarTrackId"] == secondary.identifier


def test_carnival_preferred_confirmation_track_rejects_large_model_distance_jump():
  track = mature_track(make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                                  d_rel=50.0, y_rel=-0.4, v_rel=-4.0, v_ego=20.0))
  model_data = SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0),
    laneLines=[],
  )
  lead = SimpleNamespace(
    prob=0.9,
    x=[71.52],
    y=[0.0],
    v=[19.0],
    a=[0.0],
    xStd=[2.0],
    yStd=[0.3],
    vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=20.0,
    ready=True,
    tracks={track.identifier: track},
    lead_msg=lead,
    model_v_ego=20.0,
    model_data=model_data,
    standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True,
    preferred_track_id=track.identifier,
  )

  assert not lead_state["radar"]
  assert lead_state["radarTrackId"] == -1


def test_carnival_confirmation_innovation_score_uses_distance_lateral_and_velocity():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1,
                     d_rel=20.0, y_rel=-0.2, v_rel=-1.0, v_ego=10.0)
  lead = SimpleNamespace(
    x=[21.52], y=[0.2], v=[9.0], xStd=[1.0], yStd=[0.3], vStd=[0.7],
  )
  matching_score = carnival_confirmation_innovation_score(track, lead, 10.0)
  lead.v[0] = 15.0
  mismatching_score = carnival_confirmation_innovation_score(track, lead, 10.0)

  assert matching_score == 0.0
  assert mismatching_score == 0.0

  track.vRel = -9.0
  stationary_mismatch_score = carnival_confirmation_innovation_score(track, lead, 10.0)
  assert stationary_mismatch_score > 11.345


def test_carnival_trailing_velocity_recovers_causal_distance_rate():
  track = make_velocity_track(relative_velocity=-1.0)

  estimate = carnival_trailing_relative_velocity(track, model_a_rel=0.0)

  assert estimate is not None
  assert abs(estimate + 1.0) < 1e-6


def test_carnival_velocity_correction_requires_stable_model_association():
  track = make_velocity_track(relative_velocity=-1.0)

  immature = carnival_model_first_velocity(track, -0.2, 0.0, 0.9, 11, track.identifier)
  switched = carnival_model_first_velocity(track, -0.2, 0.0, 0.9, 12, track.identifier + 1)
  corrected = carnival_model_first_velocity(track, -0.2, 0.0, 0.9, 12, track.identifier)

  assert immature == -0.2
  assert switched == -0.2
  assert abs(corrected - -0.28) < 1e-6


def test_carnival_velocity_correction_rejects_raw_range_disagreement():
  track = make_velocity_track(relative_velocity=-1.0, raw_velocity=2.0)

  corrected = carnival_model_first_velocity(track, -0.2, 0.0, 0.9, 12, track.identifier)

  assert corrected == -0.2


def test_carnival_velocity_correction_is_hard_bounded():
  track = make_velocity_track(relative_velocity=-3.0)

  corrected = carnival_model_first_velocity(track, 1.0, 0.0, 0.9, 12, track.identifier)

  assert abs(corrected - 0.9) < 1e-6


def test_carnival_distance_discontinuity_resets_velocity_history():
  track = make_velocity_track(relative_velocity=-1.0)

  track.update(90.0, 0.0, -1.0, 9.0, True, measurement_time=2.0)

  assert len(track.carnival_distance_history) == 1
  assert carnival_trailing_relative_velocity(track, model_a_rel=0.0) is None


def test_carnival_velocity_uses_raw_model_probability_not_filtered_holdover():
  track = make_velocity_track(relative_velocity=-1.0)
  lead = SimpleNamespace(
    prob=0.7, x=[track.dRel + 1.52], y=[0.0], v=[9.8], a=[0.0],
    xStd=[2.0], yStd=[0.3], vStd=[1.0],
  )

  lead_state = get_lead(
    v_ego=10.0, ready=True, tracks={track.identifier: track}, lead_msg=lead,
    model_v_ego=10.0, model_data=make_model_data(), standstill=False,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(lead_detection_probability=0.35, human_lane_changes=False),
    low_speed_override=True, lead_prob=0.9, preferred_track_id=track.identifier,
    selected_frames=12,
  )

  assert lead_state["radar"]
  assert abs(lead_state["vRel"] - -0.2) < 1e-6
