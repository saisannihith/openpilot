from types import SimpleNamespace

from openpilot.selfdrive.controls.radard import (
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN,
  KalmanParams,
  Track,
  get_lead,
  is_carnival_confirmation_track,
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
      x=[0.0, 10.0, 20.0, 40.0],
      y=[path_y, path_y, path_y, path_y],
    ),
  )


def test_carnival_confirmation_track_id_range():
  assert is_carnival_confirmation_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN)
  assert is_carnival_confirmation_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1)
  assert not is_carnival_confirmation_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN - 1)
  assert not is_carnival_confirmation_track(42)


def test_carnival_confirmation_track_requires_path_and_maturity_for_radar_only_lead():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=10.0, y_rel=0.0, v_ego=1.0)

  assert track.confirmationOnly
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

  assert not track.confirmationOnly
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


def test_carnival_confirmation_track_rejects_velocity_mismatch():
  track = make_track(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN + 1, d_rel=20.0, y_rel=0.0, v_rel=-20.0, v_ego=10.0)
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


def test_carnival_confirmation_mature_consensus_blends_bounded_radar_velocity():
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
  assert lead_state["vRel"] == 0.93
  assert lead_state["vLead"] == 10.93


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
