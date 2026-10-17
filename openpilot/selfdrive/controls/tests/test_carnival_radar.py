from types import SimpleNamespace

from openpilot.selfdrive.controls.carnival_radar import CarnivalRadarPolicy
from openpilot.selfdrive.controls.radard import RadarD
from opendbc.car import structs


def lead(prob=0.95, distance=41.52, velocity=15.0):
  return SimpleNamespace(prob=prob, x=[distance], xStd=[2.0], y=[0.0], yStd=[0.5],
                         v=[velocity], vStd=[1.0], a=[0.0])


def model():
  return SimpleNamespace(position=SimpleNamespace(x=list(range(101)), y=[0.0] * 101))


class Track:
  identifier = 0xC4101

  def __init__(self, distance=40.0, velocity=-5.0):
    self.dRel = distance
    self.yRel = 0.0
    self.vRel = velocity
    self.vLead = 15.0

  def get_RadarState(self, prob=0.0):
    return {"present": True, "dRel": self.dRel, "vRel": self.vRel,
            "radar": True, "radarTrackId": self.identifier, "modelProb": prob}


def sample(policy, track, n=1):
  for index in range(n):
    policy.update({track.identifier: track}, fresh=True, sample_time=(index + 1) * 0.05)


def test_model_led_velocity_when_radar_disagrees():
  policy = CarnivalRadarPolicy()
  track = Track(40.0, -12.0)
  sample(policy, track, 3)
  result = policy.lead(0, {track.identifier: track}, lead(), model(), 20.0, 20.0, 0.0, 0.95, True, True)
  assert result["present"] and result["radar"]
  assert result["radarTrackId"] == track.identifier
  assert result["dRel"] == 40.0
  assert result["vRel"] == -5.0


def test_no_highway_radar_only_lead():
  policy = CarnivalRadarPolicy()
  track = Track(12.0, -18.0)
  sample(policy, track, 30)
  result = policy.lead(0, {track.identifier: track}, lead(prob=0.0), model(), 20.0, 20.0, 0.0, 0.0, True, True)
  assert result == {"present": False}


def test_stale_radar_returns_model_only():
  policy = CarnivalRadarPolicy()
  track = Track()
  sample(policy, track, 3)
  result = policy.lead(0, {track.identifier: track}, lead(), model(), 20.0, 20.0, 0.0, 0.95, True, False)
  assert result["present"] and not result["radar"]


def test_low_speed_requires_sustained_path_and_distance_rate():
  policy = CarnivalRadarPolicy()
  track = Track(10.0, -1.0)
  track.vLead = 1.0
  result = {"present": False}
  for index in range(30):
    track.dRel = 10.0 - index * 0.05
    policy.update({track.identifier: track}, fresh=True, sample_time=(index + 1) * 0.05)
    result = policy.lead(0, {track.identifier: track}, lead(prob=0.0), model(), 2.0, 2.0, 0.0, 0.0, True, True)
    if index < 26:
      assert not result["present"]
  assert result["present"] and result["radar"]


def test_lateral_mismatch_blocks_radar_association():
  policy = CarnivalRadarPolicy()
  track = Track()
  track.yRel = 3.0
  sample(policy, track, 3)
  result = policy.lead(0, {track.identifier: track}, lead(), model(), 20.0, 20.0, 0.0, 0.95, True, True)
  assert result["present"] and not result["radar"]


def test_radard_carnival_path_keeps_model_velocity():
  class SubMaster:
    def __init__(self):
      self.messages = {
        "carState": SimpleNamespace(vEgo=20.0),
        "modelV2": SimpleNamespace(velocity=SimpleNamespace(x=[20.0]),
                                    acceleration=SimpleNamespace(x=[0.0]),
                                    leadsV3=[lead(), lead(prob=0.0)],
                                    position=model().position),
      }
      self.seen = {"modelV2": True}
      self.recv_frame = {"carState": 1}
      self.valid = {"radarTracks": True}
      self.updated = {"radarTracks": True}
      self.logMonoTime = {"radarTracks": 1_000_000_000, "modelV2": 1_025_000_000}

    def __getitem__(self, name):
      return self.messages[name]

    def all_checks(self):
      return True

  policy = RadarD(carnival_r0100=True)
  rr = structs.RadarData()
  point = structs.RadarData.RadarPoint()
  point.trackId = 0xC4101
  point.dRel = 40.0
  point.yRel = 0.0
  point.vRel = -12.0
  rr.points = [point]
  sm = SubMaster()
  policy.update(sm, rr)
  assert policy.radar_state.leadOne.present
  assert policy.radar_state.leadOne.radar
  assert policy.radar_state.leadOne.vRel == -5.0
