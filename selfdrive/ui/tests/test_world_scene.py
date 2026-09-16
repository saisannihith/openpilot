import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.onroad.world_scene import MAX_OBJECTS, WorldScene, lateral_at, polyline


class Messages(dict):
  def tick(self, frame, now):
    self.valid = dict.fromkeys(self, True)
    self.alive = dict.fromkeys(self, True)
    self.recv_frame = dict.fromkeys(self, frame)
    self.recv_time = dict.fromkeys(self, now)


def messages():
  def line(y):
    return NS(x=[0, 20, 40], y=[y, y + .2, y + .8])
  lead = NS(status=True, dRel=20., yRel=3., modelProb=.99)
  sm = Messages(modelV2=NS(position=line(0), laneLines=[line(y) for y in (-5.4,-1.8,1.8,5.4)],
                         laneLineProbs=[.9]*4, roadEdges=[], roadEdgeStds=[]),
                radarState=NS(leadOne=lead, leadTwo=NS(status=False)))
  sm.tick(2, 10.)
  return sm


def test_same_coordinate_system_for_model_and_radar():
  sm, scene = messages(), WorldScene()
  scene.update(sm, 0, 10.)
  assert scene.objects[0].right == -3.
  assert scene.lanes[1][0][1] < 0
  assert scene.path == ((0.,0.), (20.,.2), (40.,.8))


@pytest.mark.parametrize('field', ['valid', 'alive', 'recv_frame', 'recv_time'])
def test_bad_or_stale_messages_cannot_create_a_scene(field):
  sm, scene = messages(), WorldScene()
  getattr(sm, field).update(dict.fromkeys(sm, 0))
  scene.update(sm, 1, 10.)
  assert not scene.path and not scene.objects


def test_frozen_messages_expire_and_new_drive_resets():
  sm, scene = messages(), WorldScene()
  scene.update(sm, 0, 10.)
  scene.update(sm, 0, 10.5)
  assert not scene.path and not scene.objects
  sm.tick(3, 11.)
  scene.update(sm, 3, 11.)
  assert not scene.path and not scene.objects


def test_display_rate_does_not_move_objects_or_rebuild_geometry():
  sm, scene = messages(), WorldScene()
  scene.update(sm, 0, 10.)
  objects, path, revision = scene.objects, scene.path, scene.revision
  for i in range(20):
    scene.update(sm, 0, 10. + i*.01)
  assert scene.objects is objects and scene.path is path and scene.revision == revision


def test_road_horizon_can_shorten_without_disappearing():
  assert polyline(NS(x=[0,10,20,19], y=[0,0,0,0])) == ((0.,0.),(10.,0.),(20.,0.))
  assert polyline(NS(x=[0,10,140], y=[0,0,0])) == ((0.,0.),(10.,0.))


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_geometry_is_not_published(bad):
  assert not polyline(NS(x=[0,10,20], y=[0,bad,0]))


def test_uncertain_lanes_hide_without_changing_model_path():
  sm, scene = messages(), WorldScene()
  sm['modelV2'].laneLineProbs = [.1]*4
  scene.update(sm, 0, 10.)
  assert scene.path and not any(scene.lanes)


def test_path_width_does_not_recenter_actual_path():
  sm, scene = messages(), WorldScene()
  scene.update(sm, 0, 10.)
  assert scene.path_half_width(20,.2) == .85
  assert scene.path_half_width(20,1.9) == .08
  assert lateral_at(scene.path,20) == .2


def test_raw_targets_are_bounded_and_not_given_a_vehicle_class():
  sm, scene = messages(), WorldScene()
  sm['liveTracks'] = NS(errors=NS(canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False),
                        points=[NS(trackId=i,dRel=i+1,yRel=6) for i in range(200)])
  sm.tick(2,10.)
  scene.update(sm, 0, 10.)
  assert len(scene.objects) <= MAX_OBJECTS
  assert all(not obj.vehicle for obj in scene.objects if obj.key[0] == 'radar')
  sm['liveTracks'].errors.canError = True
  sm.tick(3,10.1)
  scene.update(sm, 0, 10.1)
  assert all(obj.key[0] != 'radar' for obj in scene.objects)


def test_radar_only_lead_is_not_claimed_to_be_a_classified_vehicle():
  sm, scene = messages(), WorldScene()
  sm['radarState'].leadOne.modelProb = 0.
  scene.update(sm, 0, 10.)
  assert not scene.objects[0].vehicle


def test_disappeared_object_has_no_ghost_retention():
  sm, scene = messages(), WorldScene()
  scene.update(sm, 0, 10.)
  sm['radarState'].leadOne.status = False
  sm.tick(3, 10.1)
  scene.update(sm, 0, 10.1)
  assert not scene.objects
