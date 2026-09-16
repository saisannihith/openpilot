import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.onroad.world_scene import MAX_OBJECTS, SceneObject, WorldScene, lateral_at, path_yaw, polyline, road_yaw, vehicle_center


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


@pytest.mark.parametrize('slope', [-.4, 0., .4])
def test_avatar_heading_matches_near_path_and_raylib_axis(slope):
  points = ((0.,0.),(6.,6*slope),(30.,-20.))
  yaw = path_yaw(points)
  assert yaw == pytest.approx(-math.degrees(math.atan(slope)))
  # A right turn rotates the negative-Z nose toward positive-X.
  assert -math.sin(math.radians(yaw)) == pytest.approx(slope/math.sqrt(1+slope*slope))


def test_heading_does_not_anticipate_far_curve_or_use_missing_path():
  assert path_yaw(((0,0),(6,0),(60,30))) == 0.
  assert path_yaw(()) == 0.
  assert path_yaw(((10,4),(20,8))) == 0.
  assert abs(path_yaw(((0,0),(2,40)))) == 35.


def test_heading_filter_is_message_driven_and_expires():
  sm, scene = messages(), WorldScene()
  scene.update(sm, 0, 10.)
  old = scene.ego_yaw
  sm['modelV2'].position.y = [0,4,8]
  sm.tick(3,10.05)
  scene.update(sm,0,10.05)
  assert path_yaw(scene.path) < scene.ego_yaw < old
  heading = scene.ego_yaw
  scene.update(sm,0,10.1)
  assert scene.ego_yaw == heading
  scene.update(sm,0,11.)
  assert scene.ego_yaw == 0.


@pytest.mark.parametrize('curve',[-.006,0.,.006])
@pytest.mark.parametrize('offset',[-3.6,-1.8,0.,1.8,3.6])
def test_vehicle_tangent_across_lanes_and_crossings_without_snapping(curve,offset):
  lines = tuple(tuple((float(x),y+curve*x*x) for x in range(0,81,2)) for y in (-5.4,-1.8,1.8,5.4))
  forward,right = 20.,curve*400+offset
  yaw = road_yaw(lines,forward,right)
  assert yaw == pytest.approx(-math.degrees(math.atan(2*curve*forward)))
  obj = SceneObject(('lead',0),forward,right,True,10.,yaw=yaw)
  center_x,center_y = vehicle_center(obj)
  a = math.radians(yaw)
  assert center_x-2.4*math.cos(a) == pytest.approx(forward)
  assert center_y+2.4*math.sin(a) == pytest.approx(right)
  assert (obj.forward,obj.right) == (forward,right)


def test_no_heading_is_invented_without_lane_support():
  assert road_yaw(((),(),(),()),20.,0.) is None
  assert road_yaw((((0,0),(50,0)),((0,9),(50,9))),20.,3.) is None
  assert road_yaw((((0,0),(50,0)),((0,3.5),(50,3.5))),60.,1.) is None


def test_track_replacement_does_not_slide_old_vehicle_into_new_target():
  sm,scene = messages(),WorldScene()
  lead = sm['radarState'].leadOne
  lead.radar,lead.radarTrackId = True,7
  scene.update(sm,0,10.)
  lead.radarTrackId,lead.dRel,lead.yRel = 8,21.,2.5
  sm.tick(3,10.05)
  scene.update(sm,0,10.05)
  assert (scene.objects[0].forward,scene.objects[0].right) == (21.,-2.5)


def test_same_radar_identity_is_deduplicated_but_distinct_close_tracks_remain():
  sm,scene = messages(),WorldScene()
  lead = sm['radarState'].leadOne
  lead.radar,lead.radarTrackId = True,7
  sm['radarState'].leadTwo = NS(status=True,dRel=21.,yRel=2.5,modelProb=.99,radar=True,radarTrackId=8)
  sm['liveTracks'] = NS(errors=NS(canError=False,radarFault=False,wrongConfig=False,radarUnavailableTemporary=False),
                        points=[NS(trackId=7,dRel=24.,yRel=3.)])
  sm.tick(2,10.)
  scene.update(sm,0,10.)
  assert len(scene.objects) == 2
  assert {o.identity for o in scene.objects} == {('track',7),('track',8)}


def test_slot_handoff_keeps_identity_and_does_not_blend_different_cars():
  sm,scene = messages(),WorldScene()
  lead = sm['radarState'].leadOne
  lead.radar,lead.radarTrackId = True,7
  scene.update(sm,0,10.)
  sm['radarState'].leadTwo = lead
  sm['radarState'].leadOne = NS(status=True,dRel=40.,yRel=-1.,modelProb=.99,radar=True,radarTrackId=8)
  lead.dRel = 21.
  sm.tick(3,10.05)
  scene.update(sm,0,10.05)
  by_id = {o.identity:o for o in scene.objects}
  assert by_id[('track',8)].forward == 40.
  assert 20. < by_id[('track',7)].forward < 21.
  assert by_id[('track',7)].key == ('lead',1)


def test_model_only_refresh_updates_heading_without_refiltering_positions():
  sm,scene = messages(),WorldScene()
  sm['radarState'].leadOne.yRel = 0.
  scene.update(sm,0,10.)
  before = scene.objects[0]
  old_yaw = before.yaw
  for line in sm['modelV2'].laneLines:
    line.y = [y+.05*x for x,y in zip(line.x,line.y,strict=True)]
  sm.recv_frame['modelV2'],sm.recv_time['modelV2'] = 3,10.05
  scene.update(sm,0,10.05)
  assert scene.objects[0] is before and before.right == 0. and before.forward == 20.
  assert before.yaw < old_yaw


def test_crossing_vehicle_is_not_pinned_to_a_lane_center():
  sm,scene = messages(),WorldScene()
  lead = sm['radarState'].leadOne
  lead.yRel = 0.
  scene.update(sm,0,10.)
  rights = []
  for step in range(1,21):
    lead.yRel = -.2*step
    sm.tick(step+2,10.+.05*step)
    scene.update(sm,0,10.+.05*step)
    rights.append(scene.objects[0].right)
  assert all(b > a for a,b in zip(rights,rights[1:],strict=False))
  assert 3.7 < rights[-1] <= 4.
  lead.status = False
  sm.tick(23,11.05)
  scene.update(sm,0,11.05)
  assert not scene.objects


def test_new_drive_cannot_reuse_old_vehicle_smoothing():
  sm,scene = messages(),WorldScene()
  scene.update(sm,0,10.)
  sm['radarState'].leadOne.dRel = 21.
  sm.tick(5,10.05)
  scene.update(sm,4,10.05)
  assert scene.objects[0].forward == 21.


def test_raw_refresh_does_not_reverse_smoothed_lead_position():
  sm,scene = messages(),WorldScene()
  sm['liveTracks'] = NS(errors=NS(canError=False,radarFault=False,wrongConfig=False,radarUnavailableTemporary=False),points=[])
  sm.tick(2,10.)
  scene.update(sm,0,10.)
  sm['radarState'].leadOne.dRel = 21.
  sm.recv_frame['radarState'],sm.recv_time['radarState'] = 3,10.05
  scene.update(sm,0,10.05)
  previous = scene.objects[0].forward
  for step in range(6,11):
    sm.recv_frame['liveTracks'],sm.recv_time['liveTracks'] = step,10.+.01*step
    scene.update(sm,0,10.+.01*step)
    assert scene.objects[0].forward == previous
