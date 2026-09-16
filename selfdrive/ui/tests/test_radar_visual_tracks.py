import math
from types import SimpleNamespace as NS
import pytest
from openpilot.selfdrive.ui.onroad.radar_visual_tracks import RadarVisualTracks
from openpilot.selfdrive.ui.onroad.world_scene import WorldScene, vehicle_center, in_lane_corridor, road_yaw


def point(d=40.,v=-5.,y=-3.5,track=17,measured=True):
  return NS(trackId=track,dRel=d,yRel=y,vRel=v,measured=measured)


def test_fast_lane_membership_matches_existing_tangent_qualification():
  for slope in (-.15,0.,.15):
    lanes = tuple(tuple((float(x),y+slope*x) for x in range(0,121,4)) for y in (-5.4,-1.8,1.8,5.4))
    for missing in (-1,0,1,2,3):
      visible = tuple(line if i != missing else () for i,line in enumerate(lanes))
      for d in (-1.,0.,1.,10.,50.,119.,120.,121.):
        for y in (-20.,-5.,-1.8,0.,1.8,5.,20.):
          assert in_lane_corridor(visible,d,y) == (road_yaw(visible,d,y) is not None)


def run(track, velocity=-5., ego=20.):
  result = {}
  for i in range(10):
    result = track.update([point(d=40+velocity*i*.05,v=velocity)],10+i*.05,ego,lambda d,y:True)
    if i < 6:
      assert not result
  return result


@pytest.mark.parametrize('velocity,direction',[(-5.,1),(-40.,-1)])
def test_persistent_motion_gets_generic_avatar_direction(velocity,direction):
  tracker = RadarVisualTracks()
  assert run(tracker,velocity) == {17:direction}


def test_stationary_returns_are_not_parked_cars():
  assert not run(RadarVisualTracks(),-20.)


def test_observed_moving_target_can_stop_without_disappearing():
  tracker = RadarVisualTracks()
  run(tracker)
  assert tracker.update([point(d=37.,v=-20.)],10.5,20.,lambda d,y:True) == {17:1}


@pytest.mark.parametrize('invalid',[
  point(d=math.nan),point(v=math.nan),point(measured=False),point(track=-1),point(d=-1),point(v=101)])
def test_invalid_or_unmeasured_returns_do_not_get_avatars(invalid):
  tracker = RadarVisualTracks()
  for i in range(20):
    assert not tracker.update([invalid],i*.05,20,lambda d,y:True)
  assert not tracker.history


def test_dropout_jump_recycled_id_offroad_geometry_and_duplicate_reset_history():
  tracker = RadarVisualTracks()
  run(tracker)
  assert not tracker.update([],10.5,20,lambda d,y:True)
  assert not tracker.history
  run(tracker)
  assert not tracker.update([point(d=90.)],10.5,20,lambda d,y:True)
  run(tracker)
  assert not tracker.update([point()],11.,20,lambda d,y:True)
  assert not tracker.update([point()],11.05,20,lambda d,y:False)
  assert not tracker.history
  assert not tracker.update([point(),point()],11.1,20,lambda d,y:True)
  assert not tracker.history


def test_history_is_bounded_for_changing_ids():
  tracker = RadarVisualTracks()
  for i in range(100):
    tracker.update((point(track=i*1000+j) for j in range(1000)),i*.05,20,lambda d,y:True)
    assert len(tracker.history) <= 128
  tracker.reset()
  assert not tracker.history


def test_live_scene_deduplicates_lead_and_uses_radar_positions_without_lane_snapping():
  from openpilot.selfdrive.ui.tests.test_world_scene import messages
  scene,sm = WorldScene(),messages()
  sm['radarState'].leadOne.status = False
  sm['carState'] = NS(vEgo=20.)
  sm['liveTracks'] = NS(errors=NS(canError=False,radarFault=False,wrongConfig=False,radarUnavailableTemporary=False),points=[])
  for i in range(10):
    sm['liveTracks'].points = [point(d=25-i*.25,y=-3.5)]
    sm.tick(i+2,10+i*.05)
    scene.update(sm,0,10+i*.05)
  obj = scene.objects[0]
  assert obj.radar_avatar and not obj.vehicle
  assert obj.right == 3.5
  assert vehicle_center(obj) == (obj.forward,obj.right)
  sm['radarState'].leadOne = NS(status=True,dRel=22.5,yRel=-3.5,modelProb=.99,radar=True,radarTrackId=17)
  sm.tick(20,10.5)
  scene.update(sm,0,10.5)
  assert len(scene.objects) == 1 and scene.objects[0].vehicle
  scene.update(sm,0,11.)
  assert not scene.objects and not scene.radar_visuals.history
