import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.onroad.world_scene import SceneObject, MAX_OBJECTS, vehicle_center
from openpilot.selfdrive.ui.onroad.world_presentation import WorldPresentation, WorldAvailability, world_lead_lines
from openpilot.selfdrive.ui.tests.test_world_scene import messages


def car(d=20., y=0., t=10., track=7, vehicle=False, yaw=0.):
  return SceneObject(('lead',0) if vehicle else ('radar',track),d,y,vehicle,t,
                     ('track',track),yaw,radar_avatar=not vehicle)


@pytest.mark.parametrize('to_lead', [True,False])
def test_identity_handoff_keeps_mesh_and_label_anchor_continuous(to_lead):
  view = WorldPresentation()
  old = car(vehicle=not to_lead)
  view.update([old],0,10.)
  old_center = view.pose(old)
  new = car(t=10.05,vehicle=to_lead)
  view.update([new],0,10.05)
  assert view.pose(new) == old_center
  view.update([new],0,10.075)
  center = view.pose(new)
  assert center.forward == pytest.approx((old_center.forward+vehicle_center(new)[0])/2)
  view.update([new],0,10.11)
  assert view.pose(new).forward == pytest.approx(vehicle_center(new)[0])
  assert len(view.poses) == 1


def test_observed_cut_in_and_pass_are_interpolated_not_lane_snapped_or_extrapolated():
  view = WorldPresentation()
  view.update([car(d=25,y=3.5)],0,10.)
  new = car(d=24,y=3.,t=10.05)
  view.update([new],0,10.05)
  view.update([new],0,10.075)
  assert view.pose(new).forward == pytest.approx(24.5)
  assert view.pose(new).right == pytest.approx(3.25)
  view.update([new],0,10.2)
  assert view.pose(new).right == 3.
  assert view.pose(new).forward == 24.
  view.update([],0,10.21)
  assert not view.poses and not view.transitions


def test_actor_lifecycle_is_immediate_then_short_fade_without_a_ghost():
  view = WorldPresentation()
  obj = car()
  view.update([obj], 0, 10.)
  assert .34 <= view.pose(obj).alpha <= .36
  view.update([obj], 0, 10.12)
  assert view.pose(obj).alpha == pytest.approx(1.)
  obj = car(t=10.12)
  view.update([obj], 0, 10.12)
  view.update([], 0, 10.17)
  assert obj.identity in view.poses
  assert 0. < view.poses[obj.identity].alpha < 1.
  view.update([], 0, 10.25)
  assert not view.poses and not view.transitions


def test_asynchronous_identity_handoff_can_have_older_fresh_timestamp():
  view = WorldPresentation()
  old = car(vehicle=True)
  view.update([old],0,10.)
  new = car(t=9.98)
  view.update([new],0,10.01)
  assert view.pose(new).forward == view.pose(old).forward == 20.
  view.update([new],0,10.07)
  assert view.pose(new).forward == 20.


def test_real_schema_unmarked_model_path_can_recover_world():
  from cereal import messaging
  sm = messages()
  msg = messaging.new_message('modelV2')
  msg.modelV2.position.x = [0.,1.,2.]
  msg.modelV2.position.y = [0.,0.,0.]
  sm['modelV2'] = msg.as_reader().modelV2
  gate = WorldAvailability()
  for i in range(17):
    now = 10+i*.05
    sm.tick(i+2,now)
    active = gate.update(sm,0,now)
  assert active


@pytest.mark.parametrize('change', ['id','jump','lateral_jump','drive','stale','clock'])
def test_discontinuities_never_animate_through_traffic(change):
  view = WorldPresentation()
  view.update([car()],0,10.)
  obj = car(d=21.,t=10.05)
  drive,now = 0,10.05
  if change == 'id':
    obj.identity = ('track',99)
  elif change == 'jump':
    obj.forward = 90.
  elif change == 'lateral_jump':
    obj.right = -4.
  elif change == 'drive':
    drive = 20
  elif change == 'stale':
    now = 11.
    obj.received = now
  else:
    now = 9.
    obj.received = now
  view.update([obj],drive,now)
  assert view.pose(obj).forward == obj.forward
  assert view.pose(obj).right == obj.right


def test_stale_objects_and_unqualified_dots_never_retained():
  view = WorldPresentation()
  obj = car()
  view.update([obj],0,10.)
  view.update([obj],0,10.36)
  assert not view.poses
  obj.radar_avatar = False
  view.update([obj],0,10.)
  assert not view.poses


def test_yaw_shortest_arc_and_bounded_history():
  view = WorldPresentation()
  view.update([car(yaw=179)],0,10.)
  new = car(yaw=-179,t=10.05)
  view.update([new],0,10.05)
  view.update([new],0,10.075)
  assert view.pose(new).yaw == pytest.approx(180.)
  for frame in range(200):
    now = 20+frame*.05
    view.update([car(t=now,track=frame*100+i) for i in range(100)],0,now)
    assert len(view.poses) == MAX_OBJECTS
    assert len(view.transitions) == MAX_OBJECTS
  view.reset()
  assert not view.poses


def test_camera_fallback_stays_stable_after_active_world_loses_model():
  sm,gate = messages(),WorldAvailability()
  for i in range(17):
    now = 10+i*.05
    sm.tick(i+2,now)
    assert gate.update(sm,0,now) == (i >= 15)
  sm.recv_time['modelV2'] = 10.0
  assert not gate.update(sm,0,10.85)
  assert gate.fallback_reason == 'model data unavailable'
  for i in range(17):
    now = 12+i*.05
    sm.tick(i+30,now)
    assert not gate.update(sm,0,now)
  assert not gate.update(sm,50,12.85)
  assert gate.fallback_reason is None
  for i in range(17):
    now = 13+i*.05
    sm.tick(i+60,now)
    assert gate.update(sm,50,now) == (i >= 15)


def test_ui_update_gap_latches_camera_until_new_drive():
  sm,gate = messages(),WorldAvailability()
  for i in range(17):
    now = 10+i*.05
    sm.tick(i+2,now)
    gate.update(sm,0,now)
  assert gate.active
  sm.tick(20,11.5)
  assert not gate.update(sm,0,11.5)
  assert gate.fallback_reason == 'UI update gap'
  for i in range(20):
    now = 11.55+i*.05
    sm.tick(i+21,now)
    assert not gate.update(sm,0,now)


@pytest.mark.parametrize('x,y', [([],[]),([0,0],[0,0]),([0,1],[0,math.nan]),([0],[0])])
def test_invalid_geometry_uses_camera(x,y):
  sm = messages()
  sm['modelV2'].position = NS(x=x,y=y)
  gate = WorldAvailability()
  for i in range(20):
    sm.tick(i+2,10+i*.05)
    assert not gate.update(sm,0,10+i*.05)


@pytest.mark.parametrize('desired', [0.,-1.,math.nan,math.inf])
def test_unavailable_desired_distance_and_standstill_gap_are_hidden(desired):
  assert world_lead_lines(30,0,0,desired,'m',' km/h',1,3.6) == ['30 m  |  0 km/h']


def test_valid_metrics_preserve_units():
  assert world_lead_lines(30,20,15,25,'m',' km/h',1,3.6) == ['30 m  |  72 km/h','2.00 s  |  desired 25 m']
