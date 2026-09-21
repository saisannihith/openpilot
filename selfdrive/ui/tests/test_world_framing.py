"""Keep the device-sized road view readable without cropping the ego avatar."""
import pytest
import pyray as rl
import numpy as np
import math

from openpilot.selfdrive.ui.onroad.tesla_road_renderer import (TeslaRoadRenderer, WORLD_CAMERA_FOVY,
                                                                ego_rear_signal_positions, ego_signal_flash)
from openpilot.selfdrive.ui.onroad.world_scene import display_lane_continuation


@pytest.mark.parametrize('width,height', [(2160,1080),(1440,720),(1920,960)])
def test_landscape_framing_uses_display_and_keeps_near_and_far_visible(width,height):
  world = TeslaRoadRenderer()
  assert world._camera.fovy == WORLD_CAMERA_FOVY == 42.
  old_focal = .5 / math.tan(math.radians(46.) / 2.)
  assert world._focal_factor == pytest.approx(.5 / math.tan(math.radians(WORLD_CAMERA_FOVY) / 2.))
  # Same physical coordinates, but about 13% more visual scale for nearby traffic.
  assert world._focal_factor / old_focal > 1.13
  scale = min(1.,1440/width,810/height)
  world._target_size = (int(width*scale),int(height*scale))
  rect = rl.Rectangle(30,20,width,height)
  front = world.project(0,0,rect)
  rear = world.project(-4.8,0,rect)
  far = world.project(100,0,rect)
  left,right = (world.project(0,y,rect) for y in (-7.1,7.1))
  assert all(p is not None for p in (front,rear,far,left,right))
  assert .50 < (front[1]-rect.y)/height < .70
  assert .80 < (rear[1]-rect.y)/height < .95
  # The lower virtual eye line keeps distant model vehicles on the road below
  # the visual horizon, instead of reading as if they were in the sky.
  assert .18 < (far[1]-rect.y)/height < .30
  assert (right[0]-left[0])/width > .50
  for distance,lateral in ((0,0),(15,2),(50,-3)):
    expected = rl.get_world_to_screen_ex(rl.Vector3(lateral,.02,-distance),world._camera,width,height)
    actual = world.project(distance,lateral,rect)
    assert actual == pytest.approx((rect.x+expected.x,rect.y+expected.y),abs=.001)


def test_lane_continuation_does_not_change_detected_points_or_invent_missing_lanes():
  points = ((0.,1.8),(3.,1.9),(6.,2.1))
  result = display_lane_continuation(points)
  assert result[1:] == points
  assert result[0] == pytest.approx((-8.,1.4))
  for unknown in ((),((0.,1.),),((5.,1.),(10.,2.))):
    assert display_lane_continuation(unknown) is unknown


def test_speed_motion_is_visual_only_and_scales_with_ego_speed():
  world = TeslaRoadRenderer()
  world._advance_motion(10.,0.,True)
  world._advance_motion(11.,0.,True)
  assert world._motion_distance == 0.
  world._advance_motion(12.,10.,True)
  assert world._motion_distance == pytest.approx(0.)
  # A continuous timestamp advances the decorative phase by travelled distance.
  world._advance_motion(12.5,10.,True)
  assert world._motion_distance == pytest.approx(5.)
  world._advance_motion(13.0,20.,False)
  assert world._motion_distance == pytest.approx(5.)


def test_ego_turn_signals_blink_together_with_the_existing_ui_cadence():
  assert ego_signal_flash(True,False,.10) == (True,False)
  assert ego_signal_flash(True,True,.60) == (False,False)
  assert ego_signal_flash(False,True,1.10) == (False,True)


def test_ego_turn_signal_positions_follow_the_carnival_display_yaw():
  anchor = rl.Vector3(0.,0.,4.6)
  left,right = ego_rear_signal_positions(anchor,0.)
  assert (left.x,left.y,left.z) == pytest.approx((-.715,1.13,6.977))
  assert (right.x,right.y,right.z) == pytest.approx((.702,1.13,6.978))
  left,right = ego_rear_signal_positions(anchor,90.)
  assert (left.x,left.z) == pytest.approx((2.377,5.315))
  assert (right.x,right.z) == pytest.approx((2.378,3.898))


@pytest.mark.parametrize('offset',[-7.1,-1.8,1.8,7.1])
def test_ribbons_reach_bottom_and_are_not_shortened_by_offscreen_endpoints(offset):
  world = TeslaRoadRenderer()
  world._target_size = (1440,720)
  rect = rl.Rectangle(0,0,2160,1080)
  points = display_lane_continuation(((0.,offset),(20.,offset),(100.,offset)))
  ribbon = world.project_ribbon(points,.045,rect)
  assert ribbon.shape == (2*len(points),2)
  assert np.isfinite(ribbon).all()
  assert ribbon[:,1].max() > rect.height
  assert world.project(-8,offset,rect) is None
  assert world.project(-8,offset,rect,clip=False) is not None
