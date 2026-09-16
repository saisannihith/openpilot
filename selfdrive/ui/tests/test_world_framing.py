"""Keep the device-sized road view readable without cropping the ego avatar."""
import pytest
import pyray as rl
import numpy as np

from openpilot.selfdrive.ui.onroad.tesla_road_renderer import TeslaRoadRenderer
from openpilot.selfdrive.ui.onroad.world_scene import display_lane_continuation


@pytest.mark.parametrize('width,height', [(2160,1080),(1440,720),(1920,960)])
def test_landscape_framing_uses_display_and_keeps_near_and_far_visible(width,height):
  world = TeslaRoadRenderer()
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
  assert .05 < (far[1]-rect.y)/height < .20
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
