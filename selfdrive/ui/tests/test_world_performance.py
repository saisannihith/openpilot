from types import SimpleNamespace as NS
import pytest

from openpilot.selfdrive.ui.onroad import world_presentation as wp
from openpilot.selfdrive.ui.onroad.world_scene import SceneObject
from openpilot.selfdrive.ui.tests.test_world_scene import messages


def test_unchanged_model_geometry_is_validated_once_but_freshness_every_time(monkeypatch):
  sm, gate = messages(), wp.WorldAvailability()
  calls = []
  original = wp.polyline
  monkeypatch.setattr(wp,'polyline',lambda p: calls.append(p) or original(p))
  sm.tick(2,10.)
  for now in (10.,10.05,10.1,10.2):
    gate.update(sm,0,now)
  assert len(calls) == 1
  assert not gate.update(sm,0,10.4)
  sm.tick(3,10.5)
  gate.update(sm,0,10.5)
  assert len(calls) == 2


def test_interpolation_reuses_storage_without_losing_new_model_heading():
  p = wp.WorldPresentation()
  obj = SceneObject(('lead',0),20.,0.,True,10.,('track',1))
  objects = (obj,)
  p.update(objects,0,10.,1)
  container, poses = p.transitions, p.poses
  pose = p.pose(obj)
  p.update(objects,0,10.025,1)
  assert container is p.transitions and poses is p.poses and pose is p.pose(obj)
  obj.yaw = 10.
  p.update(objects,0,10.05,2)
  p.update(objects,0,10.11,2)
  assert p.pose(obj).yaw == pytest.approx(10.)
  p.update(objects,0,10.5,2)
  assert not p.poses


def test_resolution_ignores_cold_spikes_and_requires_sustained_load():
  q = wp.RenderQuality()
  for i in range(20):
    q.observe(100.,i*.05)
  assert q.level == 0
  for i in range(11):
    q.observe(40.,1+i*.05)
  assert q.level == 0
  q.observe(40.,1.6)
  assert q.level == 1
  for i in range(80):
    q.observe(40.,1.65+i*.05)
  assert q.level == 1
  q.observe(40.,6.7)
  assert q.level == 2
  for i in range(220):
    q.observe(10.,6.8+i*.05)
  assert q.level == 1
  q.enabled = False
  assert q.scale == 1.


def test_fresh_empty_tracks_are_not_reported_as_sensor_failure():
  sm = messages()
  sm['liveTracks'] = NS(points=[],errors=NS(canError=False,radarFault=False,wrongConfig=False,radarUnavailableTemporary=False))
  sm.tick(2,10.)
  assert wp.degraded_world_text(sm,0,10.,True,False) == ''
  sm['liveTracks'].errors.radarFault = True
  assert wp.degraded_world_text(sm,0,10.,True,False) == 'Radar visualization unavailable'
  assert 'camera' in wp.degraded_world_text(sm,0,10.,False,False)
  assert 'rendering' in wp.degraded_world_text(sm,0,10.,False,True)
  sm.recv_time['radarState'] = 8.
  assert wp.degraded_world_text(sm,0,10.,True,False) == 'Lead data unavailable'


def test_hud_reservations_cover_speed_without_changing_driver_controls(monkeypatch):
  from openpilot.selfdrive.ui.onroad import hud_renderer as hud
  import pyray as rl
  renderer = object.__new__(hud.HudRenderer)
  renderer._font_bold = None
  renderer.draw_current_speed = renderer.draw_set_speed = renderer.draw_exp_button = True
  renderer._navigation_card = NS(_valid=False,_interactive_rect=rl.Rectangle())
  monkeypatch.setattr(hud,'ui_state',NS(starpilot_toggles={}))
  monkeypatch.setattr(hud,'measure_text_cached',lambda *_args:NS(x=280))
  rects = renderer.world_exclusions(rl.Rectangle(20,30,2160,1080))
  assert len(rects) == 3
  assert rl.check_collision_point_rec(rl.Vector2(1100,180),rects[0])
  renderer.world_mode = True
  world_rects = renderer.world_exclusions(rl.Rectangle(20,30,2160,1080))
  assert not rl.check_collision_point_rec(rl.Vector2(1100,180),world_rects[0])
  assert renderer._speed_center(rl.Rectangle(0,0,700,960)) == 350.
