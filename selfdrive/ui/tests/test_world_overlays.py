import math
from types import SimpleNamespace as NS

import numpy as np
import pyray as rl
import pytest

from openpilot.selfdrive.ui.onroad import world_overlays as wo
from openpilot.selfdrive.ui.onroad import model_renderer as mr
from openpilot.selfdrive.ui.onroad.world_scene import WorldScene
from openpilot.selfdrive.ui.onroad.starpilot import path, stopping_point


class Messages(dict):
  pass


def fixture(monkeypatch):
  sm = Messages(starpilotPlan=NS(redLight=True,forcingStopLength=20.,laneWidthLeft=3.5,laneWidthRight=3.5),
                carState=NS(standstill=False,leftBlindspot=True,rightBlindspot=False,vEgo=20.),
                modelV2=NS(acceleration=NS(x=[0.]*33)),
                radarState=NS(leadOne=NS(status=False)),selfdriveState=NS(experimentalMode=False),
                longitudinalPlan=NS(allowThrottle=True))
  sm.valid,sm.alive,sm.recv_frame,sm.recv_time = (dict.fromkeys(sm,True),dict.fromkeys(sm,True),
                                               dict.fromkeys(sm,2),dict.fromkeys(sm,10.))
  state = NS(sm=sm,started_frame=0,is_metric=True,starpilot_toggles={},status=2,always_on_lateral_active=False)
  scene = WorldScene()
  scene.path = ((0.,0.),(20.,0.),(40.,4.))
  scene.lanes = tuple(tuple((x,y+offset) for x,y in scene.path) for offset in (-5.4,-1.8,1.8,5.4))
  scene.edges = ()
  world = NS(scene=scene,project=lambda d,y,rect,**kw: (400+y*20,600-d*5))
  settings = {'ModelUI':True,'ShowStoppingPoint':True,'ShowStoppingPointMetrics':True}
  params = NS(get_bool=lambda k,**kw: settings.get(k,kw.get('default',False)),
              get=lambda k,**kw: str(settings[k]) if k in settings else None,
              get_float=lambda k: float(settings.get(k,3.5)))
  state.ui_params = params
  monkeypatch.setattr(wo.time,'monotonic',lambda:10.)
  monkeypatch.setattr(path,'ui_state',state)
  monkeypatch.setattr(stopping_point,'ui_state',state)
  return state,world,params,settings


@pytest.mark.parametrize('bad',[math.nan,math.inf,-1,100])
def test_stop_projection_rejects_invalid_or_outside_path(monkeypatch,bad):
  state,world,_,_ = fixture(monkeypatch)
  assert wo.stop_anchor(world,rl.Rectangle(0,0,1000,700),state,bad) is None


def test_stop_projection_uses_planner_distance_and_rejects_expired_geometry(monkeypatch):
  state,world,_,_ = fixture(monkeypatch)
  rect = rl.Rectangle(0,0,1000,700)
  assert wo.stop_anchor(world,rect,state,20) == (400,500)
  state.sm.recv_time['starpilotPlan'] = 8.
  assert wo.stop_anchor(world,rect,state,20) is None
  state.sm.recv_time['starpilotPlan'] = 10.
  state.sm.recv_time['modelV2'] = 8.
  assert wo.stop_anchor(world,rect,state,20) is None


@pytest.mark.parametrize('enabled,red,expected',[(True,True,1),(False,True,0),(True,False,0)])
def test_existing_stop_toggle_and_signal_control_world_marker(monkeypatch,enabled,red,expected):
  state,world,params,settings = fixture(monkeypatch)
  settings['ShowStoppingPoint'] = enabled
  state.sm['starpilotPlan'].redLight = red
  calls = []
  monkeypatch.setattr(stopping_point.rl,'draw_poly',lambda *args:calls.append(args))
  monkeypatch.setattr(stopping_point,'_draw_poly_outline',lambda *args:None)
  monkeypatch.setattr(stopping_point.rl,'draw_text_ex',lambda *args:None)
  monkeypatch.setattr(stopping_point,'measure_text_cached',lambda *args:NS(x=40,y=18))
  occupied = stopping_point.render_stopping_point(NS(),None,project_stop=lambda d:(400,500))
  assert len(calls) == expected
  assert (occupied is not None) == bool(expected)


def test_adjacent_and_blindspot_toggles_and_stale_input(monkeypatch):
  state,world,params,settings = fixture(monkeypatch)
  renderer = NS(_params=params,_rect=rl.Rectangle(0,0,1000,700),
                _adjacent_path_vertices=[np.ones((4,2),np.float32)]*2)
  calls = []
  monkeypatch.setattr(path.gui_app,'font',lambda *args:None)
  monkeypatch.setattr(path,'draw_polygon',lambda *args,**kw:calls.append(kw['gradient'].colors[0]))
  def fresh(k):
    return world.scene.fresh(state.sm,k,0,10.)
  path.render_adjacent_lanes(renderer,fresh=fresh)
  assert not calls
  settings['BlindSpotPath'] = True
  path.render_adjacent_lanes(renderer,fresh=fresh)
  assert len(calls) == 1 and calls[0].r > calls[0].g
  calls.clear()
  state.sm.recv_time['carState'] = 9.
  path.render_adjacent_lanes(renderer,fresh=fresh)
  assert not calls
  settings['AdjacentPath'] = True
  path.render_adjacent_lanes(renderer,fresh=fresh)
  assert len(calls) == 2 and all(c.g > c.r for c in calls)


def test_world_path_styles_use_existing_methods_and_reset_camera_projection(monkeypatch):
  state,world,params,settings = fixture(monkeypatch)
  renderer = object.__new__(mr.ModelRenderer)
  renderer._params = params
  renderer._path = NS(projected_points=None)
  renderer._rainbow_path = NS(update=lambda v:None)
  calls = []
  renderer._update_experimental_gradient = lambda:calls.append('gradient')
  renderer._draw_path = lambda sm:calls.append('shared_path')
  monkeypatch.setattr(wo,'draw_polygon',lambda *args,**kwargs:None)
  monkeypatch.setattr(wo,'render_path_edges',lambda *args:calls.append('edges'))
  widths = []
  world.project_ribbon = lambda points,w,rect: widths.append(w) or np.zeros((6,2),np.float32)
  settings.update(PathWidth='2.8',PathEdgeWidth='30',RainbowPath=True,AccelerationPath=True)
  from cereal import messaging
  message = messaging.new_message('modelV2')
  message.modelV2.acceleration.x = [0.,.2,float('nan')]
  state.sm['modelV2'] = message.as_reader().modelV2
  wo.render_road(renderer,world,rl.Rectangle(0,0,1000,700),state)
  assert widths[0] == pytest.approx(.98) and widths[1] == pytest.approx(1.4)
  assert calls == ['gradient','shared_path','edges']
  assert renderer._use_rainbow and renderer._use_accel_path and renderer._transform_dirty
  assert renderer._acceleration_x.tolist() == pytest.approx([0.,.2,0.])
  assert len(renderer._adjacent_path_vertices) == 2
  assert all(vertices.size == 0 for vertices in renderer._adjacent_path_vertices)
  scene = world.scene
  scene.path = ()
  wo.render_road(renderer,world,rl.Rectangle(0,0,1000,700),state)
  assert renderer._path.projected_points.size == 0


def test_measured_road_surface_precedes_lanes_and_boundaries(monkeypatch):
  state,world,params,_ = fixture(monkeypatch)
  world.scene.edges = (((0.,-7.),(20.,-7.),(40.,-3.)), ((0.,7.),(20.,7.),(40.,11.)))
  renderer = object.__new__(mr.ModelRenderer)
  renderer._params = params
  renderer._path = NS(projected_points=None)
  renderer._rainbow_path = NS(update=lambda _:None)
  renderer._update_experimental_gradient = lambda:None
  renderer._draw_path = lambda _:None
  world.project_ribbon = lambda *_: np.zeros((6,2),np.float32)
  calls = []
  monkeypatch.setattr(wo,'draw_polygon',lambda _,points,color=None,**kwargs: calls.append((points,color or kwargs.get('gradient'))))
  wo.render_road(renderer,world,rl.Rectangle(0,0,1000,700),state)
  assert calls[0][1] == wo.ROAD_ASPHALT_GRADIENT
  assert calls[0][0].shape == (8,2)
  assert any(color == wo.ROAD_EDGE_CORE for _,color in calls)


def test_reflections_are_clipped_to_live_paired_road_edges(monkeypatch):
  _,world,_,_ = fixture(monkeypatch)
  surface = ((0.,-7.,7.),(20.,-6.,8.),(40.,-3.,11.))
  points = wo.project_road_slice(world,surface,4.,12.,rl.Rectangle(0,0,1000,700))
  assert points.shape == (4,2)
  assert wo.project_road_slice(world,surface,-1.,12.,rl.Rectangle(0,0,1000,700)).size == 0


def test_primary_clipping_does_not_jump_to_secondary_lead():
  assert wo.clipped_path(((0,0),(10,0),(20,4)),15) == ((0,0),(10,0),(15,2))
