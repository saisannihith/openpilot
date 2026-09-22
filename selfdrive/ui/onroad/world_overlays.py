"""World projection adapter for existing StarPilot display policies.

No planner/lead selection or control parameters are written here.
"""
import math
import time
from itertools import islice

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad.world_scene import MAX_POINTS, display_lane_continuation, lateral_at, road_surface_segments
from openpilot.selfdrive.ui.onroad.starpilot.path import render_path_edges
from openpilot.system.ui.lib.shader_polygon import Gradient, draw_polygon


ROAD_ASPHALT_GRADIENT = Gradient(
  start=(0., 1.), end=(0., .16),
  colors=[rl.Color(7, 14, 29, 255), rl.Color(9, 23, 48, 255), rl.Color(12, 34, 66, 255)],
  stops=[0., .65, 1.],
)
ROAD_REFLECTION = rl.Color(40, 109, 196, 76)
ROAD_REFLECTION_HOT = rl.Color(131, 83, 224, 88)
ROAD_EDGE_OUTER_GLOW = rl.Color(96, 10, 49, 70)
ROAD_EDGE_INNER_GLOW = rl.Color(208, 25, 87, 130)
ROAD_EDGE_CORE = rl.Color(255, 99, 133, 255)


def clipped_path(points, end):
  if not points or not math.isfinite(end) or end <= points[0][0]:
    return ()
  result = [(x,y) for x,y in points if x < end]
  y = lateral_at(points,end)
  if y is not None:
    result.append((end,y))
  return tuple(result)


def project_road_surface(world, points, rect):
  """Project a measured road-edge span without filling unknown topology."""
  left, right = [], []
  for distance, left_y, right_y in points:
    a = world.project(distance, left_y, rect, height=.003, clip=False)
    b = world.project(distance, right_y, rect, height=.003, clip=False)
    if a is None or b is None:
      return np.empty((0,2),np.float32)
    left.append(a)
    right.append(b)
  return np.asarray((*left,*reversed(right)),np.float32) if len(left) >= 2 else np.empty((0,2),np.float32)


def project_road_slice(world, points, start, end, rect):
  """Project a short, live-edge-bounded asphalt section for visual sheen."""
  if not math.isfinite(start) or not math.isfinite(end) or end <= start:
    return np.empty((0,2),np.float32)
  left = tuple((x, y) for x, y, _ in points)
  right = tuple((x, y) for x, _, y in points)
  start_left, start_right = lateral_at(left,start), lateral_at(right,start)
  end_left, end_right = lateral_at(left,end), lateral_at(right,end)
  if None in (start_left,start_right,end_left,end_right):
    return np.empty((0,2),np.float32)
  return project_road_surface(world,((start,start_left,start_right),(end,end_left,end_right)),rect)


def render_road_reflections(world, surface, rect):
  """Subtle moving light on known asphalt, never on inferred road space."""
  near,far = max(1.,surface[0][0]),min(90.,surface[-1][0])
  if far-near < 5.:
    return
  phase = getattr(world,'_motion_distance',0.)
  for index in range(6):
    start = near+(index*13.-phase*1.15)%(far-near)
    end = min(far,start+1.2+start*.035)
    color = ROAD_REFLECTION_HOT if index % 3 == 0 else ROAD_REFLECTION
    draw_polygon(rect,project_road_slice(world,surface,start,end,rect),color)


def stop_anchor(world, rect, state, distance):
  now = time.monotonic()
  if not world.scene.fresh(state.sm,'starpilotPlan',state.started_frame,now):
    return None
  if not world.scene.fresh(state.sm,'modelV2',state.started_frame,now):
    return None
  if not math.isfinite(distance) or distance < 0:
    return None
  y = lateral_at(world.scene.path,distance)
  if y is None:
    return None
  anchor = world.project(distance,y,rect,height=world.scene.road_height(distance)+.1)
  if anchor is None or anchor[1] < rect.y+105:
    return None
  # Leave room for the octagon and its optional distance label.
  return (max(rect.x+50,min(rect.x+rect.width-50,anchor[0])),anchor[1])


def render_road(renderer, world, rect, state):
  sm, scene = state.sm, world.scene
  now = time.monotonic()
  def fresh(key):
    return scene.fresh(sm,key,state.started_frame,now)
  renderer._rect = rect
  renderer._transform_dirty = True  # Camera mode must rebuild its own projection.
  empty = np.empty((0,2),np.float32)
  renderer._path.projected_points = empty
  renderer._track_edge_vertices = empty
  renderer._adjacent_path_vertices = [empty,empty]
  if not scene.path:
    return

  params = renderer._params
  renderer._is_metric = state.is_metric
  if getattr(sm,'updated',{}).get('carParams',False):
    renderer._longitudinal_control = sm['carParams'].openpilotLongitudinalControl
  custom = params.get_bool('ModelUI',default=True)

  def width(key, default, converter, fallback):
    changed,value = renderer._param_float_changed(key,default) if custom else (False,default)
    value = converter(value) if changed else fallback
    return float(np.clip(value,0,4)) if math.isfinite(value) else fallback

  # Use the same conversions/defaults as ModelRenderer._update_model.
  from openpilot.selfdrive.ui.onroad.model_renderer import (DEFAULT_PATH_WIDTH, DEFAULT_LANE_LINES_WIDTH,
                                                           DEFAULT_ROAD_EDGES_WIDTH, DEFAULT_PATH_EDGE_WIDTH)
  pw = width('PathWidth',DEFAULT_PATH_WIDTH,renderer._path_width_to_half_m,.9)
  lw = width('LaneLinesWidth',DEFAULT_LANE_LINES_WIDTH,renderer._small_distance_to_half_m,.025)
  ew = width('RoadEdgesWidth',DEFAULT_ROAD_EDGES_WIDTH,renderer._small_distance_to_half_m,.025)
  changed,edge = renderer._param_float_changed('PathEdgeWidth',DEFAULT_PATH_EDGE_WIDTH) if custom else (False,0)
  edge = float(np.clip(edge/100,0,1)) if changed and math.isfinite(edge) else 0.
  if custom and params.get_bool('DynamicPathWidth'):
    from openpilot.selfdrive.ui.ui_state import UIStatus
    pw *= 1. if state.status == UIStatus.ENGAGED else (.75 if state.always_on_lateral_active else .5)

  path = scene.path
  if fresh('radarState') and sm['radarState'].leadOne.status:
    distance = float(sm['radarState'].leadOne.dRel)
    if math.isfinite(distance) and distance > 0:
      path = clipped_path(path,distance*2-min(distance*.7,10))
  renderer._path.projected_points = world.project_ribbon(path,pw*(1-edge),rect)
  renderer._track_edge_vertices = world.project_ribbon(path,pw,rect)

  # Use the same bounded near-field tangent as the edge strokes so the measured
  # road surface reaches the viewport instead of ending at the ego origin.
  display_edges = tuple(display_lane_continuation(edge) for edge in scene.edges)
  # A road polygon exists only where both confident model edges agree on a
  # plausible span. Everything outside it remains true OLED black.
  for surface in road_surface_segments(*display_edges):
    # A shader gradient gives the live polygon a dark-blue asphalt depth,
    # without ever inventing road outside its confident paired edges.
    draw_polygon(rect,project_road_surface(world,surface,rect),gradient=ROAD_ASPHALT_GRADIENT)
    render_road_reflections(world,surface,rect)
  display_lanes = tuple(display_lane_continuation(lane) for lane in scene.lanes)
  for lane in display_lanes:
    draw_polygon(rect,world.project_ribbon(lane,lw,rect),rl.Color(236,242,247,255))
  for display in display_edges:
    # Layered shoulder light reads as a road boundary at a glance while each
    # stroke still follows only the fresh model road-edge coordinates.
    halo = min(.22,max(.11,ew*5.))
    draw_polygon(rect,world.project_ribbon(display,halo*2.4,rect),ROAD_EDGE_OUTER_GLOW)
    draw_polygon(rect,world.project_ribbon(display,halo,rect),ROAD_EDGE_INNER_GLOW)
    draw_polygon(rect,world.project_ribbon(display,ew,rect),ROAD_EDGE_CORE)

  # Geometry does not establish adjacent traffic direction or lane availability.
  # Leave neighboring road surfaces black, not red/green "unsafe/safe" lanes.

  renderer._experimental_mode = fresh('selfdriveState') and sm['selfdriveState'].experimentalMode
  renderer._use_rainbow = params.get_bool('RainbowPath')
  renderer._use_accel_path = params.get_bool('AccelerationPath',default=True)
  if renderer._use_rainbow and fresh('carState'):
    renderer._rainbow_path.update(max(0,sm['carState'].vEgo))
  model = sm['modelV2']
  acceleration = getattr(getattr(model,'acceleration',None),'x',())
  renderer._acceleration_x = np.nan_to_num(np.fromiter(islice(acceleration,33),np.float32),nan=0,posinf=0,neginf=0)
  renderer._update_experimental_gradient()
  if fresh('longitudinalPlan'):
    renderer._draw_path(sm)
  else:
    draw_polygon(rect,renderer._path.projected_points,rl.Color(40,149,246,180))
  if edge > 0:
    render_path_edges(renderer)


def configure_world_intent(renderer, world, rect, state):
  """Pass existing path settings into the 3D renderer without screen-space paint.

  Tesla Road owns the measured road mesh. This adapter keeps StarPilot's live
  path controls working while leaving stop/lead indicators to their factual HUD
  renderers, where they already have source freshness checks.
  """
  sm, scene = state.sm, world.scene
  now = time.monotonic()
  if not scene.path:
    world.configure_road_intent((), .85, 0., False, ())
    return
  params = renderer._params
  custom = params.get_bool('ModelUI', default=True)

  def width(key, default, converter, fallback):
    changed, value = renderer._param_float_changed(key, default) if custom else (False, default)
    value = converter(value) if changed else fallback
    return float(np.clip(value, 0, 4)) if math.isfinite(value) else fallback

  from openpilot.selfdrive.ui.onroad.model_renderer import DEFAULT_PATH_WIDTH, DEFAULT_PATH_EDGE_WIDTH
  path_width = width('PathWidth', DEFAULT_PATH_WIDTH, renderer._path_width_to_half_m, .9)
  changed, edge = renderer._param_float_changed('PathEdgeWidth', DEFAULT_PATH_EDGE_WIDTH) if custom else (False, 0)
  edge = float(np.clip(edge / 100, 0, 1)) if changed and math.isfinite(edge) else 0.
  if custom and params.get_bool('DynamicPathWidth'):
    from openpilot.selfdrive.ui.ui_state import UIStatus
    path_width *= 1. if state.status == UIStatus.ENGAGED else (.75 if state.always_on_lateral_active else .5)

  path = scene.path
  if scene.fresh(sm, 'radarState', state.started_frame, now) and sm['radarState'].leadOne.status:
    distance = float(sm['radarState'].leadOne.dRel)
    if math.isfinite(distance) and distance > 0:
      path = clipped_path(path, distance * 2 - min(distance * .7, 10))
  model = sm['modelV2']
  acceleration = getattr(getattr(model, 'acceleration', None), 'x', ())
  acceleration = tuple(np.nan_to_num(np.fromiter(islice(acceleration, MAX_POINTS), np.float32), nan=0, posinf=0, neginf=0))
  rainbow = params.get_bool('RainbowPath')
  world.configure_road_intent(path, path_width, edge, rainbow, acceleration if params.get_bool('AccelerationPath', default=True) else ())
