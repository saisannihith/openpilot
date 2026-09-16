"""World projection adapter for existing StarPilot display policies.

No planner/lead selection or control parameters are written here.
"""
import math
import time
from itertools import islice

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad.world_scene import lateral_at
from openpilot.selfdrive.ui.onroad.starpilot.path import render_adjacent_lanes, render_path_edges
from openpilot.system.ui.lib.shader_polygon import draw_polygon


def clipped_path(points, end):
  if not points or not math.isfinite(end) or end <= points[0][0]:
    return ()
  result = [(x,y) for x,y in points if x < end]
  y = lateral_at(points,end)
  if y is not None:
    result.append((end,y))
  return tuple(result)


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
  anchor = world.project(distance,y,rect,height=.1)
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

  for lane in scene.lanes:
    draw_polygon(rect,world.project_ribbon(lane,lw,rect),rl.WHITE)
  for boundary in scene.edges:
    draw_polygon(rect,world.project_ribbon(boundary,ew,rect),rl.Color(230,48,48,255))

  for side,(left,right) in enumerate(((scene.lanes[0],scene.lanes[1]),(scene.lanes[2],scene.lanes[3]))):
    a,b = [],[]
    for x,y in left:
      other = lateral_at(right,x)
      if other is None or not .5 < other-y < 6.:
        continue
      l,r = world.project(x,y,rect),world.project(x,other,rect)
      if l is not None and r is not None:
        a.append(l)
        b.append(r)
    renderer._adjacent_path_vertices[side] = np.asarray(a+b[::-1],np.float32).reshape(-1,2)
  render_adjacent_lanes(renderer,fresh=fresh)

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
