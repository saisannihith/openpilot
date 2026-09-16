"""Exercise the actual shared lead overlay, not a duplicate UI implementation."""
from types import SimpleNamespace as NS

import pytest
import pyray as rl

from openpilot.selfdrive.ui.onroad import model_renderer as module
from openpilot.selfdrive.ui.onroad.world_scene import WorldScene


class Messages(dict):
  pass


def setup_overlay(monkeypatch):
  lead = NS(status=True, dRel=30., yRel=-1., vLead=20., vRel=-3., modelProb=.99)
  sm = Messages(radarState=NS(leadOne=lead, leadTwo=NS(status=False)),
                carState=NS(vEgo=15.), starpilotPlan=NS(desiredFollowDistance=25.),
                carParams=NS(openpilotLongitudinalControl=True))
  sm.valid = {'radarState': True}
  sm.alive = {'radarState': True}
  sm.recv_frame = {'radarState': 2}
  sm.recv_time = {'radarState': 10.}
  sm.updated = {'carParams': True}
  state = NS(sm=sm, started_frame=0, is_metric=True, starpilot_toggles={})
  monkeypatch.setattr(module, 'ui_state', state)
  monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
  renderer = object.__new__(module.ModelRenderer)
  renderer._params = NS(get_bool=lambda k: k == 'LeadInfo', get_int=lambda k: 50, get=lambda k: b'0')
  renderer._longitudinal_control = False
  world = NS(scene=WorldScene(), lead_anchor=lambda i, rect: (500,500) if i == 0 else None)
  return renderer, world, state, lead


def test_world_uses_selected_lead_and_shared_icons(monkeypatch):
  renderer, world, state, lead = setup_overlay(monkeypatch)
  triangles, metrics = [], []
  monkeypatch.setattr(module.rl, 'draw_triangle_fan', lambda *args: triangles.append(args))
  monkeypatch.setattr(renderer, '_draw_lead_metrics', lambda *args, **kwargs: metrics.append((args,kwargs)))
  renderer.render_world_leads(rl.Rectangle(0,0,1440,810), world)
  assert len(triangles) == 2
  assert len(metrics) == 1 and metrics[0][0][2] is lead
  assert metrics[0][1] == {'above': True}
  assert renderer._longitudinal_control
  assert renderer._lead_vehicles[0].chevron[1] == (500,500)
  state.sm.recv_time['radarState'] = 9.
  renderer.render_world_leads(rl.Rectangle(0,0,1440,810), world)
  assert len(metrics) == 1 and not renderer._lead_vehicles[0].chevron


def test_secondary_lead_never_gets_an_icon_label_or_primary_fallback(monkeypatch):
  renderer, world, state, lead = setup_overlay(monkeypatch)
  state.sm['radarState'].leadTwo = NS(status=True,dRel=50.,vLead=25.,vRel=2.,modelProb=.99)
  world.lead_anchor = lambda i, rect: (500+i*200,500)
  triangles,metrics = [],[]
  monkeypatch.setattr(module.rl,'draw_triangle_fan',lambda *args: triangles.append(args))
  monkeypatch.setattr(renderer,'_draw_lead_metrics',lambda *args,**kwargs: metrics.append(args))
  renderer.render_world_leads(rl.Rectangle(0,0,1440,810),world)
  assert len(metrics) == 1 and metrics[0][2] is lead
  assert len(triangles) == 2 and not renderer._lead_vehicles[1].chevron
  lead.status = False
  renderer.render_world_leads(rl.Rectangle(0,0,1440,810),world)
  assert len(metrics) == 1 and len(triangles) == 2


@pytest.mark.parametrize('invalid', ['hidden', 'nan', 'no_anchor', 'inactive'])
def test_invalid_or_hidden_lead_is_never_annotated(monkeypatch, invalid):
  renderer, world, state, lead = setup_overlay(monkeypatch)
  calls = []
  monkeypatch.setattr(module.rl, 'draw_triangle_fan', lambda *args: calls.append(args))
  monkeypatch.setattr(renderer, '_draw_lead_metrics', lambda *args, **kwargs: calls.append(args))
  if invalid == 'hidden':
    renderer._params.get_bool = lambda key: True
  elif invalid == 'nan':
    lead.vLead = float('nan')
  elif invalid == 'no_anchor':
    world.lead_anchor = lambda *args: None
  else:
    lead.status = False
  renderer.render_world_leads(rl.Rectangle(0,0,1440,810),world)
  assert not calls


@pytest.mark.parametrize('metric,si,expected', [
  (True,False,['30 m (Desired: 25)','72 km/h','2.00 seconds']),
  (False,False,['98 ft (Desired: 82)','45 mph','2.00 seconds']),
  (False,True,['30 m (Desired: 25)','20 m/s','2.00 seconds']),
])
def test_shared_metrics_keep_absolute_speed_and_units(monkeypatch, metric, si, expected):
  renderer, world, state, lead = setup_overlay(monkeypatch)
  state.is_metric, state.starpilot_toggles = metric, {'UseSiMetrics':si}
  renderer._longitudinal_control = True
  renderer._rect = rl.Rectangle(100,50,1200,700)
  renderer._lead_text_rects, renderer._adjacent_lead_text_rects = [], []
  from openpilot.selfdrive.ui.onroad.starpilot import path
  from openpilot.system.ui.lib import application
  calls = []
  monkeypatch.setattr(application.gui_app, 'font', lambda *_args: None)
  monkeypatch.setattr(module, 'measure_text_cached', lambda font,text,size: NS(x=len(text)*16,y=36))
  monkeypatch.setattr(path, '_draw_text_with_outline', lambda text,*args: calls.append((text,args)))
  renderer._draw_lead_metrics(False, [(515,488),(500,500),(485,488)], lead, above=True)
  assert [text for text,args in calls] == expected
  assert all(100 <= args[0] < 1300 and 50 <= args[1] < 488 for text,args in calls)
