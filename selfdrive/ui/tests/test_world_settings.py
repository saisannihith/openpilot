from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.layouts.settings.starpilot import appearance
from openpilot.selfdrive.ui.lib.ui_param_cache import UIParamCache


class Params:
  def __init__(self, view):
    self.view = view
    self.writes = []
    self.bools = {}

  def get_int(self, key, **kwargs):
    return self.view if key == 'CameraView' else kwargs.get('default',0)

  def get_bool(self, key, **kwargs):
    return self.bools.get(key, False)

  def put_bool(self, key, value):
    self.bools[key] = value

  def put_int(self, key, value):
    assert key == 'CameraView'
    self.view = value
    self.writes.append(value)


def layout(monkeypatch, view):
  monkeypatch.setattr(appearance,'AetherSettingsView',lambda *a,**kw:NS(**kw))
  monkeypatch.setattr(appearance,'AppearanceManagerView',lambda *a,**kw:NS(**kw))
  obj = object.__new__(appearance.StarPilotAppearanceLayout)
  raw = Params(view)
  obj._params = UIParamCache(raw,ttl=100.)
  obj._sub_panels = {}
  obj._wire_sub_panels = lambda:None
  obj._build_view()
  return obj,raw


@pytest.mark.parametrize('view',range(5))
def test_toggle_is_in_model_section_and_updates_shared_value_immediately(monkeypatch,view):
  obj,raw = layout(monkeypatch,view)
  row = next(r for r in obj._model_rows if r.id == 'TeslaRoad')
  assert row.visible is None and row.enabled is None
  assert not any(r.id == 'TeslaRoad' for r in obj._system_rows)
  assert 'Tesla Road' not in appearance.CAMERA_VIEWS
  for _ in range(20):
    assert not row.get_state()
    row.set_state(True)
    assert row.get_state() and obj._params.get_int('CameraView') == 5
    row.set_state(False)
    assert not row.get_state() and obj._params.get_int('CameraView') == view
  assert raw.writes == [5,view]*20


def test_persisted_world_mode_can_be_disabled_after_ui_restart(monkeypatch):
  obj,raw = layout(monkeypatch,5)
  row = next(r for r in obj._model_rows if r.id == 'TeslaRoad')
  assert row.get_state()
  row.set_state(False)
  assert raw.view == 2


def test_ambient_and_motion_are_scoped_to_tesla_road(monkeypatch):
  obj,raw = layout(monkeypatch,2)
  rows = {row.id: row for row in obj._model_rows}
  assert rows['TeslaRoadAmbient'].visible() is False
  assert rows['TeslaRoadMotion'].visible() is False
  rows['TeslaRoad'].set_state(True)
  assert rows['TeslaRoadAmbient'].visible() is True
  assert rows['TeslaRoadMotion'].visible() is True
  rows['TeslaRoadAmbient'].set_state(True)
  rows['TeslaRoadMotion'].set_state(True)
  assert raw.bools == {'TeslaRoadAmbient': True, 'TeslaRoadMotion': True}


def test_camera_picker_has_no_world_option_or_none_mismatch(monkeypatch):
  obj,raw = layout(monkeypatch,5)
  captured = []
  def dialog(title,options,current,callback):
    result = NS(selection='Wide',callback=callback)
    captured.append((options,current,result))
    return result
  monkeypatch.setattr(appearance,'MultiOptionDialog',dialog)
  monkeypatch.setattr(appearance.gui_app,'push_widget',lambda *_:None)
  obj._show_camera_view_selector()
  assert captured[0][1] == 'Standard' and 'Tesla Road' not in captured[0][0]
  captured[0][2].callback(appearance.DialogResult.CONFIRM)
  assert raw.view == 3
