from types import SimpleNamespace

import pyray as rl
import pytest

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.scroll_panel2 import ScrollState
from openpilot.system.ui.widgets.scroller import _Scroller


class FakePanel:
  def __init__(self, offset=0.0, state=ScrollState.STEADY):
    self.offset = offset
    self.state = state

  def get_offset(self):
    return self.offset

  def set_offset(self, offset):
    self.offset = offset

  def set_enabled(self, enabled):
    self.enabled = enabled

  def update(self, rect, content_size):
    self.content_size = content_size


@pytest.fixture
def scroller(monkeypatch):
  monkeypatch.setattr(gui_app, "_target_fps", 60)
  monkeypatch.setattr(gui_app, "texture", lambda *args: rl.Texture())
  scroller = _Scroller([], snap_items=True, spacing=0, pad=0, scroll_indicator=False, edge_shadows=False)
  scroller.set_rect(rl.Rectangle(0, 0, 536, 240))
  scroller.scroll_panel = FakePanel()
  return scroller


def test_programmatic_scroll_tracks_elapsed_time(monkeypatch, scroller):
  positions = []
  for fps in (20, 60):
    monkeypatch.setattr(rl, "get_frame_time", lambda fps=fps: 1 / fps)
    scroller.scroll_panel.set_offset(0)
    scroller._scrolling_to_filter.x = 0
    scroller._scrolling_to = (-500, False)
    for _ in range(fps // 2):
      scroller._update_state()
    positions.append(scroller.scroll_panel.get_offset())

  assert positions[0] == pytest.approx(positions[1], abs=0.01)
  assert -500 < positions[0] < -450


def test_long_frame_scroll_step_is_bounded(monkeypatch, scroller):
  monkeypatch.setattr(rl, "get_frame_time", lambda: 10.0)
  scroller._scrolling_to_filter.x = -1072
  scroller._scrolling_to = (-536, False)
  scroller._update_state()
  assert -1072 < scroller.scroll_panel.get_offset() < -536
  assert scroller.is_auto_scrolling


@pytest.mark.parametrize("fps", [20, 60])
def test_snap_settles_on_page_at_different_frame_rates(monkeypatch, scroller, fps):
  monkeypatch.setattr(rl, "get_frame_time", lambda: 1 / fps)
  pages = [SimpleNamespace(rect=rl.Rectangle(0, 0, 536, 240)) for _ in range(3)]
  scroller.scroll_panel.set_offset(-600)
  for _ in range(fps * 3):
    scroller._update_state()
    scroller._get_scroll(pages, 1608)
  assert scroller.scroll_panel.get_offset() == pytest.approx(-536, abs=1)


def test_snap_waits_for_manual_drag(monkeypatch, scroller):
  monkeypatch.setattr(rl, "get_frame_time", lambda: 1 / 60)
  pages = [SimpleNamespace(rect=rl.Rectangle(0, 0, 536, 240)) for _ in range(3)]
  scroller.scroll_panel = FakePanel(-600, ScrollState.MANUAL_SCROLL)
  scroller._update_state()
  scroller._get_scroll(pages, 1608)
  assert scroller.scroll_panel.get_offset() == -600
