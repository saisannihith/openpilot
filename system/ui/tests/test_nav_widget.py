import pyray as rl
import pytest

from openpilot.common.filter_simple import BounceFilter
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.nav_widget import NavBar, NavWidget, NAV_BAR_MARGIN, NAV_BAR_HEIGHT


class NavScreen(NavWidget):
  def _render(self, _):
    pass


@pytest.fixture
def screen(monkeypatch):
  monkeypatch.setattr(rl, "draw_rectangle_rec", lambda *_: None)
  monkeypatch.setattr(rl, "get_time", lambda: 10.0)
  monkeypatch.setattr(rl, "get_frame_time", lambda: 1 / 60)
  monkeypatch.setattr(gui_app, "_target_fps", 60)
  monkeypatch.setattr(gui_app, "_show_touches", False)
  monkeypatch.setattr(gui_app, "_mouse_events", [])
  screen = NavScreen()
  screen.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
  monkeypatch.setattr(screen._nav_bar, "render", lambda: None)
  return screen


@pytest.mark.parametrize("fps", [20, 30, 60])
def test_dismiss_animation_duration_tracks_elapsed_time(monkeypatch, screen, fps):
  monkeypatch.setattr(rl, "get_frame_time", lambda: 1 / fps)
  popped, dismissed, backed = [], [], []
  monkeypatch.setattr(gui_app, "pop_widget", lambda: popped.append(True))
  screen.set_back_callback(lambda: backed.append(True))
  screen.dismiss(lambda: dismissed.append(True))
  for frames in range(1, fps * 3):
    screen.render(screen.rect)
    if popped:
      break

  assert frames / fps == pytest.approx(13 / 60, abs=1 / fps)
  assert popped == dismissed == [True]
  assert not backed


def test_show_animation_retains_sixty_fps_motion(screen):
  reference = BounceFilter(gui_app.height, 0.1, 1 / 60, bounce=1)
  screen.show_event()
  for _ in range(20):
    reference.update(0.0)
    screen._update_state()
    assert screen._y_pos_filter.x == pytest.approx(reference.x)
    assert screen._y_pos_filter.velocity.x == pytest.approx(reference.velocity.x)


@pytest.mark.parametrize("fps", [20, 30, 60])
def test_navigation_bar_fade_tracks_elapsed_time(monkeypatch, screen, fps):
  monkeypatch.setattr(rl, "get_frame_time", lambda: 1 / fps)
  monkeypatch.setattr(rl, "draw_rectangle_rounded", lambda *_: None)
  monkeypatch.setattr(rl, "draw_rectangle_rounded_lines_ex", lambda *_: None)
  bar = NavBar()
  bar.set_alpha(0.0)
  for _ in range(fps // 2):
    bar._render(bar.rect)
  assert bar._alpha_filter.x == pytest.approx((1 - bar._alpha_filter.alpha) ** 30)


@pytest.mark.parametrize("fps", [20, 30, 60])
def test_navigation_bar_slide_tracks_elapsed_time(monkeypatch, screen, fps):
  monkeypatch.setattr(rl, "get_frame_time", lambda: 1 / fps)
  screen._nav_bar_y_filter.x = -NAV_BAR_MARGIN - NAV_BAR_HEIGHT
  for _ in range(fps // 2):
    screen.render(screen.rect)
  remaining = (1 - screen._nav_bar_y_filter.alpha) ** 30
  assert screen._nav_bar_y_filter.x == pytest.approx(NAV_BAR_MARGIN - (2 * NAV_BAR_MARGIN + NAV_BAR_HEIGHT) * remaining)


def test_long_frame_uses_bounded_spring_steps(monkeypatch, screen):
  screen.show_event()
  monkeypatch.setattr(rl, "get_frame_time", lambda: 0.1)
  screen._update_state()
  expected = screen._y_pos_filter.x, screen._y_pos_filter.velocity.x

  screen.show_event()
  monkeypatch.setattr(rl, "get_frame_time", lambda: 5.0)
  screen._update_state()
  assert (screen._y_pos_filter.x, screen._y_pos_filter.velocity.x) == pytest.approx(expected)


@pytest.mark.parametrize("elapsed", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_frame_duration_uses_default_step(monkeypatch, screen, elapsed):
  screen.show_event()
  screen._update_state()
  expected = screen._y_pos_filter.x, screen._y_pos_filter.velocity.x

  screen.show_event()
  monkeypatch.setattr(rl, "get_frame_time", lambda: elapsed)
  screen._update_state()
  assert (screen._y_pos_filter.x, screen._y_pos_filter.velocity.x) == pytest.approx(expected)
