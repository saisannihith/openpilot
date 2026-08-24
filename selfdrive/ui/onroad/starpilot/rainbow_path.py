from __future__ import annotations

import colorsys
import time

import pyray as rl
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient


DEFAULT_NUM_SEGMENTS = 8
DEFAULT_SPEED = 50.0
DEFAULT_SATURATION = 0.9
DEFAULT_LIGHTNESS = 0.6
BASE_ALPHA = 0.8
ALPHA_FADE = 0.3


def _hsla_to_color(h: float, s: float, l: float, a: float) -> rl.Color:
  rgb = colorsys.hls_to_rgb(h, l, s)
  return rl.Color(int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255), int(a * 255))


class RainbowPath:
  """Sunnypilot-style full-spectrum Tesla rainbow path renderer."""

  def __init__(self, num_segments: int = DEFAULT_NUM_SEGMENTS, speed: float = DEFAULT_SPEED,
               saturation: float = DEFAULT_SATURATION, lightness: float = DEFAULT_LIGHTNESS) -> None:
    self.num_segments = num_segments
    self.speed = speed
    self.saturation = saturation
    self.lightness = lightness

  def update(self, speed_ms: float) -> None:
    del speed_ms

  def set_speed(self, speed: float):
    self.speed = speed

  def set_num_segments(self, num_segments: int):
    self.num_segments = max(2, int(num_segments))

  def set_saturation(self, saturation: float):
    self.saturation = max(0.0, min(1.0, saturation))

  def set_lightness(self, lightness: float):
    self.lightness = max(0.0, min(1.0, lightness))

  def get_gradient(self, gradient_bottom: float = 1.0, gradient_top: float = 0.0) -> Gradient:
    """Build a Gradient compatible with draw_polygon().

    Args:
      gradient_bottom: Normalized y-position (0-1) of the path bottom.
      gradient_top: Normalized y-position (0-1) of the path top.
    """
    hue_offset = (time.monotonic() * self.speed) % 360.0
    stops = [i / (self.num_segments - 1) for i in range(self.num_segments)]
    colors = []

    for stop in stops:
      path_hue = (hue_offset + stop * 360.0) % 360.0
      alpha = BASE_ALPHA * (1.0 - stop * ALPHA_FADE)
      colors.append(_hsla_to_color(path_hue / 360.0, self.saturation, self.lightness, alpha))

    return Gradient(
      start=(0.0, gradient_bottom),
      end=(0.0, gradient_top),
      colors=colors,
      stops=stops,
    )

  def draw_rainbow_path(self, rect, path):
    draw_polygon(rect, path.projected_points, gradient=self.get_gradient())
