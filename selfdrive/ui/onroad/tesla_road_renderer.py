"""Lightweight model-space road visualization for the on-road UI.

This is deliberately display-only: it consumes the same model and radar
messages as the existing overlays and never publishes a control message.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import pyray as rl

from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
from openpilot.system.ui.widgets import Widget


MAX_DISTANCE_M = 120.0
MAX_RENDERED_TRACKS = 16
TRACK_STALE_SECONDS = 1.25
ROAD_HALF_WIDTH_M = 5.6

SKY_COLOR = rl.Color(17, 22, 30, 255)
ROAD_COLOR = rl.Color(42, 47, 54, 255)
ROAD_EDGE_COLOR = rl.Color(90, 96, 104, 210)
LANE_COLOR = rl.Color(228, 234, 240, 220)
PATH_ENGAGED_COLOR = rl.Color(44, 158, 255, 175)
PATH_DISENGAGED_COLOR = rl.Color(130, 145, 165, 120)
EGO_COLOR = rl.Color(47, 142, 250, 255)
VEHICLE_COLOR = rl.Color(204, 213, 224, 255)
CLOSING_VEHICLE_COLOR = rl.Color(245, 176, 76, 255)


@dataclass
class VisualTrack:
  d_rel: float
  y_rel: float
  v_rel: float
  last_seen: float


class TeslaRoadRenderer(Widget):
  """Draw a calm, Tesla-inspired road scene from already-published messages."""

  def __init__(self):
    super().__init__()
    self._tracks: dict[int, VisualTrack] = {}

  @staticmethod
  def _project(rect: rl.Rectangle, x_m: float, y_m: float) -> rl.Vector2:
    """Map car-space coordinates into a stable perspective view."""
    distance = max(0.0, min(MAX_DISTANCE_M, x_m))
    horizon = rect.y + rect.height * 0.18
    bottom = rect.y + rect.height * 1.02
    depth = distance / (distance + 16.0)
    scale = 480.0 / (distance + 8.0) + 7.0
    return rl.Vector2(rect.x + rect.width * 0.5 - y_m * scale, bottom - (bottom - horizon) * depth)

  @staticmethod
  def _scale_for_distance(distance: float) -> float:
    return 480.0 / (max(0.0, distance) + 8.0) + 7.0

  @staticmethod
  def _model_points(points_x, points_y) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for x, y in zip(points_x, points_y, strict=False):
      x_f, y_f = float(x), float(y)
      if math.isfinite(x_f) and math.isfinite(y_f) and 0.0 <= x_f <= MAX_DISTANCE_M:
        points.append((x_f, y_f))
    return points

  def _draw_road(self, rect: rl.Rectangle, path: list[tuple[float, float]]) -> None:
    if len(path) < 2:
      path = [(0.0, 0.0), (MAX_DISTANCE_M, 0.0)]

    # A wide sequence of perspective segments is cheaper and smoother than a
    # per-frame mesh while keeping the road attached to the model path.
    for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
      p0, p1 = self._project(rect, x0, y0), self._project(rect, x1, y1)
      width = max(20.0, ROAD_HALF_WIDTH_M * 2.0 * self._scale_for_distance((x0 + x1) * 0.5))
      rl.draw_line_ex(p0, p1, width, ROAD_COLOR)

    for edge in (-ROAD_HALF_WIDTH_M, ROAD_HALF_WIDTH_M):
      for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
        p0, p1 = self._project(rect, x0, y0 + edge), self._project(rect, x1, y1 + edge)
        rl.draw_line_ex(p0, p1, max(1.5, self._scale_for_distance(x0) * 0.045), ROAD_EDGE_COLOR)

  def _draw_lane_line(self, rect: rl.Rectangle, line: list[tuple[float, float]], probability: float) -> None:
    if probability < 0.35 or len(line) < 2:
      return

    for (x0, y0), (x1, y1) in zip(line, line[1:], strict=False):
      # Fixed distance dashes prevent screen-space flicker as the perspective changes.
      if int(x0 // 8.0) % 2:
        continue
      p0, p1 = self._project(rect, x0, y0), self._project(rect, x1, y1)
      alpha = int(80 + min(1.0, probability) * 150)
      rl.draw_line_ex(p0, p1, max(1.6, self._scale_for_distance(x0) * 0.04), rl.Color(LANE_COLOR.r, LANE_COLOR.g, LANE_COLOR.b, alpha))

  def _draw_path(self, rect: rl.Rectangle, path: list[tuple[float, float]]) -> None:
    if len(path) < 2:
      return

    color = PATH_ENGAGED_COLOR if ui_state.status == UIStatus.ENGAGED else PATH_DISENGAGED_COLOR
    for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
      p0, p1 = self._project(rect, x0, y0), self._project(rect, x1, y1)
      rl.draw_line_ex(p0, p1, max(5.0, self._scale_for_distance(x0) * 2.0), color)

  def _update_tracks(self) -> list[VisualTrack]:
    now = time.monotonic()
    sm = ui_state.sm
    raw_tracks = []
    if sm.valid.get("liveTracks", False):
      raw_tracks = list(sm["liveTracks"].points)

    # Radar can be absent on some supported cars. The control leads remain a
    # truthful visual fallback without pretending to discover extra objects.
    if not raw_tracks and sm.valid.get("radarState", False):
      radar_state = sm["radarState"]
      raw_tracks = [lead for lead in (radar_state.leadOne, radar_state.leadTwo) if lead.status]

    seen: set[int] = set()
    for index, track in enumerate(raw_tracks[:MAX_RENDERED_TRACKS]):
      d_rel = float(getattr(track, "dRel", float("nan")))
      y_rel = float(getattr(track, "yRel", float("nan")))
      v_rel = float(getattr(track, "vRel", 0.0))
      if not (math.isfinite(d_rel) and math.isfinite(y_rel) and math.isfinite(v_rel)):
        continue
      if not 1.5 <= d_rel <= MAX_DISTANCE_M or abs(y_rel) > ROAD_HALF_WIDTH_M * 2.0:
        continue

      track_id = int(getattr(track, "trackId", -(index + 1)))
      previous = self._tracks.get(track_id)
      if previous is None:
        visual = VisualTrack(d_rel, y_rel, v_rel, now)
      else:
        # Smooth the display position only; controls still consume untouched messages.
        blend = 0.34
        visual = VisualTrack(
          previous.d_rel + (d_rel - previous.d_rel) * blend,
          previous.y_rel + (y_rel - previous.y_rel) * blend,
          previous.v_rel + (v_rel - previous.v_rel) * blend,
          now,
        )
      self._tracks[track_id] = visual
      seen.add(track_id)

    for track_id, track in tuple(self._tracks.items()):
      if track_id not in seen and now - track.last_seen > TRACK_STALE_SECONDS:
        del self._tracks[track_id]

    return sorted(self._tracks.values(), key=lambda track: track.d_rel)[:MAX_RENDERED_TRACKS]

  def _draw_vehicle(self, rect: rl.Rectangle, track: VisualTrack) -> None:
    center = self._project(rect, track.d_rel, track.y_rel)
    scale = self._scale_for_distance(track.d_rel)
    width = max(10.0, min(70.0, scale * 1.8))
    height = max(16.0, min(105.0, scale * 4.1))
    body = rl.Rectangle(center.x - width * 0.5, center.y - height * 0.72, width, height)
    color = CLOSING_VEHICLE_COLOR if track.v_rel < -2.5 else VEHICLE_COLOR
    rl.draw_rectangle_rounded(body, 0.33, 8, color)
    glass = rl.Rectangle(body.x + width * 0.18, body.y + height * 0.18, width * 0.64, height * 0.25)
    rl.draw_rectangle_rounded(glass, 0.30, 6, rl.Color(83, 98, 113, 255))
    lamp_color = rl.Color(255, 87, 87, 255) if track.v_rel < -2.5 else rl.Color(240, 246, 255, 255)
    lamp_y = body.y + body.height * 0.78
    lamp_w = max(2.0, width * 0.16)
    rl.draw_rectangle(int(body.x + width * 0.16), int(lamp_y), int(lamp_w), max(2, int(height * 0.06)), lamp_color)
    rl.draw_rectangle(int(body.x + width * 0.68), int(lamp_y), int(lamp_w), max(2, int(height * 0.06)), lamp_color)

  def _draw_ego(self, rect: rl.Rectangle) -> None:
    center = self._project(rect, 0.0, 0.0)
    width, height = min(98.0, rect.width * 0.11), min(178.0, rect.height * 0.19)
    body = rl.Rectangle(center.x - width * 0.5, center.y - height * 0.72, width, height)
    rl.draw_rectangle_rounded(body, 0.32, 10, EGO_COLOR)
    glass = rl.Rectangle(body.x + width * 0.17, body.y + height * 0.17, width * 0.66, height * 0.30)
    rl.draw_rectangle_rounded(glass, 0.30, 8, rl.Color(28, 62, 99, 255))
    rl.draw_rectangle_rounded_lines_ex(body, 0.32, 10, 2.0, rl.Color(184, 224, 255, 220))

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, SKY_COLOR)

    sm = ui_state.sm
    path: list[tuple[float, float]] = []
    lane_lines: list[tuple[list[tuple[float, float]], float]] = []
    if sm.valid.get("modelV2", False):
      model = sm["modelV2"]
      path = self._model_points(model.position.x, model.position.y)
      probabilities = list(model.laneLineProbs)
      for index in (1, 2):
        if index < len(model.laneLines):
          probability = float(probabilities[index]) if index < len(probabilities) else 0.0
          lane = model.laneLines[index]
          lane_lines.append((self._model_points(lane.x, lane.y), probability))

    self._draw_road(rect, path)
    for line, probability in lane_lines:
      self._draw_lane_line(rect, line, probability)
    self._draw_path(rect, path)
    for track in reversed(self._update_tracks()):
      self._draw_vehicle(rect, track)
    self._draw_ego(rect)
