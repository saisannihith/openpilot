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
ROAD_HALF_WIDTH_M = 8.2
PATH_HALF_WIDTH_M = 1.45

SCENE_TOP_COLOR = rl.Color(7, 11, 17, 255)
SCENE_BOTTOM_COLOR = rl.Color(16, 23, 31, 255)
ROAD_COLOR = rl.Color(40, 47, 56, 255)
ROAD_SHADOW_COLOR = rl.Color(23, 29, 36, 255)
ROAD_EDGE_COLOR = rl.Color(88, 101, 116, 205)
LANE_COLOR = rl.Color(190, 205, 221, 205)
PATH_FILL_ENGAGED = rl.Color(21, 112, 232, 148)
PATH_EDGE_ENGAGED = rl.Color(46, 162, 255, 255)
PATH_FILL_DISENGAGED = rl.Color(90, 106, 124, 88)
PATH_EDGE_DISENGAGED = rl.Color(149, 165, 183, 178)
EGO_BODY_COLOR = rl.Color(23, 126, 246, 255)
EGO_TOP_COLOR = rl.Color(77, 176, 255, 255)
VEHICLE_BODY_COLOR = rl.Color(181, 192, 204, 255)
VEHICLE_TOP_COLOR = rl.Color(224, 231, 239, 255)
CLOSING_BODY_COLOR = rl.Color(218, 157, 77, 255)


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
  def _quad(a: rl.Vector2, b: rl.Vector2, c: rl.Vector2, d: rl.Vector2, color: rl.Color) -> None:
    rl.draw_triangle(a, b, c, color)
    rl.draw_triangle(a, c, d, color)

  @staticmethod
  def _blend(start: rl.Color, end: rl.Color, progress: float) -> rl.Color:
    p = max(0.0, min(1.0, progress))
    return rl.Color(
      int(start.r + (end.r - start.r) * p),
      int(start.g + (end.g - start.g) * p),
      int(start.b + (end.b - start.b) * p),
      int(start.a + (end.a - start.a) * p),
    )

  @staticmethod
  def _model_points(points_x, points_y) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for x, y in zip(points_x, points_y, strict=False):
      x_f, y_f = float(x), float(y)
      if math.isfinite(x_f) and math.isfinite(y_f) and 0.0 <= x_f <= MAX_DISTANCE_M:
        points.append((x_f, y_f))
    return points

  def _draw_background(self, rect: rl.Rectangle) -> None:
    bands = 12
    for index in range(bands):
      top = rect.y + rect.height * index / bands
      color = self._blend(SCENE_TOP_COLOR, SCENE_BOTTOM_COLOR, index / max(1, bands - 1))
      rl.draw_rectangle(int(rect.x), int(top), int(rect.width), int(rect.height / bands + 2), color)

  def _draw_surface(self, rect: rl.Rectangle, path: list[tuple[float, float]], left: float, right: float, color: rl.Color) -> None:
    for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
      self._quad(
        self._project(rect, x0, y0 + left),
        self._project(rect, x0, y0 + right),
        self._project(rect, x1, y1 + right),
        self._project(rect, x1, y1 + left),
        color,
      )

  def _draw_road(self, rect: rl.Rectangle, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(path) < 2:
      path = [(0.0, 0.0), (MAX_DISTANCE_M, 0.0)]

    # The road is a bounded set of car-space quads, not a camera projection.
    # It stays visually calm on curves and cannot accumulate mesh state.
    self._draw_surface(rect, path, -ROAD_HALF_WIDTH_M, ROAD_HALF_WIDTH_M, ROAD_SHADOW_COLOR)
    self._draw_surface(rect, path, -(ROAD_HALF_WIDTH_M - 0.35), ROAD_HALF_WIDTH_M - 0.35, ROAD_COLOR)

    for edge in (-ROAD_HALF_WIDTH_M, ROAD_HALF_WIDTH_M):
      for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
        p0, p1 = self._project(rect, x0, y0 + edge), self._project(rect, x1, y1 + edge)
        rl.draw_line_ex(p0, p1, max(2.0, self._scale_for_distance(x0) * 0.055), ROAD_EDGE_COLOR)
    return path

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

    engaged = ui_state.status == UIStatus.ENGAGED
    fill = PATH_FILL_ENGAGED if engaged else PATH_FILL_DISENGAGED
    edge = PATH_EDGE_ENGAGED if engaged else PATH_EDGE_DISENGAGED
    self._draw_surface(rect, path, -PATH_HALF_WIDTH_M, PATH_HALF_WIDTH_M, fill)

    for path_edge in (-PATH_HALF_WIDTH_M, PATH_HALF_WIDTH_M):
      for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
        p0 = self._project(rect, x0, y0 + path_edge)
        p1 = self._project(rect, x1, y1 + path_edge)
        rl.draw_line_ex(p0, p1, max(2.2, self._scale_for_distance(x0) * 0.065), edge)

  @staticmethod
  def _vehicle_points(center: rl.Vector2, width: float, height: float) -> tuple[rl.Vector2, ...]:
    bottom = center.y + height * 0.24
    top = center.y - height * 0.76
    return (
      rl.Vector2(center.x - width * 0.56, bottom),
      rl.Vector2(center.x + width * 0.56, bottom),
      rl.Vector2(center.x + width * 0.42, top + height * 0.22),
      rl.Vector2(center.x + width * 0.23, top),
      rl.Vector2(center.x - width * 0.23, top),
      rl.Vector2(center.x - width * 0.42, top + height * 0.22),
    )

  def _draw_vehicle_body(self, center: rl.Vector2, width: float, height: float, body_color: rl.Color, top_color: rl.Color, braking: bool) -> None:
    p = self._vehicle_points(center, width, height)
    shadow = rl.Rectangle(center.x - width * 0.64, center.y - height * 0.08, width * 1.28, height * 0.74)
    rl.draw_rectangle_rounded(shadow, 0.45, 8, rl.Color(0, 0, 0, 85))

    self._quad(p[0], p[1], p[2], p[5], body_color)
    self._quad(p[5], p[2], p[3], p[4], top_color)
    windshield = rl.Color(38, 57, 78, 255)
    self._quad(
      rl.Vector2(p[5].x + width * 0.10, p[5].y - height * 0.03),
      rl.Vector2(p[2].x - width * 0.10, p[2].y - height * 0.03),
      rl.Vector2(p[3].x - width * 0.06, p[3].y + height * 0.12),
      rl.Vector2(p[4].x + width * 0.06, p[4].y + height * 0.12),
      windshield,
    )
    outline = rl.Color(10, 16, 23, 200)
    for start, end in zip(p, (*p[1:], p[0]), strict=False):
      rl.draw_line_ex(start, end, max(1.0, width * 0.045), outline)

    light = rl.Color(255, 70, 77, 255) if braking else rl.Color(220, 232, 245, 240)
    lamp_y = p[0].y - height * 0.13
    lamp_w = max(2.0, width * 0.16)
    lamp_h = max(2.0, height * 0.055)
    rl.draw_rectangle(int(center.x - width * 0.42), int(lamp_y), int(lamp_w), int(lamp_h), light)
    rl.draw_rectangle(int(center.x + width * 0.26), int(lamp_y), int(lamp_w), int(lamp_h), light)

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

    # Radar interfaces can legitimately recycle or churn IDs. Keep a hard
    # display bound in addition to stale pruning so the UI cannot retain an
    # unbounded history during a noisy drive.
    if len(self._tracks) > MAX_RENDERED_TRACKS:
      oldest_first = sorted(self._tracks.items(), key=lambda item: (item[1].last_seen, -item[1].d_rel))
      for track_id, _ in oldest_first[:len(self._tracks) - MAX_RENDERED_TRACKS]:
        del self._tracks[track_id]

    return sorted(self._tracks.values(), key=lambda track: track.d_rel)[:MAX_RENDERED_TRACKS]

  def _draw_vehicle(self, rect: rl.Rectangle, track: VisualTrack) -> None:
    center = self._project(rect, track.d_rel, track.y_rel)
    scale = self._scale_for_distance(track.d_rel)
    width = max(10.0, min(70.0, scale * 1.8))
    height = max(16.0, min(105.0, scale * 4.1))
    braking = track.v_rel < -2.5
    body = CLOSING_BODY_COLOR if braking else VEHICLE_BODY_COLOR
    self._draw_vehicle_body(center, width, height, body, VEHICLE_TOP_COLOR, braking)

  def _draw_ego(self, rect: rl.Rectangle) -> None:
    center = self._project(rect, 0.0, 0.0)
    width, height = min(98.0, rect.width * 0.11), min(178.0, rect.height * 0.19)
    self._draw_vehicle_body(center, width, height, EGO_BODY_COLOR, EGO_TOP_COLOR, False)

  def _render(self, rect: rl.Rectangle):
    self._draw_background(rect)

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

    path = self._draw_road(rect, path)
    for line, probability in lane_lines:
      self._draw_lane_line(rect, line, probability)
    self._draw_path(rect, path)
    for track in reversed(self._update_tracks()):
      self._draw_vehicle(rect, track)
    self._draw_ego(rect)
