from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Callable

import pyray as rl

from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.inputbox import InputBox
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.label import gui_label

from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import _SettingsPage, StarPilotPanelInfo, StarPilotPanelType
from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import (
  AETHER_LIST_METRICS,
  AetherListColors,
  DEFAULT_PANEL_STYLE,
  PanelManagerView,
  SettingRow,
  SettingSection,
  draw_empty_state_card,
  draw_list_group_shell,
  draw_rounded_fill,
  draw_rounded_stroke,
  draw_section_header,
  draw_selection_list_row,
  draw_settings_list_row,
  draw_standard_toggle_row,
  with_alpha,
)


PANEL_STYLE = DEFAULT_PANEL_STYLE
SEARCH_BOX_HEIGHT = 86
SEARCH_ACTION_HEIGHT = 82
SEARCH_ROW_HEIGHT = 122
SEARCH_HEADER_HEIGHT = 54
SEARCH_GAP = 16
MAX_SEARCH_RESULTS = 80


@dataclass(frozen=True)
class SearchEntry:
  row: SettingRow
  path: str
  terms: str


class SearchInputBox(InputBox):
  def _render(self, rect: rl.Rectangle):
    super()._render(
      rect,
      color=rl.Color(4, 4, 8, 210),
      border_color=with_alpha(PANEL_STYLE.surface_border, 40),
      text_color=PANEL_STYLE.title_color,
      font_size=34,
    )


class StarPilotSearchLayout(_SettingsPage):
  def __init__(self, panel_provider: Callable[[], dict[StarPilotPanelType, StarPilotPanelInfo]]):
    super().__init__()
    self._keyboard: Keyboard | None = None
    self._manager_view = SettingsSearchView(self, panel_provider)

  def open_keyboard(self, current_text: str) -> None:
    if self._keyboard is None:
      self._keyboard = Keyboard(min_text_size=0)
    self._keyboard.clear()
    self._keyboard.set_text(current_text)
    self._keyboard.set_title(tr("Search Settings"), tr("Type a setting name or parameter key."))
    self._keyboard.set_callback(self._on_keyboard_result)
    gui_app.push_widget(self._keyboard)

  def _on_keyboard_result(self, result: DialogResult) -> None:
    if result == DialogResult.CONFIRM and isinstance(self._manager_view, SettingsSearchView):
      self._manager_view.set_query(self._keyboard.text)


class SettingsSearchView(PanelManagerView):
  METRICS = AETHER_LIST_METRICS
  PANEL_STYLE = PANEL_STYLE

  def __init__(self, controller: StarPilotSearchLayout,
               panel_provider: Callable[[], dict[StarPilotPanelType, StarPilotPanelInfo]]):
    super().__init__()
    self._controller = controller
    self._panel_provider = panel_provider
    self._query_box = self._child(SearchInputBox(max_text_size=64))
    self._entries: list[SearchEntry] = []
    self._matches: list[SearchEntry] = []
    self._last_query = ""
    self._index_ready = False

  def show_event(self):
    super().show_event()
    self._rebuild_index()
    self._last_query = ""

  def set_query(self, query: str) -> None:
    self._query_box.text = query
    self._last_query = ""

  def _row_enabled(self, row: SettingRow) -> bool:
    try:
      return row.enabled() if row.enabled is not None else True
    except Exception:
      return False

  def _row_visible(self, row: SettingRow) -> bool:
    try:
      return row.visible() if row.visible is not None else True
    except Exception:
      return False

  def _section_visible(self, section: SettingSection) -> bool:
    try:
      return section.visible() if section.visible is not None else True
    except Exception:
      return False

  def _row_terms(self, row: SettingRow, path: str) -> str:
    parts = [
      row.id,
      row.title,
      row.subtitle,
      row.disabled_label,
      row.action_text,
      path,
    ]
    return " ".join(str(p) for p in parts if p).lower()

  def _panel_title(self, panel_type: StarPilotPanelType, info: StarPilotPanelInfo) -> str:
    if info.name:
      return tr(info.name)
    return panel_type.name.replace("_", " ").title()

  def _view_title(self, name: str, widget: Widget) -> str:
    header_title = getattr(widget, "_header_title", "")
    if header_title:
      return tr(header_title)
    return name.replace("_", " ").title()

  def _collect_rows_from_view(self, widget: Widget, path: str, seen: set[int]) -> list[SearchEntry]:
    entries: list[SearchEntry] = []

    sections = getattr(widget, "_sections", None)
    if sections:
      for section in sections:
        if not isinstance(section, SettingSection) or not self._section_visible(section):
          continue
        section_path = path
        if section.title:
          section_path = f"{path} / {tr(section.title)}"
        for row in section.rows:
          if not isinstance(row, SettingRow) or id(row) in seen:
            continue
          if row.type not in ("toggle", "value", "action"):
            continue
          if row.get_state is None and row.on_click is None and row.set_state is None and row.type != "action":
            continue
          seen.add(id(row))
          entries.append(SearchEntry(row, section_path, self._row_terms(row, section_path)))

    for sub_name, sub_widget in getattr(widget, "_sub_panels", {}).items():
      if sub_widget is None:
        continue
      sub_path = f"{path} / {self._view_title(str(sub_name), sub_widget)}"
      entries.extend(self._collect_rows_from_view(sub_widget, sub_path, seen))

    return entries

  def _rebuild_index(self) -> None:
    seen: set[int] = set()
    entries: list[SearchEntry] = []
    for panel_type, info in self._panel_provider().items():
      if panel_type in (StarPilotPanelType.MAIN, StarPilotPanelType.SEARCH):
        continue
      if info.instance is None:
        continue
      panel_path = self._panel_title(panel_type, info)
      entries.extend(self._collect_rows_from_view(info.instance, panel_path, seen))
    self._entries = entries
    self._index_ready = True
    self._matches = self._filter_entries()

  def _filter_entries(self) -> list[SearchEntry]:
    query = self._query_box.text.strip().lower()
    if len(query) < 2:
      return []

    tokens = [token for token in query.split() if token]
    scored: list[tuple[int, SearchEntry]] = []
    for entry in self._entries:
      if not all(token in entry.terms for token in tokens):
        continue
      title = str(entry.row.title).lower()
      row_id = entry.row.id.lower()
      score = 0
      if title.startswith(query) or row_id.startswith(query):
        score += 40
      if query in title:
        score += 25
      if query in row_id:
        score += 20
      if self._row_visible(entry.row):
        score += 5
      scored.append((score, entry))

    scored.sort(key=lambda item: (-item[0], tr(item[1].row.title).lower(), item[1].path))
    return [entry for _score, entry in scored[:MAX_SEARCH_RESULTS]]

  def _refresh_matches_if_needed(self) -> None:
    if not self._index_ready:
      self._rebuild_index()
    query = self._query_box.text
    if query == self._last_query:
      return
    self._last_query = query
    self._matches = self._filter_entries()

  def _activate_target(self, target_id: str | None):
    if target_id is None:
      return
    if target_id == "action:keyboard":
      self._controller.open_keyboard(self._query_box.text)
      return
    if target_id == "action:clear":
      self.set_query("")
      return
    if not target_id.startswith("result:"):
      return
    try:
      entry = self._matches[int(target_id.split(":", 1)[1])]
    except (IndexError, ValueError):
      return

    row = entry.row
    if not self._row_enabled(row):
      return
    if row.type == "toggle" and row.get_state is not None and row.set_state is not None:
      row.set_state(not row.get_state())
    elif row.on_click is not None:
      row.on_click()

  def _measure_content_height(self, content_width: float) -> float:
    del content_width
    self._refresh_matches_if_needed()
    height = SEARCH_BOX_HEIGHT + SEARCH_GAP + SEARCH_ACTION_HEIGHT + SEARCH_GAP
    if len(self._query_box.text.strip()) < 2:
      return height + 220
    if not self._matches:
      return height + 220
    return height + SEARCH_HEADER_HEIGHT + len(self._matches) * SEARCH_ROW_HEIGHT

  def _draw_search_box(self, rect: rl.Rectangle) -> None:
    draw_rounded_fill(rect, rl.Color(12, 10, 18, 235), radius_px=18)
    draw_rounded_stroke(rect, with_alpha(PANEL_STYLE.surface_border, 38), radius_px=18)
    label_rect = rl.Rectangle(rect.x + 24, rect.y, 185, rect.height)
    gui_label(label_rect, tr("Search"), 31, PANEL_STYLE.title_color, FontWeight.BOLD)
    input_rect = rl.Rectangle(rect.x + 205, rect.y + 12, rect.width - 230, rect.height - 24)
    self._query_box.render(input_rect)

  def _draw_actions(self, rect: rl.Rectangle) -> None:
    col_gap = 14
    col_w = (rect.width - col_gap) / 2
    keyboard_rect = rl.Rectangle(rect.x, rect.y, col_w, rect.height)
    clear_rect = rl.Rectangle(rect.x + col_w + col_gap, rect.y, col_w, rect.height)

    hovered, pressed = self._interactive_state("action:keyboard", keyboard_rect)
    draw_selection_list_row(
      keyboard_rect,
      title=tr("Touch Keyboard"),
      subtitle=tr("Open the on-device keyboard for search text."),
      action_text=tr("Open"),
      hovered=hovered,
      pressed=pressed,
      is_last=True,
      action_pill=True,
      action_width=150,
      action_pill_width=110,
      title_size=29,
      subtitle_size=22,
      action_text_size=22,
      row_separator=PANEL_STYLE.divider_color,
    )

    hovered, pressed = self._interactive_state("action:clear", clear_rect)
    draw_selection_list_row(
      clear_rect,
      title=tr("Clear Search"),
      subtitle=tr("Reset results and show the full search box."),
      action_text=tr("Clear"),
      hovered=hovered,
      pressed=pressed,
      is_last=True,
      action_pill=True,
      action_width=150,
      action_pill_width=110,
      title_size=29,
      subtitle_size=22,
      action_text_size=22,
      row_separator=PANEL_STYLE.divider_color,
    )

  def _draw_result_row(self, rect: rl.Rectangle, entry: SearchEntry, index: int, is_last: bool) -> None:
    row = entry.row
    target_id = f"result:{index}"
    hovered, pressed = self._interactive_state(target_id, rect)
    enabled = self._row_enabled(row)
    visible = self._row_visible(row)
    path = entry.path if visible else f"{entry.path} / Hidden until dependency is enabled"
    subtitle_parts = [path]
    if row.subtitle:
      subtitle_parts.append(tr(row.subtitle))
    subtitle = " - ".join(subtitle_parts)

    if row.type == "toggle":
      value = row.get_state() if row.get_state is not None else False
      draw_standard_toggle_row(
        rect,
        tr(row.title),
        subtitle,
        value,
        enabled=enabled,
        hovered=hovered,
        pressed=pressed,
        is_last=is_last,
        style=PANEL_STYLE,
      )
    elif row.type == "value":
      draw_settings_list_row(
        rect,
        title=tr(row.title),
        subtitle=subtitle,
        value=row.get_value() if row.get_value else tr("Open"),
        enabled=enabled,
        hovered=hovered,
        pressed=pressed,
        is_last=is_last,
        show_chevron=row.on_click is not None,
        title_size=34,
        subtitle_size=22,
        value_size=26,
        style=PANEL_STYLE,
      )
    else:
      draw_selection_list_row(
        rect,
        title=tr(row.title),
        subtitle=subtitle,
        action_text=tr(row.action_text or "Run"),
        hovered=hovered,
        pressed=pressed,
        is_last=is_last,
        action_pill=True,
        title_size=34,
        subtitle_size=22,
        action_text_size=23,
        row_separator=PANEL_STYLE.divider_color,
      )

  def _draw_scroll_content(self, rect: rl.Rectangle, content_width: float) -> None:
    self._refresh_matches_if_needed()
    y = rect.y + self._scroll_offset

    self._draw_search_box(rl.Rectangle(rect.x, y, content_width, SEARCH_BOX_HEIGHT))
    y += SEARCH_BOX_HEIGHT + SEARCH_GAP

    self._draw_actions(rl.Rectangle(rect.x, y, content_width, SEARCH_ACTION_HEIGHT))
    y += SEARCH_ACTION_HEIGHT + SEARCH_GAP

    query = self._query_box.text.strip()
    if len(query) < 2:
      draw_empty_state_card(
        rl.Rectangle(rect.x, y, content_width, 220),
        tr("Type at least 2 characters"),
        tr("Search by setting name, parameter key, or page name."),
        border=with_alpha(PANEL_STYLE.surface_border, 18),
        style=PANEL_STYLE,
      )
      return

    if not self._matches:
      draw_empty_state_card(
        rl.Rectangle(rect.x, y, content_width, 220),
        tr("No matching settings"),
        tr("Try another name, page, or parameter key."),
        border=with_alpha(AetherListColors.WARNING, 45),
        style=PANEL_STYLE,
      )
      return

    draw_section_header(
      rl.Rectangle(rect.x, y, content_width, SEARCH_HEADER_HEIGHT),
      tr("Results"),
      trailing_text=str(len(self._matches)),
      title_size=32,
      trailing_size=24,
      style=PANEL_STYLE,
    )
    y += SEARCH_HEADER_HEIGHT

    group_rect = rl.Rectangle(rect.x, y, content_width, len(self._matches) * SEARCH_ROW_HEIGHT)
    draw_list_group_shell(group_rect, style=PANEL_STYLE)
    for i, entry in enumerate(self._matches):
      row_rect = rl.Rectangle(rect.x, y + i * SEARCH_ROW_HEIGHT, content_width, SEARCH_ROW_HEIGHT)
      self._draw_result_row(row_rect, entry, i, i == len(self._matches) - 1)
