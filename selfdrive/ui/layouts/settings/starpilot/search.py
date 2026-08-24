from __future__ import annotations

from collections.abc import Callable

import pyray as rl

from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.inputbox import InputBox
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.label import gui_label

from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import _SettingsPage, StarPilotPanelInfo, StarPilotPanelType
from openpilot.selfdrive.ui.layouts.settings.starpilot.settings_index import (
  SettingsEntry,
  build_settings_index,
  filter_settings_entries,
  row_enabled,
  row_visible,
)
from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import (
  AETHER_LIST_METRICS,
  AetherListColors,
  DEFAULT_PANEL_STYLE,
  PanelManagerView,
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
SEARCH_BOX_HEIGHT = 106
SEARCH_ACTION_HEIGHT = 96
SEARCH_ROW_HEIGHT = 148
SEARCH_HEADER_HEIGHT = 64
SEARCH_GAP = 16
MAX_SEARCH_RESULTS = 80


class SearchInputBox(InputBox):
  def _render(self, rect: rl.Rectangle):
    super()._render(
      rect,
      color=rl.Color(4, 4, 8, 210),
      border_color=with_alpha(PANEL_STYLE.surface_border, 40),
      text_color=PANEL_STYLE.title_color,
      font_size=42,
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
    self._entries: list[SettingsEntry] = []
    self._matches: list[SettingsEntry] = []
    self._last_query = ""
    self._index_ready = False

  def show_event(self):
    super().show_event()
    self._rebuild_index()
    self._last_query = ""

  def set_query(self, query: str) -> None:
    self._query_box.text = query
    self._last_query = ""

  def _rebuild_index(self) -> None:
    self._entries = build_settings_index(
      self._panel_provider,
      exclude_panel_types={StarPilotPanelType.MAIN, StarPilotPanelType.SEARCH},
    )
    self._index_ready = True
    self._matches = self._filter_entries()

  def _filter_entries(self) -> list[SettingsEntry]:
    return filter_settings_entries(self._entries, self._query_box.text, MAX_SEARCH_RESULTS)

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
    if not row_enabled(row):
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
    gui_label(label_rect, tr("Search"), 42, PANEL_STYLE.title_color, FontWeight.BOLD)
    input_rect = rl.Rectangle(rect.x + 205, rect.y + 12, rect.width - 230, rect.height - 24)
    self._interactive_state("action:keyboard", input_rect)
    self._query_box.render(input_rect)

  def _draw_actions(self, rect: rl.Rectangle) -> None:
    hovered, pressed = self._interactive_state("action:clear", rect)
    draw_selection_list_row(
      rect,
      title=tr("Clear Search"),
      subtitle=tr("Reset results and show the full search box."),
      action_text=tr("Clear"),
      hovered=hovered,
      pressed=pressed,
      is_last=True,
      action_pill=True,
      action_width=220,
      action_pill_height=84,
      action_pill_width=160,
      title_size=42,
      subtitle_size=30,
      action_text_size=36,
      row_separator=PANEL_STYLE.divider_color,
    )

  def _draw_result_row(self, rect: rl.Rectangle, entry: SettingsEntry, index: int, is_last: bool) -> None:
    row = entry.row
    target_id = f"result:{index}"
    hovered, pressed = self._interactive_state(target_id, rect)
    enabled = row_enabled(row)
    visible = row_visible(row)
    path = entry.label_path if visible else f"{entry.label_path} / Hidden until dependency is enabled"
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
        title_size=42,
        subtitle_size=30,
        value_size=34,
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
        action_width=260,
        action_pill_height=84,
        action_pill_width=180,
        title_size=42,
        subtitle_size=30,
        action_text_size=36,
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
      title_size=40,
      trailing_size=30,
      style=PANEL_STYLE,
    )
    y += SEARCH_HEADER_HEIGHT

    group_rect = rl.Rectangle(rect.x, y, content_width, len(self._matches) * SEARCH_ROW_HEIGHT)
    draw_list_group_shell(group_rect, style=PANEL_STYLE)
    for i, entry in enumerate(self._matches):
      row_rect = rl.Rectangle(rect.x, y + i * SEARCH_ROW_HEIGHT, content_width, SEARCH_ROW_HEIGHT)
      self._draw_result_row(row_rect, entry, i, i == len(self._matches) - 1)
