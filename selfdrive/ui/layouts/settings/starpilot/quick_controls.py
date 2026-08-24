from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path

import pyray as rl

from openpilot.common.params import UnknownKeyName
from openpilot.system.ui.lib.application import FontWeight, MouseEvent, MousePos, gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.inputbox import InputBox
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.hardware import HARDWARE

from openpilot.selfdrive.ui.lib.starpilot_state import starpilot_state
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import _SettingsPage, StarPilotPanelInfo, StarPilotPanelType
from openpilot.selfdrive.ui.layouts.settings.starpilot.settings_index import (
  SettingsEntry,
  build_settings_index,
  row_visible,
)
from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import (
  AETHER_LIST_METRICS,
  AetherSettingsView,
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
  draw_settings_panel_header,
  with_alpha,
)
from openpilot.starpilot.common.longitudinal_mode import (
  set_alpha_longitudinal,
  set_experimental_mode,
  set_openpilot_long_disabled,
)


PANEL_STYLE = DEFAULT_PANEL_STYLE
CUSTOMIZE_PANEL_METRICS = replace(AETHER_LIST_METRICS, header_height=136, content_right_gutter=52)
QUICK_CONTROL_ORDER_PARAM = "StarPilotQuickControlOrder"
STOCK_ID_PREFIX = "stock:"
ADD_SEARCH_BOX_HEIGHT = 112
ADD_SEARCH_GAP = 18
ADD_EMPTY_HEIGHT = 180
MAX_ADD_RESULTS = 60


DEFAULT_QUICK_CONTROL_IDS = [
  "stock:AlphaLongitudinalEnabled",
  "stock:ExperimentalMode",
  "stock:DisableOpenpilotLongitudinal",
  "stock:AlwaysOnLateral",
  "stock:LaneCentering",
  "stock:RadarTracksUI",
  "stock:CarnivalFusionHUD",
  "stock:LeadInfo",
  "stock:ShowStoppingPoint",
  "stock:ShowStoppingPointMetrics",
  "stock:ModelUI",
  "stock:CustomUI",
  "stock:DeveloperUI",
  "stock:DeveloperMetrics",
  "stock:DeveloperSidebar",
]


class QuickControlsSearchInputBox(InputBox):
  def _render(self, rect: rl.Rectangle):
    super()._render(
      rect,
      color=rl.Color(4, 4, 8, 210),
      border_color=with_alpha(PANEL_STYLE.surface_border, 42),
      text_color=PANEL_STYLE.title_color,
      font_size=44,
    )


class StarPilotQuickControlsLayout(_SettingsPage):
  def __init__(self, panel_provider: Callable[[], dict[StarPilotPanelType, StarPilotPanelInfo]] | None = None):
    super().__init__()
    self._panel_provider = panel_provider
    self._stock_sections: list[SettingSection] = []
    self._stock_rows: dict[str, SettingRow] = {}
    self._settings_entries: list[SettingsEntry] = []
    self._settings_rows: dict[str, SettingsEntry] = {}
    self._index_ready = False
    self._build_stock_rows()
    self._refresh_index(force=True)
    self._sub_panels["customize"] = QuickControlsCustomizeLayout(self)
    self._wire_sub_panels()
    self._build_view()

  def _alpha_long_enabled(self) -> bool:
    return self._params.get_bool("AlphaLongitudinalEnabled") and not self._params.get_bool("DisableOpenpilotLongitudinal")

  def _set_alpha_longitudinal(self, enabled: bool) -> None:
    set_alpha_longitudinal(self._params, enabled)
    starpilot_state.update(force=True)

  def _disable_openpilot_long_enabled(self) -> bool:
    return self._params.get_bool("DisableOpenpilotLongitudinal")

  def _set_disable_openpilot_long(self, disabled: bool) -> None:
    if disabled:
      def on_confirm(res):
        if res == DialogResult.CONFIRM:
          set_openpilot_long_disabled(self._params, True)
          starpilot_state.update(force=True)
          if ui_state.started:
            HARDWARE.reboot()

      gui_app.push_widget(ConfirmDialog(
        tr("Disable openpilot longitudinal control?"),
        tr("Disable"),
        callback=on_confirm,
      ))
    else:
      set_openpilot_long_disabled(self._params, False)
      starpilot_state.update(force=True)

  def _set_developer_ui(self, enabled: bool) -> None:
    self._params.put_bool("DeveloperUI", enabled)
    self._params.put_bool("GalaxyDeveloperMode", enabled)

  def _set_developer_sidebar(self, enabled: bool) -> None:
    self._params.put_bool("DeveloperSidebar", enabled)
    if enabled:
      self._set_developer_ui(True)

  def _set_developer_metrics(self, enabled: bool) -> None:
    self._params.put_bool("DeveloperMetrics", enabled)
    if enabled:
      self._set_developer_ui(True)

  def _set_experimental_mode(self, enabled: bool) -> None:
    set_experimental_mode(self._params, enabled)

  def _set_always_on_lateral(self, enabled: bool) -> None:
    if enabled:
      def on_confirm(res):
        if res == DialogResult.CONFIRM:
          self._params.put_bool("AlwaysOnLateral", True)
          if ui_state.started:
            HARDWARE.reboot()

      gui_app.push_widget(ConfirmDialog(
        tr("Enable Always On Lateral?"),
        tr("Enable"),
        callback=on_confirm,
      ))
    else:
      self._params.put_bool("AlwaysOnLateral", False)

  def _build_stock_rows(self) -> None:
    has_long = lambda: starpilot_state.car_state.hasOpenpilotLongitudinal
    has_alpha = lambda: starpilot_state.car_state.hasAlphaLongitudinal
    has_radar = lambda: starpilot_state.car_state.hasRadar

    drive_rows = [
      SettingRow(
        "AlphaLongitudinalEnabled", "toggle", tr_noop("Alpha Longitudinal"),
        subtitle=tr_noop("Let openpilot control gas and brake when supported."),
        get_state=self._alpha_long_enabled,
        set_state=self._set_alpha_longitudinal,
        enabled=has_alpha,
        disabled_label=tr_noop("Not available for this car"),
      ),
      SettingRow(
        "ExperimentalMode", "toggle", tr_noop("Experimental Mode"),
        subtitle=tr_noop("Enable model-based city behavior; chill mode is synchronized automatically."),
        get_state=lambda: self._params.get_bool("ExperimentalMode"),
        set_state=self._set_experimental_mode,
        enabled=has_long,
        disabled_label=tr_noop("Requires Alpha Longitudinal"),
      ),
      SettingRow(
        "DisableOpenpilotLongitudinal", "toggle", tr_noop("Disable openpilot Long"),
        subtitle=tr_noop("Use stock ACC instead of openpilot gas and brake."),
        get_state=self._disable_openpilot_long_enabled,
        set_state=self._set_disable_openpilot_long,
      ),
      SettingRow(
        "AlwaysOnLateral", "toggle", tr_noop("Always On Lateral"),
        subtitle=tr_noop("Keep steering available even when ACC is off. Reboot may be required."),
        get_state=lambda: self._params.get_bool("AlwaysOnLateral"),
        set_state=self._set_always_on_lateral,
      ),
      SettingRow(
        "LaneCentering", "toggle", tr_noop("Lane Centering"),
        subtitle=tr_noop("Apply StarPilot lane-centering bias logic."),
        get_state=lambda: self._params.get_bool("LaneCentering"),
        set_state=lambda s: self._params.put_bool("LaneCentering", s),
      ),
    ]

    visual_rows = [
      SettingRow(
        "RadarTracksUI", "toggle", tr_noop("Radar Tracks"),
        subtitle=tr_noop("Show radar points from the vehicle sensor on the driving screen."),
        get_state=lambda: self._params.get_bool("RadarTracksUI"),
        set_state=lambda s: self._params.put_bool("RadarTracksUI", s),
        enabled=has_radar,
        disabled_label=tr_noop("No radar detected"),
      ),
      SettingRow(
        "CarnivalFusionHUD", "toggle", tr_noop("Carnival Fusion HUD"),
        subtitle=tr_noop("Explain lead fusion, stop hold, confidence, and EPS risk."),
        get_state=lambda: self._params.get_bool("CarnivalFusionHUD"),
        set_state=lambda s: self._params.put_bool("CarnivalFusionHUD", s),
      ),
      SettingRow(
        "LeadInfo", "toggle", tr_noop("Lead Vehicle Metrics"),
        subtitle=tr_noop("Show lead-distance and lead-speed details."),
        get_state=lambda: self._params.get_bool("LeadInfo"),
        set_state=lambda s: self._params.put_bool("LeadInfo", s),
        enabled=has_long,
      ),
      SettingRow(
        "ShowStoppingPoint", "toggle", tr_noop("Stop Sign / Light Marker"),
        subtitle=tr_noop("Show detected stop target on the driving screen."),
        get_state=lambda: self._params.get_bool("ShowStoppingPoint"),
        set_state=lambda s: self._params.put_bool("ShowStoppingPoint", s),
        enabled=has_long,
      ),
      SettingRow(
        "ShowStoppingPointMetrics", "toggle", tr_noop("Stop Distance"),
        subtitle=tr_noop("Show distance to the detected stopping point."),
        get_state=lambda: self._params.get_bool("ShowStoppingPointMetrics"),
        set_state=lambda s: self._params.put_bool("ShowStoppingPointMetrics", s),
        enabled=lambda: self._params.get_bool("ShowStoppingPoint") and has_long(),
      ),
      SettingRow(
        "ModelUI", "toggle", tr_noop("Model / Path Visualization"),
        subtitle=tr_noop("Show path, lane lines, and road edges."),
        get_state=lambda: self._params.get_bool("ModelUI"),
        set_state=lambda s: self._params.put_bool("ModelUI", s),
      ),
      SettingRow(
        "CustomUI", "toggle", tr_noop("Driving Widgets"),
        subtitle=tr_noop("Show StarPilot driving widgets and HUD extras."),
        get_state=lambda: self._params.get_bool("CustomUI"),
        set_state=lambda s: self._params.put_bool("CustomUI", s),
      ),
    ]

    developer_rows = [
      SettingRow(
        "DeveloperUI", "toggle", tr_noop("Developer Options"),
        subtitle=tr_noop("Expose developer-only controls on device and in Galaxy."),
        get_state=lambda: self._params.get_bool("DeveloperUI") or self._params.get_bool("GalaxyDeveloperMode"),
        set_state=self._set_developer_ui,
      ),
      SettingRow(
        "DeveloperMetrics", "toggle", tr_noop("Developer Metrics"),
        subtitle=tr_noop("Show performance, sensor, and system metric controls."),
        get_state=lambda: self._params.get_bool("DeveloperMetrics"),
        set_state=self._set_developer_metrics,
      ),
      SettingRow(
        "DeveloperSidebar", "toggle", tr_noop("Developer Sidebar"),
        subtitle=tr_noop("Show the live metrics sidebar on the driving screen."),
        get_state=lambda: self._params.get_bool("DeveloperSidebar"),
        set_state=self._set_developer_sidebar,
      ),
    ]

    self._stock_sections = [
      SettingSection(tr_noop("Driving"), drive_rows),
      SettingSection(tr_noop("Onroad Display"), visual_rows),
      SettingSection(tr_noop("Developer"), developer_rows),
    ]
    self._stock_rows = {f"{STOCK_ID_PREFIX}{row.id}": row for section in self._stock_sections for row in section.rows}

  def _refresh_index(self, *, force: bool = False) -> None:
    if self._index_ready and not force:
      return
    if self._panel_provider is None:
      self._settings_entries = []
      self._settings_rows = {}
      self._index_ready = True
      return
    panels = self._panel_provider()
    if not panels:
      self._settings_entries = []
      self._settings_rows = {}
      self._index_ready = False
      return
    self._settings_entries = build_settings_index(
      lambda: panels,
      exclude_panel_types={StarPilotPanelType.MAIN, StarPilotPanelType.QUICK_CONTROLS, StarPilotPanelType.SEARCH},
    )
    self._settings_rows = {entry.stable_id: entry for entry in self._settings_entries}
    self._index_ready = True

  def _load_control_ids(self) -> list[str]:
    raw = self._load_order_payload()
    if isinstance(raw, bytes):
      raw = raw.decode("utf-8", errors="ignore")
    if not raw:
      return list(DEFAULT_QUICK_CONTROL_IDS)
    try:
      parsed = json.loads(raw)
    except (TypeError, ValueError):
      return list(DEFAULT_QUICK_CONTROL_IDS)
    if not isinstance(parsed, list):
      return list(DEFAULT_QUICK_CONTROL_IDS)
    control_ids = [str(item) for item in parsed if isinstance(item, str)]
    return control_ids or list(DEFAULT_QUICK_CONTROL_IDS)

  def _quick_control_order_path(self) -> Path:
    try:
      return Path(self._params.get_param_path(QUICK_CONTROL_ORDER_PARAM))
    except Exception:
      return Path("/data/params/d") / QUICK_CONTROL_ORDER_PARAM

  def _load_order_payload(self) -> str | bytes | None:
    raw = self._params.get(QUICK_CONTROL_ORDER_PARAM)
    decoded = raw.decode("utf-8", errors="ignore") if isinstance(raw, bytes) else raw
    fallback_path = self._quick_control_order_path()
    if fallback_path.exists() and (decoded is None or str(decoded).strip() in ("", "[]")):
      try:
        return fallback_path.read_text(encoding="utf-8")
      except OSError:
        return raw
    return raw

  def _save_order_fallback(self, payload: str) -> None:
    fallback_path = self._quick_control_order_path()
    fallback_path.parent.mkdir(parents=True, exist_ok=True)
    fallback_path.write_text(payload, encoding="utf-8")

  def _save_control_ids(self, control_ids: list[str]) -> None:
    deduped: list[str] = []
    seen: set[str] = set()
    for control_id in control_ids:
      if control_id in seen:
        continue
      seen.add(control_id)
      deduped.append(control_id)
    payload = json.dumps(deduped, separators=(",", ":"))
    try:
      self._params.put(QUICK_CONTROL_ORDER_PARAM, payload)
    except UnknownKeyName:
      self._save_order_fallback(payload)
    self._build_view()
    customize = self._sub_panels.get("customize")
    if isinstance(customize, QuickControlsCustomizeLayout):
      customize.refresh()

  def _resolve_row(self, control_id: str) -> SettingRow | None:
    if control_id.startswith(STOCK_ID_PREFIX):
      return self._stock_rows.get(control_id)
    entry = self._settings_rows.get(control_id)
    return entry.row if entry is not None else None

  def _control_label(self, control_id: str) -> str:
    row = self._resolve_row(control_id)
    return tr(row.title) if row is not None else control_id

  def _control_subtitle(self, control_id: str) -> str:
    if control_id.startswith(STOCK_ID_PREFIX):
      return tr("Quick Controls")
    entry = self._settings_rows.get(control_id)
    return entry.label_path if entry is not None else tr("Unavailable")

  def _effective_control_ids(self) -> list[str]:
    self._refresh_index()
    valid: list[str] = []
    seen: set[str] = set()
    for control_id in self._load_control_ids():
      if control_id in seen or self._resolve_row(control_id) is None:
        continue
      seen.add(control_id)
      valid.append(control_id)
    return valid or list(DEFAULT_QUICK_CONTROL_IDS)

  def selected_control_ids(self) -> list[str]:
    return self._effective_control_ids()

  def available_control_entries(self) -> list[tuple[str, SettingRow, str]]:
    selected = set(self._effective_control_ids())
    available: list[tuple[str, SettingRow, str]] = []

    for control_id, row in self._stock_rows.items():
      if control_id not in selected:
        available.append((control_id, row, tr("Quick Controls")))

    stock_row_ids = {row.id for row in self._stock_rows.values()}
    for entry in self._settings_entries:
      if entry.stable_id in selected or entry.row.id in stock_row_ids:
        continue
      if not row_visible(entry.row):
        continue
      available.append((entry.stable_id, entry.row, entry.label_path))

    available.sort(key=lambda item: (tr(item[1].title).lower(), item[2].lower()))
    return available

  def show_event(self):
    self._refresh_index(force=True)
    self._build_view()
    super().show_event()

  def move_control(self, control_id: str, direction: int) -> None:
    control_ids = self._effective_control_ids()
    try:
      index = control_ids.index(control_id)
    except ValueError:
      return
    new_index = index + direction
    if new_index < 0 or new_index >= len(control_ids):
      return
    control_ids[index], control_ids[new_index] = control_ids[new_index], control_ids[index]
    self._save_control_ids(control_ids)

  def remove_control(self, control_id: str) -> None:
    control_ids = [item for item in self._effective_control_ids() if item != control_id]
    self._save_control_ids(control_ids)

  def add_control(self, control_id: str) -> None:
    control_ids = self._effective_control_ids()
    if control_id not in control_ids and self._resolve_row(control_id) is not None:
      control_ids.append(control_id)
      self._save_control_ids(control_ids)

  def reset_controls(self) -> None:
    self._save_control_ids(list(DEFAULT_QUICK_CONTROL_IDS))

  def _build_view(self) -> None:
    manage_rows = [
      SettingRow(
        "CustomizeQuickControls", "action", tr_noop("Customize Quick Controls"),
        subtitle=tr_noop("Add, remove, and reorder controls saved on this device."),
        action_text=tr_noop("Open"),
        navigate_to="customize",
      ),
    ]

    control_rows = []
    for control_id in self._effective_control_ids():
      row = self._resolve_row(control_id)
      if row is not None:
        control_rows.append(row)

    self._manager_view = AetherSettingsView(
      self,
      [
        SettingSection(tr_noop("Manage"), manage_rows),
        SettingSection(tr_noop("Pinned Controls"), control_rows),
      ],
      header_title=tr_noop("Quick Controls"),
      header_subtitle=tr_noop("Saved driving, radar, model, and developer controls."),
      panel_style=PANEL_STYLE,
    )


class QuickControlsCustomizeLayout(_SettingsPage):
  def __init__(self, owner: StarPilotQuickControlsLayout):
    super().__init__()
    self._owner = owner
    self._keyboard: Keyboard | None = None
    self._manager_view = QuickControlsCustomizeView(self, owner)

  def refresh(self) -> None:
    if isinstance(self._manager_view, QuickControlsCustomizeView):
      self._manager_view.refresh()

  def open_add_search_keyboard(self, current_text: str) -> None:
    if self._keyboard is None:
      self._keyboard = Keyboard(min_text_size=0)
    self._keyboard.clear()
    self._keyboard.set_text(current_text)
    self._keyboard.set_title(tr("Add Quick Control"), tr("Search any StarPilot setting."))
    self._keyboard.set_callback(self._on_keyboard_result)
    gui_app.push_widget(self._keyboard)

  def _on_keyboard_result(self, result: DialogResult) -> None:
    if result == DialogResult.CONFIRM and isinstance(self._manager_view, QuickControlsCustomizeView):
      self._manager_view.set_add_query(self._keyboard.text)

  def show_event(self):
    self.refresh()
    super().show_event()

  def _reset(self) -> None:
    def on_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        self._owner.reset_controls()

    gui_app.push_widget(ConfirmDialog(
      tr("Reset Quick Controls to default?"),
      tr("Reset"),
      callback=on_confirm,
    ))


class QuickControlsCustomizeView(PanelManagerView):
  METRICS = CUSTOMIZE_PANEL_METRICS
  PANEL_STYLE = PANEL_STYLE

  ROW_HEIGHT = 148
  HEADER_EXTRA = 0
  SECTION_GAP = 28
  HANDLE_WIDTH = 74
  REMOVE_WIDTH = 74
  ACTION_GAP = 12
  DRAG_THRESHOLD = 8

  def __init__(self, controller: QuickControlsCustomizeLayout, owner: StarPilotQuickControlsLayout):
    super().__init__()
    self._controller = controller
    self._owner = owner
    self._query_box = self._child(QuickControlsSearchInputBox(max_text_size=64))
    self._current_ids: list[str] = []
    self._available_all: list[tuple[str, SettingRow, str]] = []
    self._add_matches: list[tuple[str, SettingRow, str]] = []
    self._last_add_query: str | None = None
    self._current_row_rects: dict[int, rl.Rectangle] = {}
    self._drag_index: int | None = None
    self._drag_insert_index: int | None = None
    self._drag_offset_y = 0.0
    self._drag_started_y = 0.0
    self._drag_y = 0.0
    self.refresh()

  @property
  def vertical_scrolling_disabled(self) -> bool:
    return self._drag_index is not None

  def refresh(self) -> None:
    self._current_ids = self._owner.selected_control_ids()
    self._available_all = self._owner.available_control_entries()
    self._last_add_query = None
    self._refresh_add_matches_if_needed()

  def set_add_query(self, query: str) -> None:
    self._query_box.text = query
    self._last_add_query = None
    self._refresh_add_matches_if_needed()

  def _refresh_add_matches_if_needed(self) -> None:
    query = self._query_box.text.strip().lower()
    if query == self._last_add_query:
      return
    self._last_add_query = query
    if len(query) < 2:
      self._add_matches = []
      return

    tokens = [token for token in query.split() if token]
    scored: list[tuple[int, tuple[str, SettingRow, str]]] = []
    for entry in self._available_all:
      control_id, row, path = entry
      terms = " ".join(
        str(part) for part in (
          control_id,
          row.id,
          row.title,
          tr(row.title),
          row.subtitle,
          tr(row.subtitle) if row.subtitle else "",
          row.disabled_label,
          path,
        ) if part
      ).lower()
      if not all(token in terms for token in tokens):
        continue
      title = str(row.title).lower()
      translated_title = tr(row.title).lower()
      row_id = row.id.lower()
      score = 0
      if title.startswith(query) or translated_title.startswith(query) or row_id.startswith(query):
        score += 40
      if query in title or query in translated_title:
        score += 25
      if query in row_id:
        score += 20
      scored.append((score, entry))

    scored.sort(key=lambda item: (-item[0], tr(item[1][1].title).lower(), item[1][2].lower()))
    self._add_matches = [entry for _score, entry in scored[:MAX_ADD_RESULTS]]

  def _draw_header(self, header_rect: rl.Rectangle) -> None:
    draw_settings_panel_header(
      header_rect,
      tr("Customize Quick Controls"),
      tr("Drag the handle to reorder. Use + and x to add or remove controls."),
      title_size=42,
      subtitle_size=31,
    )

  def _measure_content_height(self, content_width: float) -> float:
    del content_width
    self._refresh_add_matches_if_needed()
    available_height = ADD_SEARCH_BOX_HEIGHT + ADD_SEARCH_GAP
    if len(self._query_box.text.strip()) < 2 or not self._add_matches:
      available_height += ADD_EMPTY_HEIGHT
    else:
      available_height += len(self._add_matches) * self.ROW_HEIGHT
    sections = [
      self.METRICS.section_header_height + self.METRICS.section_header_gap + max(1, len(self._current_ids)) * self.ROW_HEIGHT,
      self.METRICS.section_header_height + self.METRICS.section_header_gap + available_height,
      self.METRICS.section_header_height + self.METRICS.section_header_gap + self.ROW_HEIGHT,
    ]
    return self.HEADER_EXTRA + sum(sections) + self.SECTION_GAP * (len(sections) - 1)

  def _target_at(self, mouse_pos: MousePos) -> str | None:
    for target_id, rect in self._interactive_rects.items():
      if rl.check_collision_point_rec(mouse_pos, rect):
        return target_id
    return None

  def _activate_target(self, target_id: str | None):
    if target_id is None:
      return
    if target_id.startswith("remove:"):
      index = self._parse_index(target_id)
      if index is not None and 0 <= index < len(self._current_ids):
        self._owner.remove_control(self._current_ids[index])
        self.refresh()
      return
    if target_id.startswith("add:"):
      index = self._parse_index(target_id)
      if index is not None and 0 <= index < len(self._add_matches):
        self._owner.add_control(self._add_matches[index][0])
        self.refresh()
      return
    if target_id == "action:add_search":
      self._controller.open_add_search_keyboard(self._query_box.text)
      return
    if target_id == "action:clear_add_search":
      self.set_add_query("")
      return
    if target_id == "reset":
      self._controller._reset()

  def _parse_index(self, target_id: str) -> int | None:
    try:
      return int(target_id.split(":", 1)[1])
    except (IndexError, ValueError):
      return None

  def _handle_mouse_press(self, mouse_pos: MousePos):
    super()._handle_mouse_press(mouse_pos)
    target = self._target_at(mouse_pos)
    if target is None or not target.startswith("drag:"):
      return
    index = self._parse_index(target)
    if index is None or index not in self._current_row_rects:
      return
    row_rect = self._current_row_rects[index]
    self._drag_index = index
    self._drag_insert_index = index
    self._drag_offset_y = mouse_pos.y - row_rect.y
    self._drag_started_y = mouse_pos.y
    self._drag_y = mouse_pos.y
    self._can_click = False

  def _handle_mouse_event(self, mouse_event: MouseEvent):
    if self._drag_index is None:
      super()._handle_mouse_event(mouse_event)
      return
    self._drag_y = mouse_event.pos.y
    self._drag_insert_index = self._index_for_y(mouse_event.pos.y)
    self._can_click = False

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if self._drag_index is None:
      super()._handle_mouse_release(mouse_pos)
      return
    from_index = self._drag_index
    to_index = self._drag_insert_index if self._drag_insert_index is not None else from_index
    moved = abs(mouse_pos.y - self._drag_started_y) >= self.DRAG_THRESHOLD
    self._drag_index = None
    self._drag_insert_index = None
    self._pressed_target = None
    self._can_click = True

    if moved and from_index != to_index and 0 <= from_index < len(self._current_ids):
      reordered = list(self._current_ids)
      item = reordered.pop(from_index)
      to_index = max(0, min(to_index, len(reordered)))
      reordered.insert(to_index, item)
      self._owner._save_control_ids(reordered)
    self.refresh()

  def _index_for_y(self, y: float) -> int:
    if not self._current_row_rects:
      return 0
    first_rect = self._current_row_rects.get(0)
    if first_rect is None:
      return 0
    raw_index = int((y - first_rect.y + self.ROW_HEIGHT / 2) / self.ROW_HEIGHT)
    return max(0, min(raw_index, len(self._current_ids) - 1))

  def _draw_icon_button(self, rect: rl.Rectangle, text: str, target_id: str, *, danger: bool = False) -> None:
    hovered, pressed = self._interactive_state(target_id, rect)
    fill = PANEL_STYLE.danger_fill if danger else PANEL_STYLE.current_fill
    border = PANEL_STYLE.danger_border if danger else PANEL_STYLE.current_border
    text_color = PANEL_STYLE.danger_text if danger else PANEL_STYLE.title_color
    if hovered:
      fill = with_alpha(fill, 235)
    if pressed:
      fill = with_alpha(border, 210)
    draw_rounded_fill(rect, fill, radius_px=16)
    draw_rounded_stroke(rect, border, radius_px=16)
    if text == "handle":
      center_y = rect.y + rect.height / 2
      for offset in (-10, 0, 10):
        rl.draw_line_ex(
          rl.Vector2(rect.x + 20, center_y + offset),
          rl.Vector2(rect.x + rect.width - 20, center_y + offset),
          4,
          text_color,
        )
    else:
      gui_label(rect, text, 42, text_color, FontWeight.BOLD, alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER)

  def _draw_current_row(self, rect: rl.Rectangle, control_id: str, index: int, is_last: bool, *, floating: bool = False) -> None:
    action_width = self.HANDLE_WIDTH + self.REMOVE_WIDTH + self.ACTION_GAP + 42
    hovered = self._drag_index == index
    draw_selection_list_row(
      rect,
      title=self._owner._control_label(control_id),
      subtitle=self._owner._control_subtitle(control_id),
      action_text="",
      hovered=hovered,
      pressed=floating,
      is_last=is_last,
      action_width=action_width,
      action_pill=True,
      title_size=44,
      subtitle_size=31,
      row_separator=PANEL_STYLE.divider_color,
    )

    remove_rect = rl.Rectangle(rect.x + rect.width - self.REMOVE_WIDTH - 18, rect.y + 24, self.REMOVE_WIDTH, rect.height - 48)
    handle_rect = rl.Rectangle(remove_rect.x - self.ACTION_GAP - self.HANDLE_WIDTH, remove_rect.y, self.HANDLE_WIDTH, remove_rect.height)
    self._draw_icon_button(handle_rect, "handle", f"drag:{index}")
    self._draw_icon_button(remove_rect, "x", f"remove:{index}", danger=True)

  def _draw_available_row(self, rect: rl.Rectangle, entry: tuple[str, SettingRow, str], index: int, is_last: bool) -> None:
    _control_id, row, path = entry
    draw_selection_list_row(
      rect,
      title=tr(row.title),
      subtitle=path,
      action_text="",
      hovered=False,
      pressed=False,
      is_last=is_last,
      action_width=96,
      action_pill=True,
      title_size=42,
      subtitle_size=30,
      row_separator=PANEL_STYLE.divider_color,
    )
    add_rect = rl.Rectangle(rect.x + rect.width - 92, rect.y + 24, 74, rect.height - 48)
    self._draw_icon_button(add_rect, "+", f"add:{index}")

  def _draw_add_search_box(self, rect: rl.Rectangle) -> None:
    draw_rounded_fill(rect, rl.Color(12, 10, 18, 235), radius_px=18)
    draw_rounded_stroke(rect, with_alpha(PANEL_STYLE.surface_border, 38), radius_px=18)
    label_width = 185
    clear_width = 88 if self._query_box.text.strip() else 0
    label_rect = rl.Rectangle(rect.x + 24, rect.y, label_width, rect.height)
    gui_label(label_rect, tr("Search"), 42, PANEL_STYLE.title_color, FontWeight.BOLD)

    input_right_pad = 26 + clear_width
    input_rect = rl.Rectangle(rect.x + label_width + 28, rect.y + 14, rect.width - label_width - input_right_pad - 28, rect.height - 28)
    self._interactive_state("action:add_search", input_rect)
    self._query_box.render(input_rect)

    if clear_width:
      clear_rect = rl.Rectangle(rect.x + rect.width - 112, rect.y + 16, 88, rect.height - 32)
      self._draw_icon_button(clear_rect, "x", "action:clear_add_search", danger=True)

  def _draw_reset_row(self, rect: rl.Rectangle) -> None:
    draw_selection_list_row(
      rect,
      title=tr("Reset to Default"),
      subtitle=tr("Restore the stock Quick Controls layout."),
      action_text="",
      hovered=False,
      pressed=False,
      is_last=True,
      action_width=156,
      action_pill=True,
      title_size=42,
      subtitle_size=30,
      row_separator=PANEL_STYLE.divider_color,
    )
    reset_rect = rl.Rectangle(rect.x + rect.width - 164, rect.y + 24, 140, rect.height - 48)
    self._draw_icon_button(reset_rect, tr("Reset"), "reset", danger=True)

  def _draw_scroll_content(self, rect: rl.Rectangle, content_width: float) -> None:
    self._refresh_add_matches_if_needed()
    self._current_row_rects.clear()
    y = rect.y + self._scroll_offset

    y += self.HEADER_EXTRA
    y = self._draw_current_section(rect.x, y, content_width)
    y += self.SECTION_GAP
    y = self._draw_available_section(rect.x, y, content_width)
    y += self.SECTION_GAP
    self._draw_reset_section(rect.x, y, content_width)

  def _draw_current_section(self, x: float, y: float, width: float) -> float:
    draw_section_header(rl.Rectangle(x, y, width, self.METRICS.section_header_height), tr("Current Order"), style=PANEL_STYLE)
    y += self.METRICS.section_header_height + self.METRICS.section_header_gap

    rows_count = max(1, len(self._current_ids))
    group_rect = rl.Rectangle(x, y, width, rows_count * self.ROW_HEIGHT)
    draw_list_group_shell(group_rect, style=PANEL_STYLE)

    if not self._current_ids:
      gui_label(group_rect, tr("No pinned controls"), 28, PANEL_STYLE.subtitle_color, alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER)
      return y + group_rect.height

    for i, control_id in enumerate(self._current_ids):
      row_rect = rl.Rectangle(x, y + i * self.ROW_HEIGHT, width, self.ROW_HEIGHT)
      self._current_row_rects[i] = row_rect
      if self._drag_index == i:
        placeholder = rl.Rectangle(row_rect.x + 18, row_rect.y + row_rect.height / 2 - 2, row_rect.width - 36, 4)
        draw_rounded_fill(placeholder, with_alpha(PANEL_STYLE.accent, 180), radius_px=4)
        continue
      self._draw_current_row(row_rect, control_id, i, i == len(self._current_ids) - 1)

    if self._drag_index is not None and 0 <= self._drag_index < len(self._current_ids):
      floating_y = self._drag_y - self._drag_offset_y
      floating_rect = rl.Rectangle(x + 10, floating_y, width - 20, self.ROW_HEIGHT)
      self._draw_current_row(floating_rect, self._current_ids[self._drag_index], self._drag_index, True, floating=True)

    return y + group_rect.height

  def _draw_available_section(self, x: float, y: float, width: float) -> float:
    draw_section_header(rl.Rectangle(x, y, width, self.METRICS.section_header_height), tr("Add Settings"), style=PANEL_STYLE)
    y += self.METRICS.section_header_height + self.METRICS.section_header_gap

    self._draw_add_search_box(rl.Rectangle(x, y, width, ADD_SEARCH_BOX_HEIGHT))
    y += ADD_SEARCH_BOX_HEIGHT + ADD_SEARCH_GAP

    query = self._query_box.text.strip()
    if len(query) < 2:
      draw_empty_state_card(
        rl.Rectangle(x, y, width, ADD_EMPTY_HEIGHT),
        tr("Search to add settings"),
        tr("Type at least 2 characters to find any StarPilot setting."),
        border=with_alpha(PANEL_STYLE.surface_border, 18),
        style=PANEL_STYLE,
      )
      return y + ADD_EMPTY_HEIGHT

    if not self._add_matches:
      draw_empty_state_card(
        rl.Rectangle(x, y, width, ADD_EMPTY_HEIGHT),
        tr("No matching addable settings"),
        tr("Already pinned settings are hidden from these results."),
        border=with_alpha(PANEL_STYLE.surface_border, 28),
        style=PANEL_STYLE,
      )
      return y + ADD_EMPTY_HEIGHT

    group_rect = rl.Rectangle(x, y, width, len(self._add_matches) * self.ROW_HEIGHT)
    draw_list_group_shell(group_rect, style=PANEL_STYLE)
    for i, entry in enumerate(self._add_matches):
      row_rect = rl.Rectangle(x, y + i * self.ROW_HEIGHT, width, self.ROW_HEIGHT)
      self._draw_available_row(row_rect, entry, i, i == len(self._add_matches) - 1)
    return y + group_rect.height

  def _draw_reset_section(self, x: float, y: float, width: float) -> float:
    draw_section_header(rl.Rectangle(x, y, width, self.METRICS.section_header_height), tr("Reset"), style=PANEL_STYLE)
    y += self.METRICS.section_header_height + self.METRICS.section_header_gap
    group_rect = rl.Rectangle(x, y, width, self.ROW_HEIGHT)
    draw_list_group_shell(group_rect, style=PANEL_STYLE)
    self._draw_reset_row(group_rect)
    return y + group_rect.height
