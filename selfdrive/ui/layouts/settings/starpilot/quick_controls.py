from __future__ import annotations

import json
from collections.abc import Callable

from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.hardware import HARDWARE
from openpilot.system.ui.lib.application import gui_app

from openpilot.selfdrive.ui.lib.starpilot_state import starpilot_state
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import _SettingsPage, StarPilotPanelInfo, StarPilotPanelType
from openpilot.selfdrive.ui.layouts.settings.starpilot.settings_index import (
  SettingsEntry,
  build_settings_index,
  row_visible,
)
from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import (
  AetherSettingsView,
  DEFAULT_PANEL_STYLE,
  SettingRow,
  SettingSection,
)
from openpilot.starpilot.common.longitudinal_mode import (
  set_alpha_longitudinal,
  set_experimental_mode,
  set_openpilot_long_disabled,
)


PANEL_STYLE = DEFAULT_PANEL_STYLE
QUICK_CONTROL_ORDER_PARAM = "StarPilotQuickControlOrder"
STOCK_ID_PREFIX = "stock:"


DEFAULT_QUICK_CONTROL_IDS = [
  "stock:AlphaLongitudinalEnabled",
  "stock:ExperimentalMode",
  "stock:DisableOpenpilotLongitudinal",
  "stock:AlwaysOnLateral",
  "stock:LaneCentering",
  "stock:RadarTracksUI",
  "stock:LeadInfo",
  "stock:ShowStoppingPoint",
  "stock:ShowStoppingPointMetrics",
  "stock:ModelUI",
  "stock:CustomUI",
  "stock:DeveloperUI",
  "stock:DeveloperMetrics",
  "stock:DeveloperSidebar",
]


class StarPilotQuickControlsLayout(_SettingsPage):
  def __init__(self, panel_provider: Callable[[], dict[StarPilotPanelType, StarPilotPanelInfo]] | None = None):
    super().__init__()
    self._panel_provider = panel_provider
    self._stock_sections: list[SettingSection] = []
    self._stock_rows: dict[str, SettingRow] = {}
    self._settings_entries: list[SettingsEntry] = []
    self._settings_rows: dict[str, SettingsEntry] = {}
    self._build_stock_rows()
    self._refresh_index()
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

  def _refresh_index(self) -> None:
    if self._panel_provider is None:
      self._settings_entries = []
      self._settings_rows = {}
      return
    self._settings_entries = build_settings_index(
      self._panel_provider,
      exclude_panel_types={StarPilotPanelType.MAIN, StarPilotPanelType.QUICK_CONTROLS, StarPilotPanelType.SEARCH},
    )
    self._settings_rows = {entry.stable_id: entry for entry in self._settings_entries}

  def _load_control_ids(self) -> list[str]:
    raw = self._params.get(QUICK_CONTROL_ORDER_PARAM)
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

  def _save_control_ids(self, control_ids: list[str]) -> None:
    deduped: list[str] = []
    seen: set[str] = set()
    for control_id in control_ids:
      if control_id in seen:
        continue
      seen.add(control_id)
      deduped.append(control_id)
    self._params.put(QUICK_CONTROL_ORDER_PARAM, json.dumps(deduped, separators=(",", ":")))
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
    self._build_view()

  def refresh(self) -> None:
    self._build_view()

  def show_event(self):
    self._build_view()
    super().show_event()

  def _edit_current(self, control_id: str) -> None:
    options = [tr("Move Up"), tr("Move Down"), tr("Remove")]
    dialog = MultiOptionDialog(
      self._owner._control_label(control_id),
      options,
      current=options[0],
      callback=lambda result: self._on_edit_result(result, dialog, control_id),
    )
    gui_app.push_widget(dialog)

  def _on_edit_result(self, result: DialogResult, dialog: MultiOptionDialog, control_id: str) -> None:
    if result != DialogResult.CONFIRM:
      return
    if dialog.selection == tr("Move Up"):
      self._owner.move_control(control_id, -1)
    elif dialog.selection == tr("Move Down"):
      self._owner.move_control(control_id, 1)
    elif dialog.selection == tr("Remove"):
      self._owner.remove_control(control_id)

  def _reset(self) -> None:
    def on_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        self._owner.reset_controls()

    gui_app.push_widget(ConfirmDialog(
      tr("Reset Quick Controls to default?"),
      tr("Reset"),
      callback=on_confirm,
    ))

  def _build_view(self) -> None:
    current_rows = [
      SettingRow(
        f"current:{control_id}", "action", self._owner._control_label(control_id),
        subtitle=self._owner._control_subtitle(control_id),
        action_text=tr_noop("Edit"),
        on_click=lambda cid=control_id: self._edit_current(cid),
      )
      for control_id in self._owner.selected_control_ids()
    ]

    available_rows = [
      SettingRow(
        f"add:{control_id}", "action", row.title,
        subtitle=path,
        action_text=tr_noop("Add"),
        on_click=lambda cid=control_id: self._owner.add_control(cid),
      )
      for control_id, row, path in self._owner.available_control_entries()
    ]

    manage_rows = [
      SettingRow(
        "ResetQuickControls", "action", tr_noop("Reset to Default"),
        subtitle=tr_noop("Restore the stock Quick Controls layout."),
        action_text=tr_noop("Reset"),
        action_danger=True,
        on_click=self._reset,
      ),
    ]

    self._manager_view = AetherSettingsView(
      self,
      [
        SettingSection(tr_noop("Current Order"), current_rows),
        SettingSection(tr_noop("Add Settings"), available_rows),
        SettingSection(tr_noop("Reset"), manage_rows),
      ],
      header_title=tr_noop("Customize Quick Controls"),
      header_subtitle=tr_noop("Move, remove, or add StarPilot settings. Changes save immediately."),
      panel_style=PANEL_STYLE,
    )
