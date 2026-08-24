from __future__ import annotations

from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.hardware import HARDWARE
from openpilot.system.ui.lib.application import gui_app

from openpilot.selfdrive.ui.lib.starpilot_state import starpilot_state
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import _SettingsPage
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


class StarPilotQuickControlsLayout(_SettingsPage):
  def __init__(self):
    super().__init__()
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

  def _build_view(self) -> None:
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

    self._manager_view = AetherSettingsView(
      self,
      [
        SettingSection(tr_noop("Driving"), drive_rows),
        SettingSection(tr_noop("Onroad Display"), visual_rows),
        SettingSection(tr_noop("Developer"), developer_rows),
      ],
      header_title=tr_noop("Quick Controls"),
      header_subtitle=tr_noop("Common driving, radar, model, and developer toggles in one place."),
      panel_style=PANEL_STYLE,
    )
