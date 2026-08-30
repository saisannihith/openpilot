from __future__ import annotations

import json

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.list_view import button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller


def _decode(raw: bytes | None) -> str:
  return raw.decode("utf-8", errors="replace") if raw else ""


class CarnivalLayout(Widget):
  """On-device control center for the 2024 Carnival-specific systems."""

  def __init__(self):
    super().__init__()
    self._params = Params()

    self._master = toggle_item(
      lambda: tr("Carnival Enhancements"),
      description=lambda: tr(
        "Master switch for Carnival monitoring and optional assistance tools. Saved child choices are restored when re-enabled. " +
        "Vehicle identification, CAN safety, steering limits, and the base EPS fault guard always remain active."
      ),
      initial_state=self._features_enabled(),
      callback=self._set_master,
    )
    self._confidence = button_item(
      lambda: tr("Carnival Confidence Monitor"), lambda: tr("VIEW"),
      description=self._confidence_description,
      callback=self._show_confidence,
      enabled=self._features_enabled,
    )
    self._self_tuning = button_item(
      lambda: tr("Self-Tuning Drive Profiles"), lambda: tr("REVIEW"),
      description=self._profile_description, callback=self._show_profile,
      enabled=self._features_enabled,
    )
    self._fusion_hud = toggle_item(
      lambda: tr("Radar-Vision Lead Fusion HUD"),
      description=self._fusion_description,
      initial_state=self._params.get_bool("CarnivalFusionHUD"),
      callback=lambda state: self._set_feature("CarnivalFusionHUD", state),
      enabled=self._features_enabled,
    )
    self._eps = button_item(
      lambda: tr("EPS Load Monitor"), lambda: tr("VIEW"),
      description=self._eps_description,
      callback=self._show_eps,
      enabled=self._features_enabled,
    )
    self._scorecard = button_item(
      lambda: tr("Route Replay Scorecard"), self._scorecard_button,
      description=self._scorecard_description, callback=self._run_analysis,
      enabled=lambda: self._features_enabled() and ui_state.is_offroad(),
    )
    self._auto_analyze = toggle_item(
      lambda: tr("Analyze Every Completed Drive"),
      description=lambda: tr(
        "Runs a compact qlog score after each completed drive while parked. Deep raw-radar replay stays PC-only."
      ),
      initial_state=self._params.get_bool("CarnivalAutoAnalyze"),
      callback=lambda state: self._set_feature("CarnivalAutoAnalyze", state),
      enabled=self._features_enabled,
    )
    self._scroller = Scroller([
      self._master,
      self._confidence,
      self._self_tuning,
      self._fusion_hud,
      self._eps,
      self._scorecard,
      self._auto_analyze,
    ], line_separator=True, spacing=0)
    ui_state.add_offroad_transition_callback(self._refresh)

  def _features_enabled(self) -> bool:
    return self._params.get_bool("CarnivalFeaturesEnabled")

  def _set_feature(self, key: str, state: bool) -> None:
    if self._features_enabled():
      self._params.put_bool(key, state)

  def _set_master(self, state: bool) -> None:
    self._params.put_bool("CarnivalFeaturesEnabled", state)
    if not state:
      for key in ("CarnivalAnalyzeNow", "CarnivalAnalysisRunning"):
        self._params.put_bool(key, False)
    self._refresh()

  def _json_param(self, key: str) -> dict:
    try:
      raw = self._params.get(key, return_default=True)
      if isinstance(raw, dict):
        return raw
      value = json.loads(_decode(raw) or "{}")
      return value if isinstance(value, dict) else {}
    except (TypeError, ValueError):
      return {}

  def _live_state(self):
    sm = ui_state.sm
    if sm.valid.get("carnivalState", False):
      return sm["carnivalState"]
    return None

  def _confidence_description(self) -> str:
    state = self._live_state()
    if state is None:
      return tr("Fuses road geometry, lead confirmation, steering load, EPS risk, and driver interventions. Live values appear while driving.")
    return tr("Overall: {overall}% | Lateral: {lateral}% | Longitudinal: {longitudinal}% | State: {status}<br>{reason}").format(
      overall=round(float(state.overallConfidence) * 100), lateral=round(float(state.lateralConfidence) * 100),
      longitudinal=round(float(state.longitudinalConfidence) * 100), status=str(state.governorState), reason=str(state.reason),
    )

  def _show_confidence(self):
    gui_app.push_widget(alert_dialog(self._confidence_description()))

  def _profile_description(self) -> str:
    profile = self._json_param("CarnivalPendingProfile")
    resolved = profile.get("resolved", {})
    if not resolved:
      return tr("No parameter suggestion is pending. Each completed route is scored and saved for review.")
    summary = ", ".join(f"{key}: {values.get('before')} to {values.get('after')}" for key, values in resolved.items())
    return tr("Pending from {route}: {summary}").format(route=profile.get("route", tr("latest route")), summary=summary)

  def _fusion_description(self) -> str:
    state = self._live_state()
    if state is None:
      return tr("Shows whether the lead is vision-only, radar-confirmed, stale, or a possible cut-in. Radar remains confirmation for model-led control.")
    return tr("Lead: {source} | Distance: {distance:.1f} m | Track: {track} | Cut-ins: {cutins} | Radar: {radar}").format(
      source=str(state.leadSource), distance=float(state.leadDistance), track=int(state.radarTrackId),
      cutins=int(state.cutInCandidateCount), radar=tr("stale") if state.radarStale else tr("live"),
    )

  def _eps_description(self) -> str:
    state = self._live_state()
    if state is None:
      return tr("Reports sustained steering load and saturation for route diagnosis. It never modifies steering commands or hides warnings.")
    return tr("EPS risk: {risk}% | Saturation: {sat}% | Monitoring only").format(
      risk=round(float(state.epsRisk) * 100), sat=round(float(state.steeringSaturation) * 100),
    )

  def _show_eps(self):
    gui_app.push_widget(alert_dialog(self._eps_description()))

  def _scorecard_button(self) -> str:
    return tr("RUNNING") if self._params.get_bool("CarnivalAnalysisRunning") else tr("RUN NOW")

  def _scorecard_description(self) -> str:
    card = self._json_param("CarnivalLastScorecard")
    error = _decode(self._params.get("CarnivalAnalysisError", return_default=True))
    if error:
      return tr("Last analysis error: {error}").format(error=error)
    if not card:
      return tr("No route has been scored yet. Run analysis while parked; live Carnival telemetry is logged automatically while driving.")
    return tr(
      "Overall {overall} | Lateral {lateral} | Longitudinal {longitudinal} | Radar {radar}<br>" +
      "Interventions {interventions} | False brakes {false_brakes} | Missed stops {missed} | Torque saturation {saturation}"
    ).format(
      overall=card.get("overall_score", 0), lateral=card.get("lateral_score", 0),
      longitudinal=card.get("longitudinal_score", 0), radar=card.get("radar_score", 0),
      interventions=card.get("intervention_events", 0), false_brakes=card.get("false_brake_events", 0),
      missed=card.get("missed_stop_events", 0), saturation=card.get("torque_saturation_events", 0),
    )

  def _show_profile(self):
    profile = self._json_param("CarnivalPendingProfile")
    card = self._json_param("CarnivalLastScorecard")
    recommendations = card.get("recommendations", [])
    recommendation_text = "<br>".join(
      f"<b>{item.get('target', '')}</b>: {item.get('delta', '')} ({item.get('confidence', '')})<br>{item.get('reason', '')}"
      for item in recommendations
    ) or tr("No change is justified by the latest route.")
    resolved = profile.get("resolved", {})
    apply_text = "<br>".join(f"{key}: {value.get('before')} to {value.get('after')}" for key, value in resolved.items())
    content = f"<h1>{tr('Carnival Drive Profile')}</h1><p>{recommendation_text}</p>"
    if apply_text:
      content += f"<p><b>{tr('Informational parameter suggestions')}</b><br>{apply_text}</p>"
      content += f"<p>{tr('Suggestions are never applied automatically. StarPilot remains the sole owner of driving behavior.')}</p>"
    gui_app.push_widget(ConfirmDialog(content, tr("OK"), cancel_text="", rich=True))

  def _run_analysis(self):
    if self._features_enabled() and ui_state.is_offroad():
      self._params.put_bool("CarnivalAnalyzeNow", True)

  def _refresh(self):
    enabled = self._features_enabled()
    self._master.action_item.set_state(enabled)
    for key, item in (
      ("CarnivalFusionHUD", self._fusion_hud),
      ("CarnivalAutoAnalyze", self._auto_analyze),
    ):
      item.action_item.set_state(enabled and self._params.get_bool(key))

  def show_event(self):
    self._refresh()
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)
