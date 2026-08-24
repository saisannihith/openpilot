from __future__ import annotations

import json

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.list_view import button_item, dual_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller


def _decode(raw: bytes | None) -> str:
  return raw.decode("utf-8", errors="replace") if raw else ""


class CarnivalLayout(Widget):
  """On-device control center for the 2024 Carnival-specific systems."""

  def __init__(self):
    super().__init__()
    self._params = Params()

    self._confidence = toggle_item(
      lambda: tr("Carnival Confidence Governor"),
      description=self._confidence_description,
      initial_state=self._params.get_bool("CarnivalConfidenceGovernor"),
      callback=lambda state: self._params.put_bool("CarnivalConfidenceGovernor", state),
    )
    self._self_tuning = button_item(
      lambda: tr("Self-Tuning Drive Profiles"), lambda: tr("REVIEW"),
      description=self._profile_description, callback=self._show_profile,
    )
    self._fusion_hud = toggle_item(
      lambda: tr("Radar-Vision Lead Fusion HUD"),
      description=self._fusion_description,
      initial_state=self._params.get_bool("CarnivalFusionHUD"),
      callback=lambda state: self._params.put_bool("CarnivalFusionHUD", state),
    )
    self._intersection = toggle_item(
      lambda: tr("Intersection Stop Controller"),
      description=self._intersection_description,
      initial_state=self._params.get_bool("CarnivalIntersectionController"),
      callback=lambda state: self._params.put_bool("CarnivalIntersectionController", state),
    )
    self._eps = toggle_item(
      lambda: tr("EPS Fault Predictor"),
      description=self._eps_description,
      initial_state=self._params.get_bool("CarnivalEPSPredictor"),
      callback=lambda state: self._params.put_bool("CarnivalEPSPredictor", state),
    )
    self._scorecard = button_item(
      lambda: tr("Route Replay Scorecard"), self._scorecard_button,
      description=self._scorecard_description, callback=self._run_analysis,
      enabled=ui_state.is_offroad,
    )
    self._auto_analyze = toggle_item(
      lambda: tr("Analyze Every Completed Drive"),
      description=lambda: tr("Optional heavy route replay. Off by default; long drives can take several minutes. Run Now is recommended."),
      initial_state=self._params.get_bool("CarnivalAutoAnalyze"),
      callback=lambda state: self._params.put_bool("CarnivalAutoAnalyze", state),
    )
    self._auto_apply = toggle_item(
      lambda: tr("Automatically Apply Bounded Suggestions"),
      description=lambda: tr(
        "Only allowlisted follow and stop-distance changes can apply automatically. " +
        "Steering torque, radar velocity, curve speed, and lane offset stay review-only."
      ),
      initial_state=self._params.get_bool("CarnivalAutoTuneApply"),
      callback=self._set_auto_apply,
      enabled=ui_state.is_offroad,
    )
    self._profile_actions = dual_button_item(
      lambda: tr("APPLY SUGGESTION"), lambda: tr("REVERT LAST"),
      left_callback=self._apply_profile, right_callback=self._revert_profile,
      enabled=ui_state.is_offroad,
    )
    self._scroller = Scroller([
      self._confidence,
      self._self_tuning,
      self._fusion_hud,
      self._intersection,
      self._eps,
      self._scorecard,
      self._auto_analyze,
      self._auto_apply,
      self._profile_actions,
    ], line_separator=True, spacing=0)
    ui_state.add_offroad_transition_callback(self._refresh)

  def _json_param(self, key: str) -> dict:
    try:
      value = json.loads(_decode(self._params.get(key, return_default=True)) or "{}")
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

  def _profile_description(self) -> str:
    profile = self._json_param("CarnivalPendingProfile")
    resolved = profile.get("resolved", {})
    if not resolved:
      return tr("No bounded change is pending. Each completed route is still scored and saved for review.")
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

  def _intersection_description(self) -> str:
    state = self._live_state()
    if state is None:
      return tr("Owns only the final low-speed stop, hold, and confirmed release. The driving model remains responsible for the high-speed approach.")
    return tr("State: {state} | Brake hold: {hold}").format(
      state=str(state.stopState), hold=tr("active") if state.stopHoldActive else tr("inactive"),
    )

  def _eps_description(self) -> str:
    state = self._live_state()
    if state is None:
      return tr("Predicts sustained EPS load and applies a small taper before the stronger platform safety guard. Steering-limit warnings remain visible.")
    return tr("EPS risk: {risk}% | Saturation: {sat}% | Torque scale: {scale}%").format(
      risk=round(float(state.epsRisk) * 100), sat=round(float(state.steeringSaturation) * 100),
      scale=round(float(state.torqueScale) * 100),
    )

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
      content += f"<p><b>{tr('Bounded parameter changes')}</b><br>{apply_text}</p>"
    gui_app.push_widget(ConfirmDialog(content, tr("OK"), cancel_text="", rich=True))

  def _run_analysis(self):
    if ui_state.is_offroad():
      self._params.put_bool("CarnivalAnalyzeNow", True)

  def _apply_profile(self):
    if not ui_state.is_offroad():
      return
    profile = self._json_param("CarnivalPendingProfile")
    if not profile.get("resolved"):
      gui_app.push_widget(alert_dialog(tr("There is no bounded suggestion to apply.")))
      return

    def confirmed(result: int):
      if result == DialogResult.CONFIRM:
        self._params.put_bool("CarnivalApplyProfile", True)

    gui_app.push_widget(ConfirmDialog(
      tr("Apply the latest allowlisted Carnival profile? A snapshot will be saved for one-tap revert."),
      tr("Apply"), callback=confirmed,
    ))

  def _revert_profile(self):
    if not ui_state.is_offroad():
      return
    if not self._json_param("CarnivalProfileSnapshot"):
      gui_app.push_widget(alert_dialog(tr("There is no applied Carnival profile to revert.")))
      return
    self._params.put_bool("CarnivalRevertProfile", True)

  def _set_auto_apply(self, state: bool):
    if not state:
      self._params.put_bool("CarnivalAutoTuneApply", False)
      return

    def confirmed(result: int):
      enabled = result == DialogResult.CONFIRM
      self._params.put_bool("CarnivalAutoTuneApply", enabled)
      self._auto_apply.action_item.set_state(enabled)

    gui_app.push_widget(ConfirmDialog(
      tr(
        "Automatically apply only allowlisted follow-time and stop-distance suggestions after completed drives? " +
        "Steering and radar control values remain review-only."
      ),
      tr("Enable"), callback=confirmed,
    ))

  def _refresh(self):
    for key, item in (
      ("CarnivalConfidenceGovernor", self._confidence),
      ("CarnivalFusionHUD", self._fusion_hud),
      ("CarnivalIntersectionController", self._intersection),
      ("CarnivalEPSPredictor", self._eps),
      ("CarnivalAutoAnalyze", self._auto_analyze),
      ("CarnivalAutoTuneApply", self._auto_apply),
    ):
      item.action_item.set_state(self._params.get_bool(key))

  def show_event(self):
    self._refresh()
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)
