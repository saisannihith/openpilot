import json

from openpilot.selfdrive.controls import carnival_analyzerd
from openpilot.selfdrive.controls.carnival_analyzerd import (
  apply_pending_profile,
  discover_routes,
  handle_profile_requests,
  prune_reports,
  scorecard_log_files,
)
from openpilot.tools.carnival.collect_and_report import RouteReport, summarize_compact_radar


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key, block=False, return_default=False, encoding=None, default=None):
    value = self.values.get(key, default)
    if encoding and isinstance(value, bytes):
      return value.decode(encoding)
    return value

  def get_bool(self, key, block=False, default=False):
    value = self.values.get(key, default)
    return value in (True, "1", b"1")

  def put(self, key, value):
    self.values[key] = value

  def put_bool(self, key, value):
    self.values[key] = "1" if value else "0"


def test_discover_routes_groups_segments_and_orders_by_mtime(tmp_path):
  first = tmp_path / "dongle|2026-08-24--aaaa--0"
  second = tmp_path / "dongle|2026-08-24--bbbb--0"
  first.mkdir()
  second.mkdir()
  (first / "qlog").write_bytes(b"one")
  (second / "qlog").write_bytes(b"two")
  routes = discover_routes(tmp_path)
  assert [route for route, _, _ in routes] == ["dongle|2026-08-24--aaaa", "dongle|2026-08-24--bbbb"]


def test_scorecard_prefers_qlog_for_full_drive_with_rlog_fallback(tmp_path):
  first = tmp_path / "dongle|2026-08-24--aaaa--0"
  second = tmp_path / "dongle|2026-08-24--aaaa--1"
  first.mkdir()
  second.mkdir()
  (first / "qlog.zst").write_bytes(b"compact")
  (first / "rlog.zst").write_bytes(b"full")
  (second / "rlog").write_bytes(b"fallback")

  files = scorecard_log_files(tmp_path)
  assert [path.name for path in files] == ["qlog.zst", "rlog"]


def test_apply_and_revert_pending_profile():
  profile = {
    "route": "route",
    "resolved": {"StandardFollow": {"before": 1.45, "after": 1.5}},
  }
  params = FakeParams({"CarnivalPendingProfile": json.dumps(profile)})
  assert apply_pending_profile(params)
  assert params.values["StandardFollow"] == "1.5"
  params.put_bool("CarnivalRevertProfile", True)
  handle_profile_requests(params)
  assert params.values["StandardFollow"] == "1.45"
  assert params.values["CarnivalProfileSnapshot"] == {}


def test_json_params_use_native_typed_values():
  payload = {"overall_score": 91}
  params = FakeParams({"CarnivalLastScorecard": payload})
  assert carnival_analyzerd._json_param(params, "CarnivalLastScorecard") == payload
  carnival_analyzerd._write_json_param(params, "CarnivalLastScorecard", {"overall_score": 92})
  assert params.values["CarnivalLastScorecard"] == {"overall_score": 92}


def test_report_retention_removes_old_pairs(tmp_path):
  for index in range(3):
    for suffix in ("json", "md"):
      (tmp_path / f"carnival-report-20260824-00000{index}.{suffix}").write_text("report")
  prune_reports(tmp_path, keep=2)
  assert len(list(tmp_path.glob("*.json"))) == 2
  assert len(list(tmp_path.glob("*.md"))) == 2


def test_compact_radar_summary_uses_fused_state_only():
  metrics = summarize_compact_radar(100, 50, 40, 5, 3)
  assert metrics.mode == "compact"
  assert metrics.coverage == 0.8
  assert metrics.state_samples == 100
  assert metrics.stale_samples == 5
  assert metrics.cut_in_samples == 3
  assert metrics.distance_mae is None


def test_compact_radar_summary_falls_back_to_logged_radar_state():
  metrics = summarize_compact_radar(0, 0, 0, 0, 0, fallback_vision_samples=60, fallback_radar_samples=40)
  assert metrics.mode == "compact-fallback"
  assert metrics.coverage == 0.4
  assert metrics.refs == 100
  assert metrics.selected == 40


def test_analyzer_always_uses_compact_replay(monkeypatch, tmp_path):
  calls = []

  def fake_analyze(route, files, *, compact=False):
    calls.append((route, files, compact))
    return RouteReport(route=route, files=len(files), analysis_mode="compact")

  monkeypatch.setattr(carnival_analyzerd, "analyze_route", fake_analyze)
  monkeypatch.setattr(carnival_analyzerd, "REPORT_ROOT", tmp_path)
  monkeypatch.setattr(carnival_analyzerd, "write_markdown", lambda *args: None)
  params = FakeParams()
  carnival_analyzerd.analyze_completed_route(params, "route", [tmp_path / "qlog"])
  assert calls == [("route", [tmp_path / "qlog"], True)]
  assert not params.get_bool("CarnivalAnalysisRunning")
