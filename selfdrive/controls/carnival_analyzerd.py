#!/usr/bin/env python3
from __future__ import annotations

from collections import defaultdict
from dataclasses import asdict
from datetime import UTC, datetime
import json
from pathlib import Path
import time

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.tools.carnival.collect_and_report import analyze_route, route_key, segment_index, write_markdown
from openpilot.tools.carnival.self_tune_profile import (
  apply_resolved_plan,
  build_delta_plan,
  read_param_values,
  resolve_values,
  revert_snapshot,
)


LOG_ROOT = Path("/data/media/0/realdata")
REPORT_ROOT = Path("/data/media/0/carnival_reports")
ROUTE_SETTLE_SECONDS = 45.0
POLL_SECONDS = 3.0
MAX_STORED_REPORTS = 40
LOG_NAMES = {"rlog", "rlog.zst", "rlog.bz2", "qlog", "qlog.zst", "qlog.bz2"}


def _json_param(params: Params, key: str) -> dict:
  try:
    raw = params.get(key, return_default=True)
    payload = json.loads(raw.decode() if isinstance(raw, bytes) else str(raw or "{}"))
    return payload if isinstance(payload, dict) else {}
  except (TypeError, ValueError):
    return {}


def scorecard_log_files(root: Path) -> list[Path]:
  files_by_segment: dict[Path, list[Path]] = defaultdict(list)
  for path in root.rglob("*"):
    if path.name in LOG_NAMES:
      files_by_segment[path.parent].append(path)

  selected = []
  for files in files_by_segment.values():
    qlogs = [path for path in files if path.name.startswith("qlog")]
    selected.extend(qlogs if qlogs else files)
  return sorted(selected, key=lambda path: (route_key(path, root), segment_index(path), path.name))


def discover_routes(root: Path) -> list[tuple[str, list[Path], float]]:
  grouped: dict[str, list[Path]] = defaultdict(list)
  for path in scorecard_log_files(root):
    grouped[route_key(path, root)].append(path)
  routes = []
  for route, files in grouped.items():
    newest = max(path.stat().st_mtime for path in files)
    routes.append((route, sorted(files), newest))
  return sorted(routes, key=lambda item: item[2])


def _write_json_param(params: Params, key: str, payload: dict) -> None:
  params.put(key, json.dumps(payload, separators=(",", ":")))


def prune_reports(root: Path, keep: int = MAX_STORED_REPORTS) -> None:
  stems = sorted({path.stem for path in root.glob("carnival-report-*.*")})
  expired = stems if keep <= 0 else stems[:-keep]
  for stem in expired:
    for path in root.glob(f"{stem}.*"):
      path.unlink(missing_ok=True)


def resolve_local_profile(params: Params, report_payload: dict) -> dict:
  deltas = build_delta_plan(report_payload)
  current = read_param_values(params, deltas)
  return {
    "route": report_payload.get("route", ""),
    "deltas": deltas,
    "resolved": resolve_values(current, deltas),
    "applied": False,
    "createdAt": datetime.now(UTC).isoformat(),
  }


def apply_pending_profile(params: Params, *, automatic: bool = False) -> bool:
  profile = _json_param(params, "CarnivalPendingProfile")
  resolved = profile.get("resolved", {})
  if not isinstance(resolved, dict) or not resolved:
    return False
  snapshot = apply_resolved_plan(params, resolved)
  snapshot.update({"route": profile.get("route", ""), "appliedAt": datetime.now(UTC).isoformat()})
  _write_json_param(params, "CarnivalProfileSnapshot", snapshot)
  profile["applied"] = True
  profile["automatic"] = automatic
  profile["appliedAt"] = snapshot["appliedAt"]
  _write_json_param(params, "CarnivalPendingProfile", profile)
  return True


def handle_profile_requests(params: Params) -> None:
  if params.get_bool("CarnivalApplyProfile"):
    params.put_bool("CarnivalApplyProfile", False)
    apply_pending_profile(params)
  if params.get_bool("CarnivalRevertProfile"):
    params.put_bool("CarnivalRevertProfile", False)
    snapshot = _json_param(params, "CarnivalProfileSnapshot")
    if revert_snapshot(params, snapshot):
      params.put("CarnivalProfileSnapshot", "{}")


def analyze_completed_route(params: Params, route: str, files: list[Path]) -> None:
  params.put_bool("CarnivalAnalysisRunning", True)
  params.put("CarnivalAnalysisError", "")
  try:
    report = analyze_route(route, files)
    payload = asdict(report)
    scorecard = dict(payload.get("scorecard", {}))
    scorecard.update({
      "route": route,
      "analyzedAt": datetime.now(UTC).isoformat(),
      "files": len(files),
    })
    profile = resolve_local_profile(params, payload)

    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    report_json = REPORT_ROOT / f"carnival-report-{timestamp}.json"
    report_md = REPORT_ROOT / f"carnival-report-{timestamp}.md"
    report_json.write_text(json.dumps([payload], indent=2) + "\n", encoding="utf-8")
    write_markdown([report], report_md, LOG_ROOT)
    prune_reports(REPORT_ROOT)

    _write_json_param(params, "CarnivalLastScorecard", scorecard)
    _write_json_param(params, "CarnivalPendingProfile", profile)
    params.put("CarnivalLastAnalysisRoute", route)
    params.put("CarnivalLastAnalysisTime", scorecard["analyzedAt"])
    params.put("CarnivalLastReportPath", str(report_json))
    if params.get_bool("CarnivalAutoTuneApply") and profile.get("resolved"):
      apply_pending_profile(params, automatic=True)
    cloudlog.info("Carnival route scorecard complete route=%s overall=%s files=%d", route, scorecard.get("overall_score"), len(files))
  except Exception as exc:
    params.put("CarnivalAnalysisError", f"{type(exc).__name__}: {exc}"[:300])
    cloudlog.exception("Carnival route scorecard failed route=%s", route)
  finally:
    params.put_bool("CarnivalAnalysisRunning", False)


def main() -> None:
  params = Params()
  params.put_bool("CarnivalAnalysisRunning", False)
  while True:
    handle_profile_requests(params)
    force = params.get_bool("CarnivalAnalyzeNow")
    if force:
      params.put_bool("CarnivalAnalyzeNow", False)

    if force or params.get_bool("CarnivalAutoAnalyze"):
      routes = discover_routes(LOG_ROOT) if LOG_ROOT.exists() else []
      if routes:
        route, files, newest = routes[-1]
        already_done = route == (params.get("CarnivalLastAnalysisRoute", encoding="utf-8") or "")
        settled = (datetime.now().timestamp() - newest) >= ROUTE_SETTLE_SECONDS
        if force or (not already_done and settled):
          analyze_completed_route(params, route, files)
    time.sleep(POLL_SECONDS)


if __name__ == "__main__":
  main()
