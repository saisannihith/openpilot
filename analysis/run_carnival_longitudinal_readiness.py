#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import ReadMode

from scan_longitudinal_quality import analyze, expand_logs, read_samples_and_metadata


DEFAULT_REALDATA = Path("/data/media/0/realdata")


def recent_route_patterns(root: Path, count: int) -> list[str]:
  if not root.exists():
    return []

  route_mtimes: dict[str, float] = {}
  for path in root.iterdir():
    if not path.is_dir():
      continue
    parts = path.name.split("--")
    if len(parts) < 3:
      continue
    route = "--".join(parts[:2])
    route_mtimes[route] = max(route_mtimes.get(route, 0.0), path.stat().st_mtime)

  routes = [
    route for route, _mtime in
    sorted(route_mtimes.items(), key=lambda item: item[1], reverse=True)[:count]
  ]
  return [str(root / f"{route}--*" / "qlog*") for route in routes]


def severity_rank(status: str) -> int:
  return {"pass": 0, "warn": 1, "fail": 2}[status]


def add_check(checks: list[dict[str, Any]], name: str, status: str, detail: str, evidence: Any = None) -> None:
  check: dict[str, Any] = {"name": name, "status": status, "detail": detail}
  if evidence is not None:
    check["evidence"] = evidence
  checks.append(check)


def add_request(requests: list[dict[str, str]], scenario: str, reason: str, target: str) -> None:
  requests.append({
    "scenario": scenario,
    "reason": reason,
    "target": target,
  })


def current_git_commit() -> str:
  try:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
  except Exception:
    return "unknown"


def commit_matches(current: str, logged: str | None) -> bool:
  if not current or current == "unknown" or not logged or logged == "unknown":
    return False
  return current.startswith(logged) or logged.startswith(current)


def build_report(scan: dict[str, Any], patterns: list[str], files: int, *, included_files: int, excluded_stale_files: int,
                 include_stale: bool) -> dict[str, Any]:
  checks: list[dict[str, Any]] = []
  coverage_gaps: list[str] = []
  next_drive_requests: list[dict[str, str]] = []
  current_commit = current_git_commit()
  software = scan.get("software", [])
  matching_commits = [
    item for item in software
    if commit_matches(current_commit, item.get("gitCommit")) or commit_matches(current_commit, item.get("gitSrcCommit"))
  ]
  matching_log_files = sum(int(item.get("files", 0)) for item in matching_commits)
  stale_log_files = max(0, files - matching_log_files)
  dirty_logs = [item for item in software if item.get("dirty")]

  if not software:
    coverage_gaps.append("Scanned logs did not contain initData software metadata.")
  elif not matching_commits:
    coverage_gaps.append("No scanned log software commit matches the current checkout commit.")
  elif stale_log_files:
    coverage_gaps.append(f"{stale_log_files} scanned log files were recorded on older software commits.")
  if dirty_logs:
    coverage_gaps.append("At least one scanned log was recorded from a dirty checkout.")

  no_context = scan.get("noContextHighwayHardBrakes", [])
  add_check(
    checks,
    "no_context_highway_hard_brakes",
    "pass" if not no_context else "fail",
    f"{len(no_context)} hard-brake samples without lead, red light, force stop, shouldStop, or model stop context.",
    no_context[:5],
  )

  jumps = scan.get("accelJumps", [])
  unexplained_jumps = [
    jump for jump in jumps
    if not jump.get("lead") and not jump.get("redLight") and not jump.get("forcingStop") and not jump.get("shouldStop")
  ]
  add_check(
    checks,
    "unexplained_planner_accel_jumps",
    "pass" if not unexplained_jumps else "fail",
    f"{len(unexplained_jumps)} large planner acceleration jumps without lead/stop context.",
    unexplained_jumps[:5],
  )

  gate = scan.get("currentRedLightGateAudit", {})
  enabled_strong = int(gate.get("allowedStrongBrakeLongActiveEnabledFrames", 0))
  longactive_strong = int(gate.get("allowedStrongBrakeLongActiveFrames", 0))
  suppressed_old = int(gate.get("loggedBelowCurrentCapFrames", 0))
  if enabled_strong:
    red_status = "fail"
  elif longactive_strong:
    red_status = "warn"
  else:
    red_status = "pass"
  add_check(
    checks,
    "current_red_light_gate",
    red_status,
    (
      f"{enabled_strong} enabled long-active strong-brake frames still allowed by current gate; "
      f"{longactive_strong} total long-active allowed frames; {suppressed_old} old logged frames would be capped now."
    ),
    {
      "capAccel": gate.get("capAccel"),
      "allowedStrongBrakeExamples": gate.get("allowedStrongBrakeExamples", [])[:5],
      "suppressedExamples": gate.get("suppressedExamples", [])[:5],
    },
  )

  lead_departures = scan.get("leadDepartureOpportunities", [])
  manual_before_move = [event for event in lead_departures if event.get("manualOverrideBeforeMoveDelay") is not None]
  slow_or_missing_move = [
    event for event in lead_departures
    if event.get("egoMoveDelay") is None or float(event.get("egoMoveDelay", 99.0)) > 2.5
  ]
  if manual_before_move or slow_or_missing_move:
    lead_status = "fail"
  elif not lead_departures:
    lead_status = "warn"
    coverage_gaps.append("No current-log lead-departure opportunity was found.")
    add_request(
      next_drive_requests,
      "stop_and_go_lead_departure",
      "Validate that the car resumes without gas after a stopped lead begins moving.",
      "Stop behind a lead vehicle, keep longitudinal alpha active, and let the lead move first without pressing gas unless intervention is needed.",
    )
  else:
    lead_status = "pass"
  add_check(
    checks,
    "lead_departure_auto_resume",
    lead_status,
    (
      f"{len(lead_departures)} opportunities; {len(manual_before_move)} manual overrides before movement; "
      f"{len(slow_or_missing_move)} slow/missing ego launches."
    ),
    lead_departures[:8],
  )

  stop_context = scan.get("stopContextHighwayHardBrakes", [])
  enabled_stop_context = [sample for sample in stop_context if sample.get("enabled")]
  add_check(
    checks,
    "stop_context_highway_hard_brakes",
    "pass" if not enabled_stop_context else "warn",
    f"{len(enabled_stop_context)} enabled high-speed hard-brake samples with red-light/stop context.",
    enabled_stop_context[:5],
  )

  stop_episodes = scan.get("stopEpisodes", [])
  completed_stops = [
    episode for episode in stop_episodes
    if episode.get("standstillT") is not None
  ]
  if stop_episodes and not completed_stops:
    coverage_gaps.append("Stop-context episodes were present, but none reached logged standstill.")
    add_request(
      next_drive_requests,
      "complete_red_light_or_stopped_lead_stop",
      "Validate full stop distance and smooth final braking, not just initial slowdown.",
      "Let longitudinal alpha handle one clear red-light or stopped-lead approach to a complete stop, with hands/feet ready.",
    )
  add_check(
    checks,
    "stop_episode_coverage",
    "pass" if stop_episodes else "warn",
    f"{len(stop_episodes)} stop-context episodes found in scanned logs.",
    stop_episodes[:5],
  )

  overall = "pass"
  for check in checks:
    if severity_rank(check["status"]) > severity_rank(overall):
      overall = check["status"]

  if not scan.get("routes"):
    coverage_gaps.append("No readable qlog samples were found.")
  if excluded_stale_files:
    coverage_gaps.append(f"{excluded_stale_files} stale log files were excluded from current-code scoring.")

  if overall == "pass" and coverage_gaps:
    overall = "warn"

  return {
    "status": overall,
    "currentCommit": current_commit,
    "filesScanned": files,
    "filesIncluded": included_files,
    "includeStaleLogs": include_stale,
    "excludedStaleFiles": excluded_stale_files,
    "patterns": patterns,
    "routes": scan.get("routes", []),
    "samples": scan.get("samples", 0),
    "logSoftware": software,
    "matchingLogCommits": matching_commits,
    "matchingLogFiles": matching_log_files,
    "staleLogFiles": stale_log_files,
    "coverageGaps": coverage_gaps,
    "nextDriveRequests": next_drive_requests,
    "checks": checks,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description="Produce a PASS/WARN/FAIL longitudinal readiness report for Carnival logs.")
  parser.add_argument("logs", nargs="*", help="qlog paths/globs. If omitted on-device, scans the newest routes.")
  parser.add_argument("--recent-routes", type=int, default=4, help="Number of newest route ids to scan when logs are omitted.")
  parser.add_argument("--include-stale", action="store_true", help="Include logs recorded on older commits in scoring.")
  parser.add_argument("--out", type=Path)
  parser.add_argument("--strict-warn", action="store_true", help="Return nonzero for WARN as well as FAIL.")
  args = parser.parse_args()

  patterns = args.logs or recent_route_patterns(DEFAULT_REALDATA, args.recent_routes)
  if not patterns:
    print(json.dumps({
      "status": "fail",
      "error": "No log patterns supplied and no on-device realdata routes found.",
    }, indent=2, sort_keys=True))
    return 1

  paths = expand_logs(patterns)
  current_commit = current_git_commit()
  samples = []
  software_metadata = []
  included_files = 0
  excluded_stale_files = 0
  for path in paths:
    path_samples, software = read_samples_and_metadata(path, ReadMode.QLOG)
    if software is not None:
      software_metadata.append(software)
    software_matches = (
      software is not None and (
        commit_matches(current_commit, software.get("gitCommit")) or
        commit_matches(current_commit, software.get("gitSrcCommit"))
      )
    )
    if args.include_stale or software_matches:
      samples.extend(path_samples)
      included_files += 1
    else:
      excluded_stale_files += 1

  report = build_report(
    analyze(samples, software_metadata), patterns, len(paths),
    included_files=included_files, excluded_stale_files=excluded_stale_files,
    include_stale=args.include_stale,
  )
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out is not None:
    args.out.write_text(text + "\n")

  if report["status"] == "fail" or (args.strict_warn and report["status"] == "warn"):
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
