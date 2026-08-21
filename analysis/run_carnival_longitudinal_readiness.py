#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import ReadMode

from scan_longitudinal_quality import analyze, expand_logs, read_samples


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


def build_report(scan: dict[str, Any], patterns: list[str], files: int) -> dict[str, Any]:
  checks: list[dict[str, Any]] = []

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

  return {
    "status": overall,
    "filesScanned": files,
    "patterns": patterns,
    "routes": scan.get("routes", []),
    "samples": scan.get("samples", 0),
    "checks": checks,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description="Produce a PASS/WARN/FAIL longitudinal readiness report for Carnival logs.")
  parser.add_argument("logs", nargs="*", help="qlog paths/globs. If omitted on-device, scans the newest routes.")
  parser.add_argument("--recent-routes", type=int, default=4, help="Number of newest route ids to scan when logs are omitted.")
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
  samples = []
  for path in paths:
    samples.extend(read_samples(path, ReadMode.QLOG))

  report = build_report(analyze(samples), patterns, len(paths))
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out is not None:
    args.out.write_text(text + "\n")

  if report["status"] == "fail" or (args.strict_warn and report["status"] == "warn"):
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
