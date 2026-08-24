#!/usr/bin/env python3
from __future__ import annotations

import re
import time
from collections import defaultdict
from datetime import UTC, datetime
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

LOG_ROOT = Path("/data/media/0/realdata")
LOG_NAMES = ("qlog", "qlog.zst", "qlog.bz2", "rlog", "rlog.zst", "rlog.bz2")
ROUTE_SETTLE_SECONDS = 45.0
POLL_SECONDS = 10.0


def route_from_segment(name: str) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", name)
  return match.group(1) if match else name


def newest_route(root: Path) -> tuple[str, float] | None:
  newest_by_route: dict[str, float] = defaultdict(float)
  if not root.exists():
    return None
  for segment in root.iterdir():
    if not segment.is_dir():
      continue
    route = route_from_segment(segment.name)
    for name in LOG_NAMES:
      path = segment / name
      if path.is_file():
        newest_by_route[route] = max(newest_by_route[route], path.stat().st_mtime)
        break
  return max(newest_by_route.items(), key=lambda item: item[1]) if newest_by_route else None


def request_new_route(params: Params, root: Path, now: float, last_requested: str = "") -> str:
  if params.get_bool("CarnivalAnalyzeNow") or params.get_bool("CarnivalAnalysisRunning"):
    return last_requested
  latest = newest_route(root)
  if latest is None:
    return last_requested
  route, modified = latest
  completed = params.get("CarnivalLastAnalysisRoute", encoding="utf-8") or ""
  if route != completed and route != last_requested and now - modified >= ROUTE_SETTLE_SECONDS:
    params.put_bool("CarnivalAnalyzeNow", True)
    cloudlog.info("Carnival compact scorecard requested route=%s", route)
    return route
  return last_requested


def main() -> None:
  params = Params()
  last_requested = ""
  while True:
    last_requested = request_new_route(params, LOG_ROOT, datetime.now(UTC).timestamp(), last_requested)
    time.sleep(POLL_SECONDS)


if __name__ == "__main__":
  main()
