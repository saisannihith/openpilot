#!/usr/bin/env python3
"""Index local comma routes by CarParams longitudinal mode using one log per route."""

from __future__ import annotations

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader, ReadMode


LATERAL_PARAM_KEYS = (
  "AdvancedLateralTune", "NNFF", "NNFFLite", "ForceTorqueController",
  "SteerFriction", "SteerKP", "SteerLatAccel", "SteerRatio",
  "SteerFrictionStock", "SteerKPStock", "SteerLatAccelStock", "SteerRatioStock",
  "LaneCentering", "LaneCenteringRoadAware", "LaneCenteringE2EAuthority",
)


def init_params(init_data):
  values = {}
  try:
    for entry in init_data.params.entries:
      key = str(entry.key)
      if key in LATERAL_PARAM_KEYS:
        values[key] = bytes(entry.value).decode("utf-8", errors="replace")
  except Exception:
    pass
  return values


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("root", type=Path)
  args = parser.parse_args()

  routes = defaultdict(list)
  for path in args.root.glob("*/*"):
    if path.name in ("qlog.zst", "qlog.bz2", "qlog", "rlog.zst", "rlog.bz2", "rlog"):
      routes[route_name(path)].append(path)

  report = {}
  for route, paths in sorted(routes.items()):
    preferred = sorted(path for path in paths if path.name.startswith("qlog")) or sorted(paths)
    result = {"file": str(preferred[0]), "carParams": None, "software": None, "lateralParams": None}
    try:
      for msg in LogReader(str(preferred[0]), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
        if msg.which() == "carParams" and result["carParams"] is None:
          cp = msg.carParams
          result["carParams"] = {
            "carFingerprint": str(cp.carFingerprint),
            "openpilotLongitudinalControl": bool(cp.openpilotLongitudinalControl),
            "pcmCruise": bool(cp.pcmCruise),
            "radarUnavailable": bool(cp.radarUnavailable),
            "flags": int(cp.flags),
          }
        elif msg.which() == "initData" and result["software"] is None:
          result["software"] = {
            "commit": str(msg.initData.gitCommit),
            "branch": str(msg.initData.gitBranch),
          }
          result["lateralParams"] = init_params(msg.initData)
        if result["carParams"] is not None and result["software"] is not None:
          break
    except Exception as error:
      result["error"] = str(error)
    report[route] = result

  print(json.dumps(report, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
