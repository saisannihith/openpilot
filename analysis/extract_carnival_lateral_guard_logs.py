#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader, ReadMode


PATTERNS = (
  "Carnival 4th gen manual-turn EPS guard",
  "Carnival 4th gen predictive EPS taper",
  "Carnival 4th gen EPS torque guard",
  "Carnival 4th gen MDPS temporary steering fault",
)


def segment_number(path: Path) -> int:
  try:
    return int(path.parent.name.rsplit("--", 1)[1])
  except Exception:
    return -1


def main() -> None:
  parser = argparse.ArgumentParser(description="Extract Carnival lateral guard and MDPS fault transitions from route logs.")
  parser.add_argument("logs", nargs="+", type=Path)
  args = parser.parse_args()

  paths: list[Path] = []
  for path in args.logs:
    if path.is_dir():
      paths.extend(path.glob("rlog.zst"))
      paths.extend(path.glob("qlog.zst"))
    else:
      paths.extend(path.parent.glob(path.name))

  found = 0
  for path in sorted((p for p in paths if p.exists()), key=lambda p: (str(p.parent.parent), segment_number(p), p.name)):
    first_ns: int | None = None
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
      if first_ns is None:
        first_ns = int(msg.logMonoTime)
      if msg.which() != "logMessage":
        continue
      raw = str(msg.logMessage)
      if not any(pattern in raw for pattern in PATTERNS):
        continue
      try:
        parsed = json.loads(raw)
        text = str(parsed.get("msg", parsed.get("message", raw)))
      except Exception:
        text = raw
      found += 1
      elapsed = (int(msg.logMonoTime) - first_ns) / 1e9
      print(f"segment={segment_number(path)} t={elapsed:.3f} {text}")

  print(f"EVENTS_FOUND={found}")


if __name__ == "__main__":
  main()
