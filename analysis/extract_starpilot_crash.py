#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader, ReadMode


def segment_number(path: Path) -> int:
  try:
    return int(path.parent.name.rsplit("--", 1)[1])
  except Exception:
    return -1


def main() -> None:
  parser = argparse.ArgumentParser(description="Extract StarPilot process tracebacks from route logs.")
  parser.add_argument("logs", nargs="+", type=Path)
  parser.add_argument("--limit", type=int, default=20, help="Maximum tracebacks to print. Use 0 for no limit.")
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
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
      if msg.which() != "logMessage":
        continue
      text = str(msg.logMessage)
      if "starpilot_process" not in text or "Traceback" not in text:
        continue
      found += 1
      if args.limit and found > args.limit:
        print(f"TRACEBACKS_FOUND_AT_LEAST={found}")
        return
      print(f"FILE {path}")
      try:
        data = json.loads(text)
        print(data.get("exc_info", text))
      except Exception:
        print(text)
      print()
  print(f"TRACEBACKS_FOUND={found}")


if __name__ == "__main__":
  main()
