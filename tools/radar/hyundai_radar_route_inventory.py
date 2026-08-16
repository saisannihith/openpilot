#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
from collections import Counter, defaultdict
from pathlib import Path

try:
  os.register_at_fork
except AttributeError:
  os.register_at_fork = lambda *args, **kwargs: None  # type: ignore[attr-defined]

from openpilot.tools.lib.logreader import LogReader


def iter_log_files(root: Path) -> list[Path]:
  return sorted(p for p in root.rglob("*") if p.name in ("rlog", "rlog.bz2", "rlog.zst", "qlog", "qlog.bz2", "qlog.zst"))


def route_key(path: Path, root: Path) -> str:
  rel = path.relative_to(root)
  segment = rel.parts[0] if len(rel.parts) > 1 else path.parent.name
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", segment)
  return match.group(1) if match else segment


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("log_root", type=Path)
  args = parser.parse_args()

  by_route = defaultdict(list)
  for file in iter_log_files(args.log_root):
    by_route[route_key(file, args.log_root)].append(file)

  for route, files in sorted(by_route.items()):
    cps = Counter()
    services = Counter()
    for file in files:
      for event in LogReader(str(file)):
        which = event.which()
        services[which] += 1
        if which == "carParams":
          cp = event.carParams
          cps[(cp.carFingerprint, int(cp.flags), bool(cp.radarUnavailable),
               bool(cp.openpilotLongitudinalControl), bool(cp.pcmCruise))] += 1
    print(f"{route} files={len(files)} carParams={cps.most_common(3)} services={dict((k, services[k]) for k in ('can', 'carParams', 'radarState', 'liveTracks', 'carState') if services[k])}")


if __name__ == "__main__":
  main()
