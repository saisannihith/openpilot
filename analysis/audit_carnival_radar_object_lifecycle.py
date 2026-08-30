#!/usr/bin/env python3
"""Audit Carnival R0100 object identity and validity fields from raw logs."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader, ReadMode


@dataclass(frozen=True)
class RadarObject:
  raw_id: int
  state_alt: int
  state: int
  d_rel: float
  y_rel: float
  v_rel: float
  heartbeat: int


def extract(raw: int, start: int, size: int, signed: bool = False) -> int:
  value = (raw >> start) & ((1 << size) - 1)
  return value - (1 << size) if signed and value & (1 << (size - 1)) else value


def decode(dat: bytes, offset: int) -> RadarObject:
  raw = int.from_bytes(dat, "little")
  return RadarObject(
    extract(raw, offset + 42, 8), extract(raw, offset + 51, 4), extract(raw, offset + 55, 3),
    extract(raw, offset + 64, 13) * 0.05, extract(raw, offset + 78, 11, True) * 0.05,
    extract(raw, offset + 91, 11, True) * 0.05 + 2.4, extract(raw, offset + 124, 4),
  )


def candidate(obj: RadarObject) -> bool:
  return obj.raw_id != 0 and 0.5 <= obj.d_rel <= 220.0 and abs(obj.y_rel) <= 50.0 and abs(obj.v_rel) <= 60.0


def continuous(previous: tuple[float, RadarObject] | None, now: float, obj: RadarObject) -> bool:
  if previous is None:
    return False
  previous_time, previous_obj = previous
  dt = now - previous_time
  return (0.0 <= dt <= 0.15 and abs(obj.d_rel - previous_obj.d_rel) <= max(1.5, 60.0 * dt) and
          abs(obj.y_rel - previous_obj.y_rel) <= max(1.0, 20.0 * dt) and abs(obj.v_rel - previous_obj.v_rel) <= 8.0)


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def iter_logs(root: Path) -> list[Path]:
  segments: dict[Path, list[Path]] = defaultdict(list)
  for path in root.rglob("*"):
    if path.name in ("rlog", "rlog.zst", "rlog.bz2", "qlog", "qlog.zst", "qlog.bz2"):
      segments[path.parent].append(path)
  logs = []
  for files in segments.values():
    rlogs = [path for path in files if path.name.startswith("rlog")]
    logs.extend(rlogs or files)
  return sorted(logs)


def analyze(root: Path) -> dict:
  paths = iter_logs(root)
  counts: Counter[str] = Counter()
  routes: Counter[str] = Counter()
  state: Counter[int] = Counter()
  state_alt: Counter[int] = Counter()
  heartbeat: Counter[int] = Counter()
  state_heartbeat: Counter[str] = Counter()
  previous: dict[tuple[str, int], tuple[float, RadarObject]] = {}

  for file_index, path in enumerate(paths, start=1):
    route = route_name(path)
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      if msg.which() != "can":
        continue
      now = float(msg.logMonoTime) / 1e9
      batch: dict[int, RadarObject] = {}
      conflicts: set[int] = set()
      for can in msg.can:
        dat = bytes(can.dat)
        if int(can.src) != 1 or not (0x180 <= int(can.address) <= 0x184) or len(dat) != 32:
          continue
        counts["messages"] += 1
        for offset in (0, 128):
          counts["slots"] += 1
          obj = decode(dat, offset)
          if not candidate(obj):
            continue
          counts["candidates"] += 1
          routes[route] += 1
          state[obj.state] += 1
          state_alt[obj.state_alt] += 1
          heartbeat[obj.heartbeat] += 1
          state_heartbeat[f"{obj.state}/{obj.heartbeat}"] += 1
          if obj.raw_id in batch and batch[obj.raw_id] != obj:
            conflicts.add(obj.raw_id)
          else:
            batch[obj.raw_id] = obj
      counts["conflictingIds"] += len(conflicts)
      for raw_id in conflicts:
        batch.pop(raw_id, None)
      for raw_id, obj in batch.items():
        key = (route, raw_id)
        if continuous(previous.get(key), now, obj):
          counts["continuous"] += 1
          old = previous[key][1]
          counts["continuousHeartbeatZero"] += obj.heartbeat == 0
          counts["continuousStateOutside345"] += obj.state not in (3, 4, 5)
          counts["heartbeatTransitions"] += obj.heartbeat != old.heartbeat
          counts["stateTransitions"] += obj.state != old.state
          counts["stateAltTransitions"] += obj.state_alt != old.state_alt
        previous[key] = (now, obj)
    print(f"[{file_index}/{len(paths)}] {path.parent.name}", flush=True)

  total = max(counts["candidates"], 1)
  stable = max(counts["continuous"], 1)
  return {
    "status": "pass",
    "files": len(paths),
    "routes": dict(sorted(routes.items())),
    "counts": dict(counts),
    "distributions": {
      "state": {str(k): v for k, v in sorted(state.items())},
      "stateAlt": {str(k): v for k, v in sorted(state_alt.items())},
      "heartbeat": {str(k): v for k, v in sorted(heartbeat.items())},
      "stateHeartbeat": dict(sorted(state_heartbeat.items())),
    },
    "rates": {
      "heartbeatZeroAll": round(heartbeat[0] / total, 6),
      "heartbeatZeroContinuous": round(counts["continuousHeartbeatZero"] / stable, 6),
      "stateOutside345Continuous": round(counts["continuousStateOutside345"] / stable, 6),
      "heartbeatTransitions": round(counts["heartbeatTransitions"] / stable, 6),
      "stateTransitions": round(counts["stateTransitions"] / stable, 6),
      "stateAltTransitions": round(counts["stateAltTransitions"] / stable, 6),
    },
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()
  output = json.dumps(analyze(args.log_root), indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.write_text(output + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
