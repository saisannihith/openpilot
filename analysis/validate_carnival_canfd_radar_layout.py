#!/usr/bin/env python3
"""Validate the proposed two-slot 0x180-0x184 Carnival radar layout offline."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode


@dataclass(frozen=True)
class RadarObject:
  route: str
  t: float
  addr: int
  slot: int
  track_id: int
  state: int
  state_alt: int
  distance: float
  lateral: float
  velocity: float
  lateral_velocity: float
  acceleration: float
  heartbeat: int


def extract(raw: int, start: int, size: int, signed: bool = False) -> int:
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


def decode(route: str, t: float, addr: int, dat: bytes) -> list[RadarObject]:
  raw = int.from_bytes(dat, "little", signed=False)
  objects = []
  for slot, offset in ((1, 0), (2, 128)):
    objects.append(RadarObject(
      route=route,
      t=t,
      addr=addr,
      slot=slot,
      track_id=extract(raw, offset + 42, 8),
      state=extract(raw, offset + 55, 4),
      state_alt=extract(raw, offset + 51, 4),
      distance=extract(raw, offset + 64, 13) * 0.05,
      lateral=extract(raw, offset + 78, 11, True) * 0.05,
      velocity=extract(raw, offset + 91, 11, True) * 0.05 + 2.4,
      lateral_velocity=extract(raw, offset + 104, 9, True) * 0.05 + 0.6,
      acceleration=extract(raw, offset + 115, 9, True) * 0.1,
      heartbeat=extract(raw, offset + 124, 4),
    ))
  return objects


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def iter_logs(root: Path) -> list[Path]:
  by_segment: dict[Path, list[Path]] = defaultdict(list)
  for path in root.rglob("*"):
    if path.name in ("rlog", "rlog.zst", "rlog.bz2", "qlog", "qlog.zst", "qlog.bz2"):
      by_segment[path.parent].append(path)
  logs = []
  for files in by_segment.values():
    rlogs = [path for path in files if path.name.startswith("rlog")]
    logs.extend(rlogs if rlogs else files)
  return sorted(logs)


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
  return ordered[index]


def summary(values: list[float]) -> dict[str, Any]:
  return {
    "samples": len(values),
    "mae": None if not values else round(mean(values), 4),
    "p50": None if not values else round(percentile(values, 50) or 0.0, 4),
    "p95": None if not values else round(percentile(values, 95) or 0.0, 4),
    "p99": None if not values else round(percentile(values, 99) or 0.0, 4),
  }


def slope(points: list[tuple[float, float]]) -> float | None:
  if len(points) < 8 or points[-1][0] - points[0][0] < 0.45:
    return None
  x_bar = mean(x for x, _ in points)
  y_bar = mean(y for _, y in points)
  denom = sum((x - x_bar) ** 2 for x, _ in points)
  if denom <= 1e-9:
    return None
  value = sum((x - x_bar) * (y - y_bar) for x, y in points) / denom
  return value if math.isfinite(value) else None


def continuous(previous: RadarObject, current: RadarObject) -> bool:
  dt = current.t - previous.t
  return (
    previous.route == current.route and
    previous.track_id == current.track_id and
    0.005 <= dt <= 0.12 and
    abs(current.distance - previous.distance) <= max(0.5, 60.0 * dt) and
    abs(current.lateral - previous.lateral) <= max(0.5, 20.0 * dt) and
    abs(current.velocity - previous.velocity) <= 8.0
  )


def valid_object(obj: RadarObject) -> bool:
  return (obj.track_id != 0 and 0.5 <= obj.distance <= 220.0 and
          abs(obj.lateral) <= 50.0 and abs(obj.velocity) <= 60.0)


def local_slopes(objects: list[RadarObject], index: int, half_window: float = 0.45) -> tuple[float | None, float | None, float | None]:
  center = objects[index]
  lo = index
  while lo > 0 and center.t - objects[lo - 1].t <= half_window and continuous(objects[lo - 1], objects[lo]):
    lo -= 1
  hi = index
  while hi + 1 < len(objects) and objects[hi + 1].t - center.t <= half_window and continuous(objects[hi], objects[hi + 1]):
    hi += 1
  window = [
    obj for obj in objects[lo:hi + 1]
    if 0.5 <= obj.distance <= 220.0
  ]
  return (
    slope([(obj.t, obj.distance) for obj in window]),
    slope([(obj.t, obj.lateral) for obj in window]),
    slope([(obj.t, obj.velocity) for obj in window]),
  )


def analyze_channel(objects: list[RadarObject]) -> dict[str, Any]:
  objects.sort(key=lambda obj: (obj.route, obj.t))
  valid = [obj for obj in objects if valid_object(obj)]
  velocity_errors = []
  lateral_velocity_errors = []
  acceleration_errors = []
  state_velocity_errors: dict[str, list[float]] = defaultdict(list)
  evaluated = 0
  stride = max(1, len(objects) // 30000)
  for index in range(0, len(objects), stride):
    obj = objects[index]
    if not valid_object(obj):
      continue
    d_dot, y_dot, v_dot = local_slopes(objects, index)
    if d_dot is not None and abs(d_dot) <= 70.0:
      error = abs(obj.velocity - d_dot)
      velocity_errors.append(error)
      state_velocity_errors[f"{obj.state}/{obj.state_alt}"].append(error)
      evaluated += 1
    if y_dot is not None and abs(y_dot) <= 25.0:
      lateral_velocity_errors.append(abs(obj.lateral_velocity - y_dot))
    if v_dot is not None and abs(v_dot) <= 20.0:
      acceleration_errors.append(abs(obj.acceleration - v_dot))

  velocity_p95 = percentile(velocity_errors, 95) or 999.0
  return {
    "frames": len(objects),
    "validFrames": len(valid),
    "validCoverage": round(len(valid) / max(len(objects), 1), 4),
    "trackIds": len({obj.track_id for obj in valid}),
    "states": dict(Counter(f"{obj.state}/{obj.state_alt}" for obj in valid).most_common()),
    "distanceRange": None if not valid else [round(min(obj.distance for obj in valid), 2), round(max(obj.distance for obj in valid), 2)],
    "velocityRange": None if not valid else [round(min(obj.velocity for obj in valid), 2), round(max(obj.velocity for obj in valid), 2)],
    "velocityVsDistanceDerivative": summary(velocity_errors),
    "lateralVelocityVsLateralDerivative": summary(lateral_velocity_errors),
    "accelerationVsVelocityDerivative": summary(acceleration_errors),
    "velocityByState": {state: summary(errors) for state, errors in sorted(state_velocity_errors.items())},
    "velocityReady": evaluated >= 200 and velocity_p95 < 1.0,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--stock-scc-report", type=Path,
                      help="Independent stock-SCC velocity proof produced by reverse_engineer_carnival_radar_velocity.py")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  grouped: dict[str, list[Path]] = defaultdict(list)
  for path in iter_logs(args.log_root):
    grouped[route_name(path)].append(path)

  route_results = {}
  combined: dict[tuple[int, int], list[RadarObject]] = defaultdict(list)
  for route, paths in sorted(grouped.items()):
    channels: dict[tuple[int, int], list[RadarObject]] = defaultdict(list)
    message_counts = Counter()
    for path in paths:
      for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
        if msg.which() != "can":
          continue
        t = float(msg.logMonoTime) / 1e9
        for can in msg.can:
          dat = bytes(can.dat)
          addr = int(can.address)
          if int(can.src) != 1 or not (0x180 <= addr <= 0x184) or len(dat) != 32:
            continue
          message_counts[addr] += 1
          for obj in decode(route, t, addr, dat):
            channels[(addr, obj.slot)].append(obj)
            combined[(addr, obj.slot)].append(obj)
    route_results[route] = {
      "files": len(paths),
      "messageCounts": {f"0x{addr:x}": count for addr, count in sorted(message_counts.items())},
      "channels": {f"0x{addr:x}.{slot}": analyze_channel(objects)
                   for (addr, slot), objects in sorted(channels.items())},
    }
    print(f"{route}: {dict(message_counts)}", flush=True)

  combined_results = {f"0x{addr:x}.{slot}": analyze_channel(objects)
                      for (addr, slot), objects in sorted(combined.items())}
  derivative_ready_channels = [name for name, result in combined_results.items() if result["velocityReady"]]
  stock_scc_report = {}
  if args.stock_scc_report:
    stock_scc_report = json.loads(args.stock_scc_report.read_text())
  stock_scc_velocity_ready = bool(stock_scc_report.get("decoderReady"))
  complete_object_bank = len(combined_results) == 10 and all(result["validFrames"] >= 100 for result in combined_results.values())
  control_fields_ready = stock_scc_velocity_ready and complete_object_bank
  report = {
    "status": "pass" if control_fields_ready else "fail",
    "controlFieldsReady": control_fields_ready,
    "completeTenObjectBank": complete_object_bank,
    "stockSccVelocityReady": stock_scc_velocity_ready,
    "layout": {
      "messages": "bus 1, 0x180-0x184, 32 bytes, two 128-bit object slots",
      "trackId": "u8@42",
      "distance": "u13@64 * 0.05 m",
      "lateral": "s11@78 * 0.05 m",
      "relativeVelocity": "s11@91 * 0.05 + 2.4 m/s",
      "rollingCounter": "u4@124; zero is valid",
      "lateralVelocityCandidate": "s9@104 * 0.05 + 0.6 m/s; research only",
      "relativeAccelerationCandidate": "s9@115 * 0.1 m/s^2; research only",
    },
    "minimumRadarPointFields": ["trackId", "dRel", "yRel", "vRel", "measured"],
    "optionalFieldsPublishedAsNaN": ["aRel", "yvRel"],
    "derivativeResearchGate": "at least 200 stable samples and derivative p95 below 1.0 per channel",
    "derivativeReadyChannels": derivative_ready_channels,
    "stockSccEvidence": {
      "path": None if args.stock_scc_report is None else str(args.stock_scc_report),
      "readinessConclusion": stock_scc_report.get("readinessConclusion"),
      "comparisons": stock_scc_report.get("stockSccComparison", {}),
    },
    "combinedChannels": combined_results,
    "routes": route_results,
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  return 0 if control_fields_ready else 1


if __name__ == "__main__":
  raise SystemExit(main())
