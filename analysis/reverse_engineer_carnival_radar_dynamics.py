#!/usr/bin/env python3
"""Validate unresolved Carnival R0100 dynamic fields across routes.

Offline only: this script never changes radar publication or control.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from openpilot.tools.lib.logreader import LogReader, ReadMode


@dataclass(frozen=True)
class Point:
  t: float
  route: str
  segment: str
  track_id: int
  quality: int
  state: int
  raw: int
  d_rel: float
  y_rel: float
  v_rel: float


@dataclass(frozen=True)
class Sample:
  route: str
  raw: int
  d_rel: float
  y_rel: float
  v_rel: float
  d_dot: float
  y_dot: float
  v_dot: float


def extract(raw: int, start: int, size: int, signed: bool = False) -> int:
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)$", path.parent.name)
  return int(match.group(1)) if match else -1


def iter_logs(root: Path, filters: list[str]) -> dict[str, list[Path]]:
  segments: dict[Path, list[Path]] = defaultdict(list)
  for path in root.rglob("*log.zst"):
    if re.search(r"--\d+$", path.parent.name):
      segments[path.parent].append(path)
  routes: dict[str, list[Path]] = defaultdict(list)
  for files in segments.values():
    rlogs = [path for path in files if path.name == "rlog.zst"]
    selected = rlogs or [path for path in files if path.name == "qlog.zst"]
    for path in selected[:1]:
      route = route_name(path)
      if not filters or any(token in route for token in filters):
        routes[route].append(path)
  for paths in routes.values():
    paths.sort(key=segment_number)
  return dict(sorted(routes.items()))


def decode(t: float, route: str, segment: str, dat: bytes) -> list[Point]:
  message = int.from_bytes(dat, "little", signed=False)
  points = []
  for offset in (0, 128):
    raw = (message >> offset) & ((1 << 128) - 1)
    point = Point(
      t=t, route=route, segment=segment,
      track_id=extract(raw, 42, 8), quality=extract(raw, 32, 8), state=extract(raw, 55, 3),
      raw=raw,
      d_rel=extract(raw, 64, 13) * 0.05,
      y_rel=extract(raw, 78, 11, True) * 0.05,
      v_rel=extract(raw, 91, 11, True) * 0.05 + 2.4,
    )
    if (point.track_id and point.quality and point.state and 0.5 <= point.d_rel <= 220.0 and
        abs(point.y_rel) <= 50.0 and abs(point.v_rel) <= 60.0):
      points.append(point)
  return points


def collect(route: str, paths: list[Path]) -> list[Point]:
  points = []
  for index, path in enumerate(paths, start=1):
    print(f"  [{index}/{len(paths)}] {path.parent.name}", flush=True)
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      if msg.which() != "can":
        continue
      batch: dict[int, Point] = {}
      conflicts = set()
      for can in msg.can:
        if int(can.src) != 1 or not 0x180 <= int(can.address) <= 0x184 or len(can.dat) != 32:
          continue
        for point in decode(float(msg.logMonoTime) / 1e9, route, path.parent.name, bytes(can.dat)):
          if point.track_id in batch and batch[point.track_id] != point:
            conflicts.add(point.track_id)
          else:
            batch[point.track_id] = point
      points.extend(point for key, point in batch.items() if key not in conflicts)
  return points


def continuous(previous: Point, point: Point) -> bool:
  dt = point.t - previous.t
  return (0.0 < dt <= 0.15 and point.segment == previous.segment and
          abs(point.d_rel - previous.d_rel) <= max(1.5, 60.0 * dt) and
          abs(point.y_rel - previous.y_rel) <= max(1.0, 20.0 * dt) and
          abs(point.v_rel - previous.v_rel) <= 8.0)


def sequences(points: list[Point]) -> list[list[Point]]:
  by_track: dict[int, list[Point]] = defaultdict(list)
  for point in points:
    by_track[point.track_id].append(point)
  result = []
  for track_points in by_track.values():
    current = []
    for point in sorted(track_points, key=lambda item: item.t):
      if current and not continuous(current[-1], point):
        if len(current) >= 8:
          result.append(current)
        current = []
      current.append(point)
    if len(current) >= 8:
      result.append(current)
  return result


def slope(times: np.ndarray, values: np.ndarray, index: int) -> float | None:
  selected = np.flatnonzero(np.abs(times - times[index]) <= 0.35)
  if len(selected) < 8 or times[selected[-1]] - times[selected[0]] < 0.30:
    return None
  x = times[selected] - float(np.mean(times[selected]))
  y = values[selected] - float(np.mean(values[selected]))
  denominator = float(np.dot(x, x))
  value = float(np.dot(x, y) / denominator) if denominator > 1e-9 else math.nan
  return value if math.isfinite(value) else None


def make_samples(route_sequences: list[list[Point]]) -> list[Sample]:
  result = []
  for sequence in route_sequences:
    times = np.asarray([point.t for point in sequence])
    fields = [np.asarray([getattr(point, name) for point in sequence]) for name in ("d_rel", "y_rel", "v_rel")]
    for index, point in enumerate(sequence):
      derivatives = [slope(times, field, index) for field in fields]
      if all(value is not None for value in derivatives):
        result.append(Sample(point.route, point.raw, point.d_rel, point.y_rel, point.v_rel, *derivatives))
  return result


def values(samples: list[Sample], start: int, size: int, signed: bool) -> np.ndarray:
  mask = (1 << size) - 1
  result = np.asarray([(sample.raw >> start) & mask for sample in samples], dtype=float)
  if signed:
    result[result >= (1 << (size - 1))] -= 1 << size
  return result


def fit(x: np.ndarray, y: np.ndarray) -> tuple[float, float] | None:
  if len(x) < 100 or float(np.var(x)) <= 1e-9:
    return None
  scale, offset = np.linalg.lstsq(np.column_stack((x, np.ones(len(x)))), y, rcond=None)[0]
  return float(scale), float(offset)


def metrics(prediction: np.ndarray, target: np.ndarray) -> dict[str, Any]:
  error = np.abs(prediction - target)
  return {
    "samples": len(error), "mae": round(float(np.mean(error)), 5),
    "p50": round(float(np.percentile(error, 50)), 5),
    "p95": round(float(np.percentile(error, 95)), 5),
    "p99": round(float(np.percentile(error, 99)), 5),
    "bias": round(float(np.mean(prediction - target)), 5),
    "correlation": round(float(np.corrcoef(prediction, target)[0, 1]), 6),
  }


def evaluate(samples: list[Sample], start: int, size: int, signed: bool, target_name: str) -> dict[str, Any] | None:
  x = values(samples, start, size, signed)
  y = np.asarray([getattr(sample, target_name) for sample in samples])
  route = np.asarray([sample.route for sample in samples])
  prediction = np.full(len(samples), np.nan)
  route_fits = {}
  for held_out in sorted(set(route)):
    test = route == held_out
    fitted = fit(x[~test], y[~test])
    if fitted is None:
      continue
    scale, offset = fitted
    prediction[test] = x[test] * scale + offset
    route_fits[held_out] = {"scale": round(scale, 8), "offset": round(offset, 6), "samples": int(np.sum(test))}
  valid = np.isfinite(prediction)
  if np.sum(valid) < 100:
    return None
  global_fit = fit(x, y)
  result = {
    "field": {"start": start, "size": size, "signed": signed},
    "target": target_name, "routeFits": route_fits,
    "crossRoute": metrics(prediction[valid], y[valid]),
  }
  if global_fit is not None:
    scale, offset = global_fit
    result["globalFit"] = {"scale": round(scale, 8), "offset": round(offset, 6)}
    result["global"] = metrics(x * scale + offset, y)
  return result


def search(samples: list[Sample], starts: range, sizes: range, target_name: str, limit: int,
           max_end: int = 124) -> list[dict[str, Any]]:
  search_samples = samples[::max(1, len(samples) // 100000)]
  candidates = []
  for start in starts:
    for size in sizes:
      if start + size > max_end:
        continue
      for signed in (False, True):
        result = evaluate(search_samples, start, size, signed, target_name)
        if result is None:
          continue
        summary = result["crossRoute"]
        result["rank"] = round(summary["p95"] + 2.0 * (1.0 - abs(summary["correlation"])), 6)
        candidates.append(result)
  return sorted(candidates, key=lambda item: item["rank"])[:limit]


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--route", action="append", default=[])
  parser.add_argument("--top", type=int, default=20)
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  all_samples = []
  route_summary = {}
  for route, paths in iter_logs(args.log_root, args.route).items():
    print(f"Analyzing {route} ({len(paths)} segments)", flush=True)
    route_points = collect(route, paths)
    route_sequences = sequences(route_points)
    route_samples = make_samples(route_sequences)
    all_samples.extend(route_samples)
    route_summary[route] = {
      "files": len(paths), "points": len(route_points),
      "sequences": len(route_sequences), "samples": len(route_samples),
    }

  report = {
    "status": "complete" if all_samples else "no_samples",
    "routes": route_summary, "samples": len(all_samples),
    "relativeAccelerationSearch": search(all_samples, range(110, 120), range(7, 13), "v_dot", args.top),
    "lateralVelocitySearch": search(all_samples, range(100, 110), range(7, 14), "y_dot", args.top),
    "prefixDistanceSearch": search(all_samples, range(0, 32), range(4, 17), "d_rel", args.top, max_end=32),
    "prefixLateralSearch": search(all_samples, range(0, 32), range(4, 17), "y_rel", args.top, max_end=32),
    "prefixVelocitySearch": search(all_samples, range(0, 32), range(4, 17), "v_rel", args.top, max_end=32),
    "prefixDistanceRateSearch": search(all_samples, range(0, 32), range(4, 17), "d_dot", args.top, max_end=32),
    "exactAccelerationCandidate": evaluate(all_samples, 115, 9, True, "v_dot"),
    "exactLateralCandidate": evaluate(all_samples, 104, 9, True, "y_dot"),
    "distanceVelocityConsistency": metrics(
      np.asarray([sample.v_rel for sample in all_samples]),
      np.asarray([sample.d_dot for sample in all_samples]),
    ) if all_samples else None,
    "notes": [
      "All candidate fits are leave-one-route-out.",
      "Object histories split on gaps, segment changes, and physical discontinuities.",
      "Derivatives use centered local regression rather than adjacent-frame differencing.",
      "Prefix searches are bounded to bits 0..31; slot 1 contains the frame CRC/counter while slot 2 remains object metadata.",
      "A correlated field is not control-ready until physical scale and harmful-event behavior are validated.",
    ],
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  return 0 if all_samples else 1


if __name__ == "__main__":
  raise SystemExit(main())
