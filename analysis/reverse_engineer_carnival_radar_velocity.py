#!/usr/bin/env python3
"""Validate Carnival 0x180 velocity fields against route-separated raw rlogs.

This is an offline research tool. It never changes radar publishing or control.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode


CONFIRMATION_TRACK_MIN = 0xC4100
CONFIRMATION_TRACK_MAX = 0xC41FF


@dataclass(frozen=True)
class RawFrame:
  t: float
  raw: int
  d_rel: float
  track_id: int
  state: int
  state_alt: int


@dataclass(frozen=True)
class Reference:
  t: float
  d_rel: float
  v_rel: float


@dataclass(frozen=True)
class SccReference:
  t: float
  bus: int
  d_rel: float
  v_rel: float


@dataclass(frozen=True)
class Sample:
  route: str
  frame: RawFrame
  reference: Reference
  d_dot: float | None


def extract(raw: int, start: int, size: int, signed: bool = False) -> int:
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


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


def collect_route(paths: list[Path]) -> tuple[list[RawFrame], list[Reference], list[SccReference], dict[str, Any]]:
  frames = []
  refs = []
  scc_refs = []
  firmware = set()
  software = set()
  openpilot_long = set()

  for path in paths:
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      which = msg.which()
      t = float(msg.logMonoTime) / 1e9
      if which == "initData":
        software.add((str(msg.initData.gitCommit), str(msg.initData.gitBranch), bool(msg.initData.dirty)))
      elif which == "carParams":
        openpilot_long.add(bool(msg.carParams.openpilotLongitudinalControl))
        for fw in msg.carParams.carFw:
          if str(fw.ecu).endswith("fwdRadar"):
            firmware.add(bytes(fw.fwVersion).decode("latin1", errors="replace").strip("\x00"))
      elif which == "radarState":
        for lead in (msg.radarState.leadOne, msg.radarState.leadTwo):
          track_id = int(getattr(lead, "radarTrackId", -1))
          if lead.status and lead.radar and CONFIRMATION_TRACK_MIN <= track_id <= CONFIRMATION_TRACK_MAX:
            refs.append(Reference(t, float(lead.dRel), float(lead.vRel)))
      elif which == "can":
        for can in msg.can:
          dat = bytes(can.dat)
          if int(can.address) == 0x1A0 and len(dat) == 32:
            scc_raw = int.from_bytes(dat, "little", signed=False)
            scc_refs.append(SccReference(
              t=t,
              bus=int(can.src),
              d_rel=extract(scc_raw, 24, 11) * 0.1,
              v_rel=extract(scc_raw, 35, 9) * 0.1 - 16.4,
            ))
          if int(can.src) != 1 or int(can.address) != 0x180 or len(dat) != 32:
            continue
          raw = int.from_bytes(dat, "little", signed=False)
          frames.append(RawFrame(
            t=t,
            raw=raw,
            d_rel=extract(raw, 64, 13) * 0.05,
            track_id=extract(raw, 42, 8),
            state=extract(raw, 55, 4),
            state_alt=extract(raw, 51, 4),
          ))

  frames.sort(key=lambda frame: frame.t)
  refs.sort(key=lambda ref: ref.t)
  scc_refs.sort(key=lambda ref: ref.t)
  return frames, refs, scc_refs, {
    "firmware": sorted(firmware),
    "software": [list(item) for item in sorted(software)],
    "openpilotLongitudinalControl": sorted(openpilot_long),
  }


def local_distance_slope(frames: list[RawFrame], index: int, half_window: float = 0.45) -> float | None:
  center = frames[index]
  lo = index
  while lo > 0 and center.t - frames[lo - 1].t <= half_window:
    lo -= 1
  hi = index
  while hi + 1 < len(frames) and frames[hi + 1].t - center.t <= half_window:
    hi += 1

  points = [
    (frame.t - center.t, frame.d_rel) for frame in frames[lo:hi + 1]
    if (frame.track_id == center.track_id and frame.state == center.state and frame.state_alt == center.state_alt and
        0.5 <= frame.d_rel <= 220.0)
  ]
  if len(points) < 12 or points[-1][0] - points[0][0] < 0.55:
    return None

  x_bar = mean(point[0] for point in points)
  y_bar = mean(point[1] for point in points)
  denom = sum((x - x_bar) ** 2 for x, _ in points)
  if denom <= 1e-9:
    return None
  slope = sum((x - x_bar) * (y - y_bar) for x, y in points) / denom
  return slope if math.isfinite(slope) and abs(slope) <= 70.0 else None


def align_route(route: str, frames: list[RawFrame], refs: list[Reference]) -> list[Sample]:
  if not frames or not refs:
    return []
  times = [frame.t for frame in frames]
  samples = []
  last_frame_index = -1
  for ref in refs:
    idx = bisect.bisect_left(times, ref.t)
    candidates = [pos for pos in (idx - 1, idx) if 0 <= pos < len(frames)]
    if not candidates:
      continue
    best = min(candidates, key=lambda pos: abs(frames[pos].t - ref.t))
    frame = frames[best]
    if best == last_frame_index or abs(frame.t - ref.t) > 0.06:
      continue
    if frame.state not in (3, 4, 5) or not (0.5 <= frame.d_rel <= 220.0):
      continue
    if abs(frame.d_rel - ref.d_rel) > max(1.0, ref.d_rel * 0.03):
      continue
    samples.append(Sample(route, frame, ref, local_distance_slope(frames, best)))
    last_frame_index = best
  return samples


def compare_stock_scc(frames: list[RawFrame], refs: list[SccReference]) -> dict[str, Any]:
  rx_refs = [ref for ref in refs if ref.bus < 128]
  if not frames or not rx_refs:
    return {"references": len(refs), "receivedReferences": len(rx_refs), "aligned": 0,
            "buses": dict(Counter(ref.bus for ref in refs)), "velocityError": errors_summary([])}
  times = [frame.t for frame in frames]
  errors = []
  buses = Counter(ref.bus for ref in refs)
  aligned = 0
  for ref in rx_refs:
    if not (0.5 <= ref.d_rel <= 200.0):
      continue
    idx = bisect.bisect_left(times, ref.t)
    candidates = [pos for pos in (idx - 1, idx) if 0 <= pos < len(frames)]
    if not candidates:
      continue
    frame = min((frames[pos] for pos in candidates), key=lambda candidate: abs(candidate.t - ref.t))
    if abs(frame.t - ref.t) > 0.06 or abs(frame.d_rel - ref.d_rel) > max(1.0, ref.d_rel * 0.03):
      continue
    raw_velocity = extract(frame.raw, 91, 11, True) * 0.05 + 2.4
    errors.append(abs(raw_velocity - ref.v_rel))
    aligned += 1
  return {
    "references": len(refs),
    "receivedReferences": len(rx_refs),
    "aligned": aligned,
    "buses": {str(bus): count for bus, count in sorted(buses.items())},
    "velocityError": errors_summary(errors),
  }


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
  return ordered[index]


def errors_summary(values: list[float]) -> dict[str, Any]:
  return {
    "samples": len(values),
    "mae": None if not values else round(mean(values), 4),
    "p50": None if not values else round(percentile(values, 50) or 0.0, 4),
    "p95": None if not values else round(percentile(values, 95) or 0.0, 4),
    "p99": None if not values else round(percentile(values, 99) or 0.0, 4),
  }


def evaluate_fixed(samples: list[Sample], start: int, size: int, signed: bool,
                   scale: float, offset: float) -> dict[str, Any]:
  model_errors = []
  derivative_errors = []
  model_by_route: dict[str, list[float]] = defaultdict(list)
  derivative_by_route: dict[str, list[float]] = defaultdict(list)
  derivative_by_state: dict[str, list[float]] = defaultdict(list)
  values = []
  for sample in samples:
    value = extract(sample.frame.raw, start, size, signed) * scale + offset
    values.append(value)
    error = abs(value - sample.reference.v_rel)
    model_errors.append(error)
    model_by_route[sample.route].append(error)
    if sample.d_dot is not None:
      derivative_error = abs(value - sample.d_dot)
      derivative_errors.append(derivative_error)
      derivative_by_route[sample.route].append(derivative_error)
      derivative_by_state[f"{sample.frame.state}/{sample.frame.state_alt}"].append(derivative_error)
  return {
    "field": {"start": start, "size": size, "signed": signed, "scale": scale, "offset": offset},
    "valueRange": None if not values else [round(min(values), 3), round(max(values), 3)],
    "vsModelVelocity": errors_summary(model_errors),
    "vsSmoothedDistanceDerivative": errors_summary(derivative_errors),
    "modelRouteP95": {route: round(percentile(errors, 95) or 0.0, 4) for route, errors in sorted(model_by_route.items())},
    "derivativeRouteP95": {route: round(percentile(errors, 95) or 0.0, 4)
                           for route, errors in sorted(derivative_by_route.items())},
    "derivativeState": {state: errors_summary(errors) for state, errors in sorted(derivative_by_state.items())},
  }


def linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float] | None:
  if len(xs) < 100 or len(xs) != len(ys):
    return None
  x_bar = mean(xs)
  y_bar = mean(ys)
  denom = sum((x - x_bar) ** 2 for x in xs)
  if denom <= 1e-9:
    return None
  scale = sum((x - x_bar) * (y - y_bar) for x, y in zip(xs, ys, strict=True)) / denom
  return scale, y_bar - scale * x_bar


def leave_one_route_out(samples: list[Sample], start: int, size: int, signed: bool,
                        target: str) -> dict[str, Any] | None:
  routes = sorted({sample.route for sample in samples})
  all_model_errors = []
  all_derivative_errors = []
  fits = {}
  for route in routes:
    usable = [sample for sample in samples if target == "model" or sample.d_dot is not None]
    train = [sample for sample in usable if sample.route != route]
    test = [sample for sample in usable if sample.route == route]
    fit = linear_fit(
      [extract(sample.frame.raw, start, size, signed) for sample in train],
      [sample.reference.v_rel if target == "model" else float(sample.d_dot) for sample in train],
    )
    if fit is None or not test:
      continue
    scale, offset = fit
    fits[route] = {"scale": round(scale, 8), "offset": round(offset, 6), "samples": len(test)}
    for sample in test:
      value = extract(sample.frame.raw, start, size, signed) * scale + offset
      all_model_errors.append(abs(value - sample.reference.v_rel))
      if sample.d_dot is not None:
        all_derivative_errors.append(abs(value - sample.d_dot))
  if not fits:
    return None
  return {
    "field": {"start": start, "size": size, "signed": signed},
    "fitTarget": target,
    "fits": fits,
    "vsModelVelocity": errors_summary(all_model_errors),
    "vsSmoothedDistanceDerivative": errors_summary(all_derivative_errors),
  }


def candidate_search(samples: list[Sample], limit: int, target: str) -> list[dict[str, Any]]:
  stride = max(1, len(samples) // 20000)
  search_samples = samples[::stride]
  candidates = []
  for start in range(80, 121):
    for size in range(7, 17):
      if start + size > 128:
        continue
      for signed in (False, True):
        result = leave_one_route_out(search_samples, start, size, signed, target)
        if result is None:
          continue
        model_p95 = result["vsModelVelocity"]["p95"]
        derivative_p95 = result["vsSmoothedDistanceDerivative"]["p95"]
        if model_p95 is None:
          continue
        primary_p95 = derivative_p95 if target == "derivative" else model_p95
        secondary_p95 = model_p95 if target == "derivative" else derivative_p95
        result["rank"] = round(primary_p95 + 0.1 * (secondary_p95 if secondary_p95 is not None else 25.0), 4)
        candidates.append(result)
  candidates.sort(key=lambda item: item["rank"])
  return candidates[:limit]


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--top", type=int, default=20)
  parser.add_argument("--no-search", action="store_true")
  parser.add_argument("--route", action="append", default=[])
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  paths = iter_logs(args.log_root)
  grouped: dict[str, list[Path]] = defaultdict(list)
  for path in paths:
    grouped[route_name(path)].append(path)
  if args.route:
    grouped = defaultdict(list, {route: route_paths for route, route_paths in grouped.items()
                                 if any(token in route for token in args.route)})

  all_samples = []
  route_report = {}
  metadata = {}
  stock_scc = {}
  for route, route_paths in sorted(grouped.items()):
    frames, refs, scc_refs, meta = collect_route(route_paths)
    samples = align_route(route, frames, refs)
    all_samples.extend(samples)
    metadata[route] = meta
    stock_scc[route] = compare_stock_scc(frames, scc_refs)
    route_report[route] = {
      "files": len(route_paths),
      "rawFrames": len(frames),
      "confirmationReferences": len(refs),
      "alignedSamples": len(samples),
      "smoothedDerivativeSamples": sum(sample.d_dot is not None for sample in samples),
      "states": sorted({f"{sample.frame.state}/{sample.frame.state_alt}" for sample in samples}),
      "trackIds": sorted({sample.frame.track_id for sample in samples}),
    }
    print(f"{route}: frames={len(frames)} refs={len(refs)} aligned={len(samples)}", flush=True)

  exact_pr = evaluate_fixed(all_samples, 91, 11, True, 0.05, 2.4)
  current_truncated = evaluate_fixed(all_samples, 91, 8, True, 0.050066083726851514, 2.4059439014445294)
  candidates = [] if args.no_search else candidate_search(all_samples, args.top, "derivative")
  exact_derivative_fit = leave_one_route_out(all_samples, 91, 11, True, "derivative")
  exact_p95 = exact_pr["vsModelVelocity"]["p95"] or 999.0
  exact_derivative_p95 = exact_pr["vsSmoothedDistanceDerivative"]["p95"] or 999.0
  stock_scc_ready = any(
    comparison["aligned"] >= 200 and (comparison["velocityError"]["p95"] or 999.0) < 1.0
    for comparison in stock_scc.values()
  )
  control_promotion_ready = len(all_samples) >= 200 and exact_p95 < 1.0 and exact_derivative_p95 < 1.0
  report = {
    "status": "pass" if stock_scc_ready else "fail",
    "decoderReady": stock_scc_ready,
    "controlPromotionReady": control_promotion_ready,
    "readinessConclusion": (
      "raw_velocity_decoded_stock_scc_verified"
      if stock_scc_ready else
      "raw_velocity_candidate_unverified"
    ),
    "promotionGate": {
      "minimumSamples": 200,
      "maximumModelVelocityP95": 1.0,
      "maximumDistanceDerivativeP95": 1.0,
    },
    "routes": route_report,
    "metadata": metadata,
    "stockSccComparison": stock_scc,
    "alignedSamples": len(all_samples),
    "exactPublicPrField": exact_pr,
    "exactPublicPrRouteSeparatedDerivativeFit": exact_derivative_fit,
    "currentTruncatedState3Alt8Field": current_truncated,
    "topRouteSeparatedDerivativeCandidates": candidates,
    "notes": [
      "radarState confirmation velocity is model-led in SNITHPilot and is an independent reference for raw velocity decoding",
      "distance derivative uses a local regression over stable track/state identity instead of adjacent-frame differencing",
      "candidate fits are trained without the evaluated route and therefore cannot pass solely by memorizing one drive",
      "a passing field still requires live shadow validation before any control use",
    ],
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
