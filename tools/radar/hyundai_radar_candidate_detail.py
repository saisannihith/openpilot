#!/usr/bin/env python3
"""
Detail verifier for promising Hyundai/Kia radar candidate banks.

Decodes an MRR30-like two-slot object layout from an arbitrary CAN bank and
compares the selected object to the logged lead. This is offline/probe-only.
"""

from __future__ import annotations

import argparse
import bisect
import math
import os
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

try:
  os.register_at_fork
except AttributeError:
  os.register_at_fork = lambda *args, **kwargs: None  # type: ignore[attr-defined]

from openpilot.tools.lib.logreader import LogReader


@dataclass(frozen=True)
class RefLead:
  t: float
  d_rel: float
  y_rel: float
  v_rel: float
  v_ego: float


@dataclass(frozen=True)
class Obj:
  addr: int
  slot: int
  state: int
  d_rel: float
  y_rel: float
  v_rel: float
  raw_state_alt: int


def iter_log_files(root: Path) -> list[Path]:
  files_by_segment = defaultdict(list)
  for p in root.rglob("*"):
    if p.name in ("rlog", "rlog.bz2", "rlog.zst", "qlog", "qlog.bz2", "qlog.zst"):
      files_by_segment[p.parent].append(p)

  out = []
  for files in files_by_segment.values():
    rlogs = [p for p in files if p.name.startswith("rlog")]
    out.extend(rlogs if rlogs else files)
  return sorted(out)


def route_key(path: Path, root: Path) -> str:
  rel = path.relative_to(root)
  segment = rel.parts[0] if len(rel.parts) > 1 else path.parent.name
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", segment)
  return match.group(1) if match else segment


def event_time_sec(event) -> float:
  return float(event.logMonoTime) / 1e9


def extract_little(dat: bytes, start: int, size: int, signed: bool = False) -> int:
  raw = int.from_bytes(dat, "little", signed=False)
  val = (raw >> start) & ((1 << size) - 1)
  if signed and val & (1 << (size - 1)):
    val -= 1 << size
  return val


def decode_mrr30_like(addr: int, dat: bytes) -> list[Obj]:
  return [
    Obj(addr, 1,
        extract_little(dat, 55, 4),
        extract_little(dat, 64, 12) * 0.05,
        extract_little(dat, 76, 12, signed=True) * 0.05,
        extract_little(dat, 88, 14, signed=True) * 0.01,
        extract_little(dat, 51, 4)),
    Obj(addr, 2,
        extract_little(dat, 183, 4),
        extract_little(dat, 192, 12) * 0.05,
        extract_little(dat, 204, 12, signed=True) * 0.05,
        extract_little(dat, 216, 14, signed=True) * 0.01,
        extract_little(dat, 179, 4)),
  ]


def downsample_refs(refs: list[RefLead], min_dt: float = 0.20) -> list[RefLead]:
  out = []
  last_t = -1e9
  for ref in refs:
    if ref.t - last_t >= min_dt:
      out.append(ref)
      last_t = ref.t
  return out


def collect_route(files: list[Path], bus: int, start_addr: int, end_addr: int, msg_len: int):
  can_by_addr = defaultdict(list)
  refs = []
  car_state = None

  for file in files:
    for event in LogReader(str(file)):
      which = event.which()
      t = event_time_sec(event)
      if which == "carState":
        car_state = event.carState
      elif which == "radarState":
        lead = event.radarState.leadOne
        if lead.status and car_state is not None and 1.0 < lead.dRel < 180.0:
          refs.append(RefLead(t, float(lead.dRel), float(lead.yRel), float(lead.vRel), float(car_state.vEgo)))
      elif which == "can":
        for msg in event.can:
          dat = bytes(msg.dat)
          if int(msg.src) == bus and start_addr <= int(msg.address) <= end_addr and len(dat) == msg_len:
            can_by_addr[int(msg.address)].append((t, dat))

  return {addr: seq for addr, seq in can_by_addr.items()}, downsample_refs(refs)


def nearest_messages(can_by_addr, ts_by_addr, ref: RefLead, window: float):
  out = []
  for addr, seq in can_by_addr.items():
    ts = ts_by_addr[addr]
    idx = bisect.bisect_left(ts, ref.t)
    for pos in (idx - 1, idx):
      if 0 <= pos < len(seq):
        t, dat = seq[pos]
        if abs(t - ref.t) <= window:
          out.extend(decode_mrr30_like(addr, dat))
  return out


def percentile(values: list[float], pct: float) -> float:
  if not values:
    return float("nan")
  values = sorted(values)
  idx = min(len(values) - 1, max(0, round((pct / 100.0) * (len(values) - 1))))
  return values[idx]


def summarize(route: str, can_by_addr, refs: list[RefLead], window: float):
  selected = []
  no_msgs = 0
  no_match = 0
  ts_by_addr = {addr: [t for t, _dat in seq] for addr, seq in can_by_addr.items()}

  for ref in refs:
    objs = nearest_messages(can_by_addr, ts_by_addr, ref, window)
    if not objs:
      no_msgs += 1
      continue
    objs = [obj for obj in objs if 0.5 <= obj.d_rel <= 220.0]
    if not objs:
      no_match += 1
      continue
    best = min(objs, key=lambda obj: abs(obj.d_rel - ref.d_rel))
    gate = max(3.0, min(12.0, ref.d_rel * 0.18))
    if abs(best.d_rel - ref.d_rel) <= gate:
      selected.append((ref, best))
    else:
      no_match += 1

  d_err = [abs(obj.d_rel - ref.d_rel) for ref, obj in selected]
  y_err = [abs(obj.y_rel - ref.y_rel) for ref, obj in selected]
  y_err_flipped = [abs(-obj.y_rel - ref.y_rel) for ref, obj in selected]
  v_err = [abs(obj.v_rel - ref.v_rel) for ref, obj in selected]
  deriv_err = []
  last_by_key = {}
  for ref, obj in selected:
    key = (obj.addr, obj.slot)
    prev = last_by_key.get(key)
    if prev is not None:
      prev_ref, prev_obj = prev
      dt = ref.t - prev_ref.t
      if 0.05 <= dt <= 1.0:
        d_dot = (obj.d_rel - prev_obj.d_rel) / dt
        if math.isfinite(d_dot) and abs(d_dot) < 80.0:
          deriv_err.append(abs(d_dot - ref.v_rel))
    last_by_key[key] = (ref, obj)
  states = Counter((obj.slot, obj.state, obj.raw_state_alt) for _ref, obj in selected)
  addrs = Counter(obj.addr for _ref, obj in selected)
  slots = Counter(obj.slot for _ref, obj in selected)

  print(f"\nROUTE {route}", flush=True)
  print(f"refs={len(refs)} selected={len(selected)} coverage={len(selected) / max(len(refs), 1):.3f} no_msgs={no_msgs} no_match={no_match}", flush=True)
  if not selected:
    return
  print(f"d_err mae={mean(d_err):.2f} p50={percentile(d_err, 50):.2f} p90={percentile(d_err, 90):.2f}")
  print(f"y_err mae={mean(y_err):.2f} flipped_mae={mean(y_err_flipped):.2f}")
  print(f"v_err mae={mean(v_err):.2f}")
  if deriv_err:
    print(f"d_dot_vs_vrel mae={mean(deriv_err):.2f} p50={percentile(deriv_err, 50):.2f} p90={percentile(deriv_err, 90):.2f}")
  print(f"address_counts={dict(addrs)} slot_counts={dict(slots)}")
  print(f"state_counts_top={states.most_common(10)}")
  print("samples")
  for ref, obj in selected[::max(1, len(selected) // 12)][:12]:
    print(
      f" t={ref.t:.2f} lead(d/y/v)=({ref.d_rel:.1f},{ref.y_rel:.2f},{ref.v_rel:.1f}) "
      f"obj=0x{obj.addr:x}.{obj.slot} state={obj.state}/{obj.raw_state_alt} "
      f"(d/y/v)=({obj.d_rel:.1f},{obj.y_rel:.2f},{obj.v_rel:.1f})"
    )


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--bus", type=int, default=1)
  parser.add_argument("--start", type=lambda v: int(v, 0), default=0x180)
  parser.add_argument("--end", type=lambda v: int(v, 0), default=0x184)
  parser.add_argument("--len", type=int, default=32)
  parser.add_argument("--window", type=float, default=0.08)
  args = parser.parse_args()

  files = iter_log_files(args.log_root)
  if not files:
    raise SystemExit(f"No rlog/qlog files found under {args.log_root}")

  by_route = defaultdict(list)
  for file in files:
    by_route[route_key(file, args.log_root)].append(file)

  print(f"LOG_ROOT {args.log_root}")
  for route, route_files in sorted(by_route.items()):
    can_by_addr, refs = collect_route(route_files, args.bus, args.start, args.end, args.len)
    summarize(route, can_by_addr, refs, args.window)


if __name__ == "__main__":
  main()
