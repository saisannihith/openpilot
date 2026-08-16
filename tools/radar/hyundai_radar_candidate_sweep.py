#!/usr/bin/env python3
"""
Offline Hyundai/Kia radar candidate sweeper.

This tool replays rlogs/qlogs and tries known Hyundai radar layouts against
observed CAN address banks. It is intentionally read-only and does not affect
on-road behavior.
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
from opendbc.can.dbc import DBC as DBCReader
from opendbc.can.parser import get_raw_value


@dataclass(frozen=True)
class Layout:
  name: str
  msg_len: int
  slots_per_msg: int
  state_based: bool


@dataclass(frozen=True)
class Candidate:
  bus: int
  start_addr: int
  count: int
  msg_len: int
  avg_hz: float
  avg_count: float

  @property
  def end_addr(self) -> int:
    return self.start_addr + self.count - 1

  @property
  def label(self) -> str:
    return f"bus={self.bus} len={self.msg_len} 0x{self.start_addr:x}-0x{self.end_addr:x}"


@dataclass
class DecodedPoint:
  t: float
  addr: int
  slot: int
  d_rel: float
  y_rel: float
  v_rel: float
  valid: bool


LAYOUTS = (
  Layout("mando_0x500_8b", 8, 1, True),
  Layout("mrrevo14f_0x602_8b_two_slot", 8, 2, False),
  Layout("mrr30_0x210_32b_two_slot", 32, 2, True),
  Layout("mrr35_0x3a5_24b", 24, 1, True),
)

DBC_LAYOUTS = {
  "mando_0x500_8b": ("hyundai_kia_mando_front_radar_generated", 0x500),
  "mrrevo14f_0x602_8b_two_slot": ("hyundai_mrrevo14f_radar_generated", 0x602),
  "mrr30_0x210_32b_two_slot": ("hyundai_mrr30_radar_generated", 0x210),
  "mrr35_0x3a5_24b": ("hyundai_mrr35_radar_generated", 0x3A5),
}

KNOWN_BANKS = (
  (0, 0x3A5, 32, 24, "mrr35 confirmed CAN-FD radar"),
  (1, 0x3A5, 32, 24, "mrr35 camera-bus variant"),
  (0, 0x210, 16, 32, "mrr30 radar"),
  (1, 0x500, 32, 8, "mando radar"),
  (1, 0x602, 16, 8, "mrrevo14f radar"),
  (1, 0x180, 5, 32, "Sunny ALT_180_184 candidate"),
  (1, 0x1B6, 4, 32, "Sunny ALT_1B6_1B9 candidate"),
  (1, 0x2BB, 5, 32, "Sunny ALT_2BB_2BF candidate"),
  (1, 0x2A2, 3, 32, "observed 20Hz candidate"),
  (2, 0x1B5, 1, 32, "observed bus-2 20Hz candidate"),
  (0, 0x1BA, 1, 24, "observed 20Hz candidate"),
  (0, 0x1E5, 1, 16, "observed 20Hz candidate"),
  (0, 0x1F0, 1, 16, "observed 20Hz candidate"),
)


SIGNALS = {}


def load_layout_signals(layout_name: str):
  if layout_name not in SIGNALS:
    dbc_name, template_addr = DBC_LAYOUTS[layout_name]
    SIGNALS[layout_name] = DBCReader(dbc_name).addr_to_msg[template_addr].sigs
  return SIGNALS[layout_name]


def signal_value(layout_name: str, signal_name: str, dat: bytes) -> float:
  sig = load_layout_signals(layout_name)[signal_name]
  raw = get_raw_value(dat, sig)
  if sig.is_signed:
    raw -= ((raw >> (sig.size - 1)) & 1) * (1 << sig.size)
  return raw * sig.factor + sig.offset


def plausible(point: DecodedPoint) -> bool:
  return (
    point.valid
    and 0.5 <= point.d_rel <= 260.0
    and abs(point.y_rel) <= 25.0
    and -90.0 <= point.v_rel <= 90.0
    and all(math.isfinite(v) for v in (point.d_rel, point.y_rel, point.v_rel))
  )


def decode_mando(dat: bytes, t: float, addr: int) -> list[DecodedPoint]:
  layout_name = "mando_0x500_8b"
  state = signal_value(layout_name, "STATE", dat)
  azimuth = signal_value(layout_name, "AZIMUTH", dat)
  long_dist = signal_value(layout_name, "LONG_DIST", dat)
  rel_speed = signal_value(layout_name, "REL_SPEED", dat)
  valid = state in (3, 4)
  y_rel = -0.5 * math.sin(math.radians(azimuth)) * long_dist
  d_rel = math.cos(math.radians(azimuth)) * long_dist
  return [DecodedPoint(t, addr, 0, d_rel, y_rel, rel_speed, valid)]


def decode_mrrevo14f(dat: bytes, t: float, addr: int) -> list[DecodedPoint]:
  points = []
  layout_name = "mrrevo14f_0x602_8b_two_slot"
  for slot, prefix in enumerate(("1", "2")):
    d_rel = signal_value(layout_name, f"{prefix}_DISTANCE", dat)
    y_rel = signal_value(layout_name, f"{prefix}_LATERAL", dat)
    v_rel = signal_value(layout_name, f"{prefix}_SPEED", dat)
    points.append(DecodedPoint(t, addr, slot, d_rel, y_rel, v_rel, d_rel != 255.75))
  return points


def decode_mrr30(dat: bytes, t: float, addr: int) -> list[DecodedPoint]:
  points = []
  layout_name = "mrr30_0x210_32b_two_slot"
  for slot, prefix in enumerate(("1", "2")):
    state = signal_value(layout_name, f"{prefix}_STATE", dat)
    d_rel = signal_value(layout_name, f"{prefix}_LONG_DIST", dat)
    y_rel = signal_value(layout_name, f"{prefix}_LAT_DIST", dat)
    v_rel = signal_value(layout_name, f"{prefix}_REL_SPEED", dat)
    points.append(DecodedPoint(t, addr, slot, d_rel, y_rel, v_rel, state in (3, 4)))
  return points


def decode_mrr35(dat: bytes, t: float, addr: int) -> list[DecodedPoint]:
  layout_name = "mrr35_0x3a5_24b"
  state = signal_value(layout_name, "STATE", dat)
  d_rel = signal_value(layout_name, "LONG_DIST", dat)
  y_rel = signal_value(layout_name, "LAT_DIST", dat)
  v_rel = signal_value(layout_name, "REL_SPEED", dat)
  return [DecodedPoint(t, addr, 0, d_rel, y_rel, v_rel, state in (3, 4))]


DECODERS = {
  "mando_0x500_8b": decode_mando,
  "mrrevo14f_0x602_8b_two_slot": decode_mrrevo14f,
  "mrr30_0x210_32b_two_slot": decode_mrr30,
  "mrr35_0x3a5_24b": decode_mrr35,
}


def iter_log_files(root: Path) -> list[Path]:
  return sorted(p for p in root.rglob("*") if p.name in ("rlog", "rlog.bz2", "rlog.zst", "qlog", "qlog.bz2", "qlog.zst"))


def route_key(path: Path, root: Path) -> str:
  rel = path.relative_to(root)
  segment = rel.parts[0] if len(rel.parts) > 1 else path.parent.name
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", segment)
  return match.group(1) if match else segment


def event_time_sec(event) -> float:
  return float(event.logMonoTime) / 1e9


def scan_route(files: list[Path]):
  can_counts = Counter()
  can_samples = defaultdict(list)
  can_by_key = defaultdict(list)
  radar_leads = []
  live_track_points = 0
  live_track_msgs = 0
  car_params = Counter()

  for file in files:
    for event in LogReader(str(file)):
      which = event.which()
      t = event_time_sec(event)
      if which == "carParams":
        cp = event.carParams
        car_params[(cp.carFingerprint, int(cp.flags), bool(cp.radarUnavailable),
                    bool(cp.openpilotLongitudinalControl), bool(cp.pcmCruise))] += 1
      elif which == "radarState":
        lead = event.radarState.leadOne
        if lead.status:
          radar_leads.append((t, float(lead.dRel), float(lead.yRel), float(lead.vRel), bool(lead.radar)))
      elif which == "liveTracks":
        live_track_msgs += 1
        live_track_points += len(event.liveTracks.points)
      elif which == "can":
        for msg in event.can:
          dat = bytes(msg.dat)
          key = (int(msg.src), int(msg.address), len(dat))
          can_counts[key] += 1
          if len(can_samples[key]) < 3:
            can_samples[key].append(dat)
          can_by_key[key].append((t, dat))

  candidates = discover_candidates(can_counts, can_by_key)
  candidates.extend(forced_known_candidates(can_counts, can_by_key))
  candidates = unique_candidates(candidates)

  return car_params, live_track_msgs, live_track_points, radar_leads, can_by_key, candidates


def discover_candidates(can_counts, can_by_key) -> list[Candidate]:
  by_bus_len = defaultdict(list)
  for bus, addr, msg_len in can_counts:
    if can_counts[(bus, addr, msg_len)] >= 100:
      by_bus_len[(bus, msg_len)].append(addr)

  candidates = []
  for (bus, msg_len), addrs in by_bus_len.items():
    addrs = sorted(set(addrs))
    run = []
    for addr in addrs:
      if not run or addr == run[-1] + 1:
        run.append(addr)
      else:
        add_run_candidate(candidates, bus, msg_len, run, can_counts, can_by_key)
        run = [addr]
    add_run_candidate(candidates, bus, msg_len, run, can_counts, can_by_key)
  return candidates


def add_run_candidate(candidates, bus, msg_len, run, can_counts, can_by_key):
  if len(run) < 3:
    return
  counts = [can_counts[(bus, addr, msg_len)] for addr in run]
  first_ts = [can_by_key[(bus, addr, msg_len)][0][0] for addr in run if can_by_key[(bus, addr, msg_len)]]
  last_ts = [can_by_key[(bus, addr, msg_len)][-1][0] for addr in run if can_by_key[(bus, addr, msg_len)]]
  duration = max(last_ts) - min(first_ts) if first_ts and last_ts else 0.0
  avg_hz = mean(counts) / duration if duration > 0 else 0.0
  if avg_hz >= 1.0:
    candidates.append(Candidate(bus, run[0], len(run), msg_len, avg_hz, mean(counts)))


def forced_known_candidates(can_counts, can_by_key) -> list[Candidate]:
  candidates = []
  for bus, start, count, msg_len, _desc in KNOWN_BANKS:
    present = [addr for addr in range(start, start + count) if (bus, addr, msg_len) in can_counts]
    if not present:
      continue
    counts = [can_counts[(bus, addr, msg_len)] for addr in present]
    first_ts = [can_by_key[(bus, addr, msg_len)][0][0] for addr in present if can_by_key[(bus, addr, msg_len)]]
    last_ts = [can_by_key[(bus, addr, msg_len)][-1][0] for addr in present if can_by_key[(bus, addr, msg_len)]]
    duration = max(last_ts) - min(first_ts) if first_ts and last_ts else 0.0
    avg_hz = mean(counts) / duration if duration > 0 else 0.0
    candidates.append(Candidate(bus, start, count, msg_len, avg_hz, mean(counts)))
  return candidates


def unique_candidates(candidates: list[Candidate]) -> list[Candidate]:
  seen = set()
  out = []
  for candidate in candidates:
    key = (candidate.bus, candidate.start_addr, candidate.count, candidate.msg_len)
    if key not in seen:
      seen.add(key)
      out.append(candidate)
  return sorted(out, key=lambda c: (-c.avg_count, c.bus, c.start_addr, c.msg_len))


def score_candidate(candidate: Candidate, layout: Layout, can_by_key, radar_leads):
  if candidate.msg_len != layout.msg_len:
    return None

  decoder = DECODERS[layout.name]
  all_points = []
  for addr in range(candidate.start_addr, candidate.start_addr + candidate.count):
    for t, dat in can_by_key.get((candidate.bus, addr, candidate.msg_len), []):
      all_points.extend(decoder(dat, t, addr))

  if not all_points:
    return None

  plausible_points = [p for p in all_points if plausible(p)]
  near_lane = [p for p in plausible_points if abs(p.y_rel) <= 3.8]
  d_values = [p.d_rel for p in near_lane]
  lead_corr = correlate_to_lead(near_lane, radar_leads)

  return {
    "layout": layout.name,
    "decoded": len(all_points),
    "plausible": len(plausible_points),
    "near_lane": len(near_lane),
    "valid_ratio": len(plausible_points) / max(len(all_points), 1),
    "near_lane_ratio": len(near_lane) / max(len(all_points), 1),
    "d_min": min(d_values) if d_values else float("nan"),
    "d_med": percentile(d_values, 50.0) if d_values else float("nan"),
    "d_max": max(d_values) if d_values else float("nan"),
    "lead_mae": lead_corr[0],
    "lead_matches": lead_corr[1],
  }


def percentile(values: list[float], pct: float) -> float:
  if not values:
    return float("nan")
  values = sorted(values)
  idx = min(len(values) - 1, max(0, round((pct / 100.0) * (len(values) - 1))))
  return values[idx]


def correlate_to_lead(points: list[DecodedPoint], radar_leads):
  if not points or not radar_leads:
    return float("nan"), 0

  lead_ts = [t for t, *_ in radar_leads]
  errors = []
  points_by_bucket = defaultdict(list)
  for point in points:
    points_by_bucket[round(point.t, 1)].append(point)

  for t, lead_d, lead_y, _lead_v, _lead_radar in radar_leads[::10]:
    idx = bisect.bisect_left(lead_ts, t)
    if idx >= len(lead_ts):
      continue
    nearby = []
    for bucket in (round(t - 0.1, 1), round(t, 1), round(t + 0.1, 1)):
      nearby.extend(points_by_bucket.get(bucket, []))
    if not nearby:
      continue
    best = min(nearby, key=lambda p: abs(p.d_rel - lead_d) + abs(p.y_rel - lead_y) * 2.0)
    if abs(best.d_rel - lead_d) <= 20.0 and abs(best.y_rel - lead_y) <= 5.0:
      errors.append(abs(best.d_rel - lead_d))

  return (mean(errors), len(errors)) if errors else (float("nan"), 0)


def print_route_report(route: str, car_params, live_track_msgs, live_track_points, radar_leads, can_by_key, candidates, top_n: int):
  print(f"\nROUTE {route}")
  print("carParams", list(car_params.items())[:4])
  print(f"liveTracks msgs={live_track_msgs} points={live_track_points}")
  radar_lead_count = len(radar_leads)
  radar_true_count = sum(1 for *_rest, radar in radar_leads if radar)
  print(f"radarState lead.status={radar_lead_count} lead.radar={radar_true_count}")

  rows = []
  for candidate in candidates:
    for layout in LAYOUTS:
      score = score_candidate(candidate, layout, can_by_key, radar_leads)
      if score is not None and (score["plausible"] > 0 or candidate.start_addr in (0x180, 0x1B6, 0x2BB, 0x3A5)):
        rows.append((candidate, score))

  rows.sort(key=lambda item: (
    -(item[1]["lead_matches"] or 0),
    item[1]["lead_mae"] if math.isfinite(item[1]["lead_mae"]) else 1e9,
    -item[1]["near_lane_ratio"],
    -item[1]["valid_ratio"],
  ))

  print("\nCANDIDATE SWEEP")
  for candidate, score in rows[:top_n]:
    d_med = score["d_med"]
    d_range = "n/a" if not math.isfinite(d_med) else f"{score['d_min']:.1f}/{score['d_med']:.1f}/{score['d_max']:.1f}m"
    lead_mae = "n/a" if not math.isfinite(score["lead_mae"]) else f"{score['lead_mae']:.1f}m"
    print(
      f"{candidate.label:<30} {score['layout']:<24} "
      f"plausible={score['plausible']:>7}/{score['decoded']:<7} "
      f"near_lane={score['near_lane']:>7} d_min/med/max={d_range:<18} "
      f"lead_matches={score['lead_matches']:<5} lead_mae={lead_mae}"
    )


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path, help="Directory containing copied route logs")
  parser.add_argument("--top", type=int, default=40, help="Rows to print per route")
  args = parser.parse_args()

  files = iter_log_files(args.log_root)
  if not files:
    raise SystemExit(f"No rlog/qlog files found under {args.log_root}")

  by_route = defaultdict(list)
  for file in files:
    by_route[route_key(file, args.log_root)].append(file)

  print(f"LOG_ROOT {args.log_root}")
  print(f"LOG_FILES {len(files)}")
  for route, route_files in sorted(by_route.items()):
    result = scan_route(route_files)
    print_route_report(route, *result, top_n=args.top)


if __name__ == "__main__":
  main()
