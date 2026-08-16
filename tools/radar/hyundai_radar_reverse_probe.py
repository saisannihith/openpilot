#!/usr/bin/env python3
"""
Offline reverse-engineering probe for Hyundai/Kia lead/radar-like CAN banks.

The script brute-forces raw bitfields from selected CAN banks and ranks fields
that can explain the logged lead distance over time. It is a research tool only:
it never publishes tracks or changes control behavior.
"""

from __future__ import annotations

import argparse
import bisect
import math
import os
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

try:
  os.register_at_fork
except AttributeError:
  os.register_at_fork = lambda *args, **kwargs: None  # type: ignore[attr-defined]

from openpilot.tools.lib.logreader import LogReader


@dataclass(frozen=True)
class Bank:
  name: str
  bus: int
  start_addr: int
  end_addr: int
  msg_len: int


@dataclass(frozen=True)
class RefLead:
  t: float
  d_rel: float
  y_rel: float
  v_rel: float
  v_ego: float


@dataclass(frozen=True)
class FieldSpec:
  start: int
  size: int
  signed: bool
  scale: float
  offset: float

  @property
  def label(self) -> str:
    typ = "s" if self.signed else "u"
    return f"{typ}{self.size}@{self.start}*{self.scale:g}{self.offset:+g}"


@dataclass
class MatchScore:
  bank: Bank
  spec: FieldSpec
  refs: int
  matches: int
  mae: float
  p90: float
  continuity: float
  addr_count: int
  sample: list[tuple[float, float, int, int]]
  neg_coverage: float = 0.0
  neg_mae: float = float("nan")

  @property
  def coverage(self) -> float:
    return self.matches / max(self.refs, 1)

  @property
  def rank(self) -> tuple[float, float, float, float]:
    lift = self.coverage - self.neg_coverage
    return (lift, self.coverage, -self.mae, self.continuity)


BANKS = (
  Bank("sunny_alt_180_184", 1, 0x180, 0x184, 32),
  Bank("sunny_alt_1b6_1b9", 1, 0x1B6, 0x1B9, 32),
  Bank("sunny_alt_2bb_2bf", 1, 0x2BB, 0x2BF, 32),
  Bank("observed_2a2_2a4", 1, 0x2A2, 0x2A4, 32),
  Bank("observed_bus2_1b5", 2, 0x1B5, 0x1B5, 32),
  Bank("observed_bus0_1ba", 0, 0x1BA, 0x1BA, 24),
  Bank("observed_bus0_1e5", 0, 0x1E5, 0x1E5, 16),
  Bank("observed_bus0_1f0", 0, 0x1F0, 0x1F0, 16),
  Bank("partial_416_41c", 0, 0x416, 0x41C, 8),
  Bank("partial_410_414", 0, 0x410, 0x414, 8),
  Bank("partial_37f_384", 0, 0x37F, 0x384, 8),
  Bank("partial_3a5_3c4_len8", 0, 0x3A5, 0x3C4, 8),
)

DIST_SCALES = (0.01, 0.02, 0.025, 0.03, 0.05, 0.1, 0.125, 0.25)
FIELD_SIZES = (8, 9, 10, 11, 12, 13, 14, 16)


def iter_log_files(root: Path) -> list[Path]:
  return sorted(p for p in root.rglob("*") if p.name in ("rlog", "rlog.bz2", "rlog.zst", "qlog", "qlog.bz2", "qlog.zst"))


def route_key(path: Path, root: Path) -> str:
  rel = path.relative_to(root)
  segment = rel.parts[0] if len(rel.parts) > 1 else path.parent.name
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", segment)
  return match.group(1) if match else segment


def event_time_sec(event) -> float:
  return float(event.logMonoTime) / 1e9


def extract_little(dat: bytes, start: int, size: int, signed: bool) -> int:
  raw = int.from_bytes(dat, "little", signed=False)
  val = (raw >> start) & ((1 << size) - 1)
  if signed and val & (1 << (size - 1)):
    val -= 1 << size
  return val


def collect_route(files: list[Path], banks: tuple[Bank, ...]):
  wanted = {
    (bank.bus, addr, bank.msg_len)
    for bank in banks
    for addr in range(bank.start_addr, bank.end_addr + 1)
  }
  can_by_key = defaultdict(list)
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
          key = (int(msg.src), int(msg.address), len(dat))
          if key in wanted:
            can_by_key[key].append((t, dat))

  return can_by_key, downsample_refs(refs, min_dt=0.30)


def downsample_refs(refs: list[RefLead], min_dt: float = 0.20) -> list[RefLead]:
  out = []
  last_t = -1e9
  for ref in refs:
    if ref.t - last_t >= min_dt:
      out.append(ref)
      last_t = ref.t
  return out


def build_bank_ref_messages(can_by_key, bank: Bank, refs: list[RefLead], window: float = 0.08):
  seq_by_addr = {}
  ts_by_addr = {}
  for addr in range(bank.start_addr, bank.end_addr + 1):
    seq = can_by_key.get((bank.bus, addr, bank.msg_len), [])
    if seq:
      seq_by_addr[addr] = seq
      ts_by_addr[addr] = [t for t, _dat in seq]

  out = []
  for ref in refs:
    out.append((ref, nearby_messages(seq_by_addr, ts_by_addr, ref, window=window)))
  return out


def shifted_ref_messages(ref_messages):
  if not ref_messages:
    return []
  refs = [ref for ref, _messages in ref_messages]
  shift = max(1, len(refs) // 2)
  out = []
  for idx, (_ref, messages) in enumerate(ref_messages):
    shifted = refs[(idx + shift) % len(refs)]
    out.append((shifted, messages))
  return out


def nearby_messages(seq_by_addr, ts_by_addr, ref: RefLead, window: float = 0.08):
  out = []
  for addr, seq in seq_by_addr.items():
    ts = ts_by_addr[addr]
    idx = bisect.bisect_left(ts, ref.t)
    for pos in (idx - 1, idx):
      if 0 <= pos < len(seq):
        t, dat = seq[pos]
        if abs(t - ref.t) <= window:
          out.append((addr, pos, dat))
  return out


def field_specs(msg_len: int):
  total_bits = msg_len * 8
  for size in FIELD_SIZES:
    for start in range(0, total_bits - size + 1):
      for signed in (False,):
        for scale in DIST_SCALES:
          yield FieldSpec(start, size, signed, scale, 0.0)


def score_field(bank: Bank, ref_messages, spec: FieldSpec) -> MatchScore | None:
  errors = []
  selected = []
  last_key = None
  same_key = 0
  candidate_refs = 0

  for ref, messages in ref_messages:
    if not messages:
      continue
    candidate_refs += 1

    best = None
    for addr, pos, dat in messages:
      raw = extract_little(dat, spec.start, spec.size, spec.signed)
      value = raw * spec.scale + spec.offset
      if not (0.5 <= value <= 220.0):
        continue
      err = abs(value - ref.d_rel)
      key = (addr, pos)
      if best is None or err < best[0]:
        best = (err, value, addr, raw, key)

    if best is None:
      continue

    err, value, addr, raw, key = best
    gate = max(3.0, min(12.0, ref.d_rel * 0.18))
    if err <= gate:
      errors.append(err)
      selected.append((ref.d_rel, value, addr, raw))
      if last_key is not None and key[0] == last_key[0]:
        same_key += 1
      last_key = key

  if len(errors) < max(20, int(candidate_refs * 0.03)):
    return None

  sorted_errors = sorted(errors)
  p90 = sorted_errors[min(len(sorted_errors) - 1, int(0.90 * (len(sorted_errors) - 1)))]
  continuity = same_key / max(len(errors) - 1, 1)
  addr_count = len({addr for _lead_d, _value, addr, _raw in selected})
  sample = selected[::max(1, len(selected) // 5)][:5]
  return MatchScore(bank, spec, candidate_refs, len(errors), mean(errors), p90, continuity, addr_count, sample)


def run_route(route: str, files: list[Path], top: int):
  can_by_key, refs = collect_route(files, BANKS)
  print(f"\nROUTE {route}", flush=True)
  print(f"reference_leads={len(refs)}", flush=True)
  if not refs:
    return

  route_scores = []
  for bank in BANKS:
    present = sum(1 for addr in range(bank.start_addr, bank.end_addr + 1) if (bank.bus, addr, bank.msg_len) in can_by_key)
    if not present:
      continue
    print(f"scanning {bank.name} bus={bank.bus} len={bank.msg_len} present={present}/{bank.end_addr - bank.start_addr + 1}", flush=True)
    ref_messages = build_bank_ref_messages(can_by_key, bank, refs)
    neg_ref_messages = shifted_ref_messages(ref_messages)
    bank_scores = []
    for spec in field_specs(bank.msg_len):
      score = score_field(bank, ref_messages, spec)
      if score is not None:
        bank_scores.append(score)
    bank_scores.sort(key=lambda s: (s.coverage, -s.mae, s.continuity), reverse=True)
    for score in bank_scores[:15]:
      neg_score = score_field(bank, neg_ref_messages, score.spec)
      if neg_score is not None:
        score.neg_coverage = neg_score.coverage
        score.neg_mae = neg_score.mae
    bank_scores.sort(key=lambda s: s.rank, reverse=True)
    route_scores.extend(bank_scores[:10])

  route_scores.sort(key=lambda s: s.rank, reverse=True)
  print("\nTOP DISTANCE FIELD HYPOTHESES")
  for score in route_scores[:top]:
    sample = "; ".join(f"lead={lead_d:.1f}->cand={value:.1f}@0x{addr:x}" for lead_d, value, addr, _raw in score.sample)
    neg_mae = "n/a" if not math.isfinite(score.neg_mae) else f"{score.neg_mae:.2f}"
    lift = score.coverage - score.neg_coverage
    print(
      f"{score.bank.name:<22} {score.bank.bus}:{score.bank.start_addr:x}-{score.bank.end_addr:x} "
      f"{score.spec.label:<18} coverage={score.coverage:.3f} "
      f"neg={score.neg_coverage:.3f} lift={lift:.3f} "
      f"matches={score.matches}/{score.refs} mae={score.mae:.2f} p90={score.p90:.2f} "
      f"neg_mae={neg_mae} "
      f"continuity={score.continuity:.2f} addrs={score.addr_count} samples=[{sample}]"
    )


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--top", type=int, default=30)
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
    run_route(route, route_files, args.top)


if __name__ == "__main__":
  main()
