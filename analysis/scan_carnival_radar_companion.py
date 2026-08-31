#!/usr/bin/env python3
"""Search Carnival CAN traffic for an independent R0100 target qualifier.

Factory SCC supplies the target label. The R0100 object bank and SCC_CONTROL
are excluded from candidate data so a passing result cannot simply rediscover
the fields used to create the label. Candidate rules are learned on early
segments and evaluated on later route segments.
"""

from __future__ import annotations

import argparse
import capnp
import glob
import json
import math
import re
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from opendbc.can.dbc import DBC as DBCReader
from opendbc.can.parser import get_raw_value
from opendbc.car import Bus
from opendbc.car.hyundai.radar_interface import (
  CARNIVAL_4TH_GEN_OBJECT_END_ADDR,
  CARNIVAL_4TH_GEN_OBJECT_START_ADDR,
  carnival_radar_frame_valid,
  carnival_radar_object_valid,
  decode_carnival_radar_object,
)
from opendbc.car.hyundai.values import CAR, DBC
from openpilot.tools.lib.logreader import LogReader, ReadMode


SCC_CONTROL_ADDR = 0x1A0
MAX_OBJECT_AGE_NS = int(0.15e9)
MAX_COMPANION_AGE_NS = int(0.12e9)
# Known MFC/camera families from hyundai_canfd_generated.dbc. Correlation in
# these frames cannot independently qualify a radar target.
CAMERA_COMPANION_ADDRS = {
  0x185, 0x1B6, 0x1B7, 0x1B8, 0x1B9, 0x1FB,
  0x2A2, 0x2A3, 0x2A4, 0x2BB, 0x2BC, 0x2BD, 0x2BE,
}
POSITIVE_SCC_GROUP = "0/0/4/1/0"
NEGATIVE_SCC_GROUP = "0/0/0/0/0"
MIN_VALUE_TRAIN_SAMPLES = 50
MIN_HOLDOUT_SAMPLES = 100


@dataclass(frozen=True)
class ObjectState:
  time: int
  track_id: int
  d_rel: float
  v_rel: float


@dataclass(frozen=True)
class Sample:
  route: str
  segment: int
  positive: bool
  target_track_id: int | None
  frames: dict[tuple[int, int, int], bytes]


class SccDecoder:
  def __init__(self) -> None:
    dbc = DBCReader(DBC[CAR.KIA_CARNIVAL_4TH_GEN][Bus.pt])
    self.signals = list(dbc.addr_to_msg[SCC_CONTROL_ADDR].sigs.values())

  def decode(self, dat: bytes) -> dict[str, float]:
    values = {}
    for sig in self.signals:
      raw = get_raw_value(dat, sig)
      if sig.is_signed:
        raw -= ((raw >> (sig.size - 1)) & 1) * (1 << sig.size)
      values[sig.name] = raw * sig.factor + sig.offset
    return values


def finite(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if math.isfinite(result) else default


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)$", path.parent.name)
  return int(match.group(1)) if match else -1


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def expand_paths(patterns: list[str]) -> list[Path]:
  paths: set[Path] = set()
  for pattern in patterns:
    matches = glob.glob(pattern)
    paths.update(Path(match).resolve() for match in (matches or [pattern]) if Path(match).is_file())
  return sorted(paths, key=lambda path: (segment_number(path), str(path)))


def extract(dat: bytes, start: int, width: int) -> int:
  return (int.from_bytes(dat, "little") >> start) & ((1 << width) - 1)


def choose_oem_match(objects: list[ObjectState], scc_d: float, scc_v: float) -> ObjectState | None:
  if not objects or not 0.5 <= scc_d <= 200.0:
    return None
  best = min(objects, key=lambda obj: (
    abs(obj.d_rel - scc_d) / max(0.5, 0.015 * max(scc_d, 1.0)) + abs(obj.v_rel - scc_v) / 0.5,
    abs(obj.d_rel - scc_d),
  ))
  if abs(best.d_rel - scc_d) > max(1.0, 0.025 * max(scc_d, 1.0)) or abs(best.v_rel - scc_v) > 1.25:
    return None
  return best


def metrics(predictions: np.ndarray, labels: np.ndarray) -> dict[str, Any]:
  tp = int(np.sum(predictions & labels))
  fp = int(np.sum(predictions & ~labels))
  fn = int(np.sum(~predictions & labels))
  tn = len(labels) - tp - fp - fn
  return {
    "samples": len(labels), "positiveSamples": int(np.sum(labels)),
    "tp": tp, "fp": fp, "fn": fn, "tn": tn,
    "precision": round(tp / max(tp + fp, 1), 6),
    "recall": round(tp / max(tp + fn, 1), 6),
  }


def collect(paths: list[Path]) -> tuple[list[Sample], Counter]:
  decoder = SccDecoder()
  samples: list[Sample] = []
  inventory: Counter = Counter()

  for index, path in enumerate(paths, start=1):
    print(f"[{index}/{len(paths)}] {path.parent.name}", flush=True)
    objects: dict[int, ObjectState] = {}
    latest_frames: dict[tuple[int, int, int], tuple[int, bytes]] = {}
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      if msg.which() != "can":
        continue
      now = int(msg.logMonoTime)
      try:
        frames = [(int(frame.address), bytes(frame.dat), int(frame.src)) for frame in msg.can]
      except capnp.KjException:
        continue

      for address, dat, src in frames:
        if src < 128:
          inventory[(src, address, len(dat))] += 1
        if src == 1 and CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR and len(dat) == 32:
          if not carnival_radar_frame_valid(address, dat):
            continue
          for offset in (0, 128):
            obj = decode_carnival_radar_object(dat, offset)
            if carnival_radar_object_valid(obj):
              objects[obj.raw_track_id] = ObjectState(now, obj.raw_track_id, obj.d_rel, obj.v_rel)
          continue
        # The R0100 bank and its plausible companion families are on physical
        # bus 1. Excluding other buses also prevents duplicated gateway traffic
        # from looking like an independent qualifier.
        if src == 1 and address != SCC_CONTROL_ADDR and address not in CAMERA_COMPANION_ADDRS:
          latest_frames[(src, address, len(dat))] = (now, dat)

      stale = [track_id for track_id, obj in objects.items() if now - obj.time > MAX_OBJECT_AGE_NS]
      for track_id in stale:
        objects.pop(track_id, None)

      for address, dat, src in frames:
        if src >= 128 or address != SCC_CONTROL_ADDR or len(dat) != 32:
          continue
        scc = decoder.decode(dat)
        group = "/".join(str(int(round(finite(scc.get(name), -1)))) for name in
                         ("ObjValid", "OBJ_STATUS", "ACCMode", "MainMode_ACC", "CRUISE_STANDSTILL"))
        if group not in (POSITIVE_SCC_GROUP, NEGATIVE_SCC_GROUP):
          continue
        fresh_objects = [obj for obj in objects.values() if 0 <= now - obj.time <= MAX_OBJECT_AGE_NS]
        scc_distance = finite(scc.get("ACC_ObjDist"), -1.0)
        target = choose_oem_match(
          fresh_objects, scc_distance, finite(scc.get("ACC_ObjRelSpd"), 0.0),
        )
        # ACCMode 4 with a zero object distance is the most useful negative:
        # SCC is active, but it has not selected a target. This prevents the
        # scan from merely learning an ACC-active status bit.
        if group == POSITIVE_SCC_GROUP and target is None and scc_distance > 0.5:
          continue
        positive = target is not None
        snapshot = {
          key: frame for key, (frame_time, frame) in latest_frames.items()
          if 0 <= now - frame_time <= MAX_COMPANION_AGE_NS
        }
        samples.append(Sample(route_name(path), segment_number(path), positive,
                              target.track_id if target else None, snapshot))
  return samples, inventory


def analyze(samples: list[Sample]) -> dict[str, Any]:
  routes = sorted({sample.route for sample in samples})
  if len(routes) > 1:
    positive_by_route = Counter(sample.route for sample in samples if sample.positive)
    training_route = max(routes, key=lambda route: positive_by_route[route])
    training_routes = {training_route}
    holdout_routes = set(routes) - training_routes
    train = [sample for sample in samples if sample.route in training_routes]
    holdout = [sample for sample in samples if sample.route in holdout_routes]
  else:
    positive_segments = sorted({sample.segment for sample in samples if sample.positive})
    negative_segments = sorted({sample.segment for sample in samples if not sample.positive})
    holdout_segment_set = {
      *(positive_segments[-1:] if len(positive_segments) > 1 else []),
      *(negative_segments[-1:] if len(negative_segments) > 1 else []),
    }
    train = [sample for sample in samples if sample.segment not in holdout_segment_set]
    holdout = [sample for sample in samples if sample.segment in holdout_segment_set]
    training_routes = set(routes)
    holdout_routes = set(routes)
  keys = sorted({key for sample in samples for key in sample.frames})
  candidates = []
  exact_id = []

  for key in keys:
    train_rows = [(sample, sample.frames[key]) for sample in train if key in sample.frames]
    holdout_rows = [(sample, sample.frames[key]) for sample in holdout if key in sample.frames]
    if len(train_rows) < MIN_HOLDOUT_SAMPLES or len(holdout_rows) < MIN_HOLDOUT_SAMPLES:
      continue
    length = min(len(train_rows[0][1]), len(holdout_rows[0][1]))
    max_bits = length * 8
    train_data = np.frombuffer(b"".join(dat[:length] for _, dat in train_rows), dtype=np.uint8).reshape(-1, length)
    holdout_data = np.frombuffer(b"".join(dat[:length] for _, dat in holdout_rows), dtype=np.uint8).reshape(-1, length)
    train_labels = np.asarray([sample.positive for sample, _ in train_rows], dtype=bool)
    holdout_labels = np.asarray([sample.positive for sample, _ in holdout_rows], dtype=bool)
    train_target_ids = np.asarray([
      sample.target_track_id if sample.target_track_id is not None else -1 for sample, _ in train_rows
    ], dtype=np.int16)
    holdout_target_ids = np.asarray([
      sample.target_track_id if sample.target_track_id is not None else -1 for sample, _ in holdout_rows
    ], dtype=np.int16)

    def field(data: np.ndarray, start: int, width: int) -> np.ndarray:
      byte = start // 8
      shift = start % 8
      values = data[:, byte].astype(np.uint16)
      if shift + width > 8:
        values |= data[:, byte + 1].astype(np.uint16) << 8
      return (values >> shift) & ((1 << width) - 1)

    positive_train = train_labels & (train_target_ids >= 0)
    positive_holdout = holdout_labels & (holdout_target_ids >= 0)
    for start in range(max_bits - 7):
      train_values = field(train_data, start, 8)
      holdout_values = field(holdout_data, start, 8)
      train_matches = int(np.sum(positive_train & (train_values == train_target_ids)))
      holdout_matches = int(np.sum(positive_holdout & (holdout_values == holdout_target_ids)))
      if train_matches or holdout_matches:
        exact_id.append({
          "source": key[0], "address": key[1], "length": key[2], "start": start,
          "trainingMatches": train_matches, "trainingSamples": int(np.sum(positive_train)),
          "holdoutMatches": holdout_matches, "holdoutSamples": int(np.sum(positive_holdout)),
          "holdoutMatchRate": round(holdout_matches / max(int(np.sum(positive_holdout)), 1), 6),
        })

    for width in range(1, 9):
      for start in range(max_bits - width + 1):
        train_values = field(train_data, start, width)
        holdout_values = field(holdout_data, start, width)
        bins = 1 << width
        positive_counts = np.bincount(train_values[train_labels], minlength=bins)
        negative_counts = np.bincount(train_values[~train_labels], minlength=bins)
        accepted_mask = ((positive_counts >= MIN_VALUE_TRAIN_SAMPLES) &
                         (positive_counts / np.maximum(positive_counts + negative_counts, 1) >= 0.98))
        accepted = np.flatnonzero(accepted_mask)
        if not len(accepted):
          continue
        train_predictions = accepted_mask[train_values]
        holdout_predictions = accepted_mask[holdout_values]
        train_result = metrics(train_predictions, train_labels)
        holdout_result = metrics(holdout_predictions, holdout_labels)
        if train_result["recall"] < 0.10 and holdout_result["recall"] < 0.10:
          continue
        candidates.append({
          "source": key[0], "address": key[1], "length": key[2],
          "start": start, "width": width, "acceptedValues": accepted.tolist(),
          "training": train_result, "holdout": holdout_result,
        })

  candidates.sort(key=lambda row: (
    row["holdout"]["precision"] >= 0.995,
    row["holdout"]["recall"], row["holdout"]["precision"],
    -row["holdout"]["fp"],
  ), reverse=True)
  exact_id.sort(key=lambda row: (row["holdoutMatchRate"], row["holdoutMatches"]), reverse=True)
  holdout_has_both_classes = bool(any(sample.positive for sample in holdout) and
                                  any(not sample.positive for sample in holdout))
  return {
    "trainingRoutes": sorted(training_routes), "holdoutRoutes": sorted(holdout_routes),
    "trainingSegments": sorted({sample.segment for sample in train}),
    "holdoutSegments": sorted({sample.segment for sample in holdout}),
    "trainingSamples": len(train), "holdoutSamples": len(holdout),
    "positiveTrainingSamples": sum(sample.positive for sample in train),
    "positiveHoldoutSamples": sum(sample.positive for sample in holdout),
    "holdoutHasBothClasses": holdout_has_both_classes,
    "exactTargetIdCandidates": exact_id[:50],
    "targetValidityCandidates": candidates[:100],
    "independentQualifierFound": holdout_has_both_classes and any(
      row["holdout"]["precision"] >= 0.995 and row["holdout"]["recall"] >= 0.95 and row["holdout"]["fp"] <= 2
      for row in candidates
    ),
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("paths", nargs="+")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()
  paths = expand_paths(args.paths)
  samples, inventory = collect(paths)
  report = analyze(samples)
  report["files"] = len(paths)
  report["inventory"] = [
    {"source": source, "address": address, "length": length, "samples": count}
    for (source, address, length), count in inventory.most_common()
  ]
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
