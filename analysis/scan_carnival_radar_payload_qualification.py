#!/usr/bin/env python3
"""Test every R0100 metadata field as an independent OEM-target qualifier.

Factory SCC supplies the selected-target label. Every valid object in the
ten-slot R0100 bank is labelled selected or unselected, then each 1-8 bit field
in the object metadata region is evaluated with leave-one-route-out validation.
This tool is offline only and never changes control behavior.
"""

from __future__ import annotations

import argparse
import capnp
import glob
import json
import math
import re
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
from analysis.scan_carnival_radar_companion import Sample as CompanionSample
from analysis.scan_carnival_radar_companion import CAMERA_COMPANION_ADDRS
from analysis.scan_carnival_radar_companion import analyze as analyze_companion


SCC_CONTROL_ADDR = 0x1A0
MAX_OBJECT_AGE_NS = int(0.15e9)
MAX_COMPANION_AGE_NS = int(0.12e9)
POSITIVE_SCC_GROUP = "0/0/4/1/0"
NEGATIVE_SCC_GROUP = "0/0/0/0/0"
METADATA_START = 32
METADATA_END = 64
MIN_TRAIN_POSITIVES = 50
MIN_ROUTE_SAMPLES = 100


@dataclass(frozen=True)
class ObjectState:
  time: int
  track_id: int
  d_rel: float
  v_rel: float
  raw: bytes


@dataclass(frozen=True)
class Sample:
  route: str
  selected: bool
  raw: bytes


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


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)$", path.parent.name)
  return int(match.group(1)) if match else -1


def expand_paths(patterns: list[str]) -> list[Path]:
  paths: set[Path] = set()
  for pattern in patterns:
    matches = glob.glob(pattern)
    paths.update(Path(match).resolve() for match in (matches or [pattern]) if Path(match).is_file())
  return sorted(paths, key=lambda path: (route_name(path), segment_number(path)))


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


def collect(paths: list[Path]) -> tuple[list[Sample], list[CompanionSample]]:
  decoder = SccDecoder()
  samples: list[Sample] = []
  companion_samples: list[CompanionSample] = []
  for index, path in enumerate(paths, start=1):
    print(f"[{index}/{len(paths)}] {path.parent.name}", flush=True)
    objects: dict[tuple[int, int], ObjectState] = {}
    latest_frames: dict[tuple[int, int, int], tuple[int, bytes]] = {}
    emitted: set[tuple[int, int, bytes]] = set()
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      if msg.which() != "can":
        continue
      now = int(msg.logMonoTime)
      try:
        frames = [(int(frame.address), bytes(frame.dat), int(frame.src)) for frame in msg.can]
      except capnp.KjException:
        continue

      for address, dat, src in frames:
        if (src == 1 and address != SCC_CONTROL_ADDR and
            address not in CAMERA_COMPANION_ADDRS and
            not CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR):
          latest_frames[(src, address, len(dat))] = (now, dat)
        if (src != 1 or not CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR or
            len(dat) != 32 or not carnival_radar_frame_valid(address, dat)):
          continue
        for slot, offset in enumerate((0, 128)):
          obj = decode_carnival_radar_object(dat, offset)
          key = (address, slot)
          if carnival_radar_object_valid(obj):
            objects[key] = ObjectState(now, obj.raw_track_id, obj.d_rel, obj.v_rel, dat[offset // 8:offset // 8 + 16])
          else:
            objects.pop(key, None)

      for key in [key for key, obj in objects.items() if now - obj.time > MAX_OBJECT_AGE_NS]:
        objects.pop(key, None)

      for address, dat, src in frames:
        if src >= 128 or address != SCC_CONTROL_ADDR or len(dat) != 32:
          continue
        scc = decoder.decode(dat)
        group = "/".join(str(int(round(finite(scc.get(name), -1)))) for name in
                         ("ObjValid", "OBJ_STATUS", "ACCMode", "MainMode_ACC", "CRUISE_STANDSTILL"))
        if group not in (POSITIVE_SCC_GROUP, NEGATIVE_SCC_GROUP):
          continue
        fresh = [obj for obj in objects.values() if 0 <= now - obj.time <= MAX_OBJECT_AGE_NS]
        scc_distance = finite(scc.get("ACC_ObjDist"), -1.0)
        target = choose_oem_match(fresh, scc_distance, finite(scc.get("ACC_ObjRelSpd"), 0.0))
        if group == POSITIVE_SCC_GROUP and target is None and scc_distance > 0.5:
          continue
        for obj in fresh:
          emission_key = (obj.time, obj.track_id, obj.raw)
          if emission_key in emitted:
            continue
          emitted.add(emission_key)
          samples.append(Sample(route_name(path), target is not None and obj == target, obj.raw))
        snapshot = {
          key: frame for key, (frame_time, frame) in latest_frames.items()
          if 0 <= now - frame_time <= MAX_COMPANION_AGE_NS
        }
        companion_samples.append(CompanionSample(
          route_name(path), segment_number(path), target is not None,
          target.track_id if target is not None else None, snapshot,
        ))
  return samples, companion_samples


def field(data: np.ndarray, start: int, width: int) -> np.ndarray:
  byte = start // 8
  shift = start % 8
  values = data[:, byte].astype(np.uint16)
  if shift + width > 8:
    values |= data[:, byte + 1].astype(np.uint16) << 8
  return (values >> shift) & ((1 << width) - 1)


def metrics(predictions: np.ndarray, labels: np.ndarray) -> dict[str, Any]:
  tp = int(np.sum(predictions & labels))
  fp = int(np.sum(predictions & ~labels))
  fn = int(np.sum(~predictions & labels))
  tn = len(labels) - tp - fp - fn
  return {
    "samples": len(labels), "positives": int(np.sum(labels)),
    "tp": tp, "fp": fp, "fn": fn, "tn": tn,
    "precision": round(tp / max(tp + fp, 1), 6),
    "recall": round(tp / max(tp + fn, 1), 6),
  }


def analyze(samples: list[Sample]) -> dict[str, Any]:
  routes = sorted({sample.route for sample in samples})
  raw = np.frombuffer(b"".join(sample.raw for sample in samples), dtype=np.uint8).reshape(-1, 16)
  labels = np.asarray([sample.selected for sample in samples], dtype=bool)
  route_values = np.asarray([sample.route for sample in samples])
  candidates = []

  for width in range(1, 9):
    for start in range(METADATA_START, METADATA_END - width + 1):
      values = field(raw, start, width)
      bins = 1 << width
      all_positive_counts = np.bincount(values[labels], minlength=bins)
      all_negative_counts = np.bincount(values[~labels], minlength=bins)
      supported = all_positive_counts >= MIN_TRAIN_POSITIVES
      value_precision = all_positive_counts / np.maximum(all_positive_counts + all_negative_counts, 1)
      best_value = int(np.argmax(np.where(supported, value_precision, -1.0))) if np.any(supported) else -1
      folds = []
      accepted_by_fold = []
      qualified = True
      for holdout_route in routes:
        holdout_mask = route_values == holdout_route
        train_mask = ~holdout_mask
        train_values = values[train_mask]
        train_labels = labels[train_mask]
        holdout_values = values[holdout_mask]
        holdout_labels = labels[holdout_mask]
        positive_counts = np.bincount(train_values[train_labels], minlength=bins)
        negative_counts = np.bincount(train_values[~train_labels], minlength=bins)
        accepted_mask = ((positive_counts >= MIN_TRAIN_POSITIVES) &
                         (positive_counts / np.maximum(positive_counts + negative_counts, 1) >= 0.98))
        accepted = np.flatnonzero(accepted_mask)
        result = metrics(accepted_mask[holdout_values], holdout_labels)
        folds.append({"route": holdout_route, "acceptedValues": accepted.tolist(), **result})
        accepted_by_fold.append(tuple(accepted.tolist()))
        if result["samples"] < MIN_ROUTE_SAMPLES:
          qualified = False
        elif result["positives"]:
          qualified &= result["precision"] >= 0.995 and result["recall"] >= 0.95 and result["fp"] <= 2
        else:
          qualified &= result["fp"] == 0

      active_folds = [fold for fold in folds if fold["tp"] + fold["fp"]]
      positive_recalls = [fold["recall"] for fold in folds if fold["positives"]]
      candidates.append({
        "start": start, "width": width,
        "bestValue": best_value,
        "bestValuePrecision": round(float(value_precision[best_value]), 6) if best_value >= 0 else 0.0,
        "bestValueRecall": round(float(all_positive_counts[best_value] / max(int(np.sum(labels)), 1)), 6)
                           if best_value >= 0 else 0.0,
        "bestValueSelectedSamples": int(all_positive_counts[best_value]) if best_value >= 0 else 0,
        "bestValueUnselectedSamples": int(all_negative_counts[best_value]) if best_value >= 0 else 0,
        "sameAcceptedValuesEveryFold": len(set(accepted_by_fold)) == 1,
        "worstPrecision": min((fold["precision"] for fold in active_folds), default=0.0),
        "worstRecall": min(positive_recalls, default=0.0),
        "totalFalsePositives": sum(fold["fp"] for fold in folds),
        "totalFalseNegatives": sum(fold["fn"] for fold in folds),
        "qualified": bool(qualified), "folds": folds,
      })

  candidates.sort(key=lambda row: (
    row["qualified"], row["worstPrecision"], row["worstRecall"], row["bestValuePrecision"], row["bestValueRecall"],
    -row["totalFalsePositives"], -row["totalFalseNegatives"],
  ), reverse=True)

  def value_distribution(start: int, width: int) -> list[dict[str, Any]]:
    values = field(raw, start, width)
    bins = 1 << width
    selected = np.bincount(values[labels], minlength=bins)
    unselected = np.bincount(values[~labels], minlength=bins)
    return [
      {"value": value, "selected": int(selected[value]), "unselected": int(unselected[value]),
       "precision": round(float(selected[value] / max(selected[value] + unselected[value], 1)), 6),
       "recall": round(float(selected[value] / max(int(np.sum(labels)), 1)), 6)}
      for value in range(bins) if selected[value] or unselected[value]
    ]

  return {
    "routes": routes, "samples": len(samples),
    "selectedSamples": int(np.sum(labels)), "unselectedSamples": int(np.sum(~labels)),
    "fieldRegion": [METADATA_START, METADATA_END],
    "independentQualifierFound": any(candidate["qualified"] for candidate in candidates),
    "state55Width4": next(candidate for candidate in candidates if candidate["start"] == 55 and candidate["width"] == 4),
    "knownFieldDistributions": {
      "quality32Width8": value_distribution(32, 8),
      "stateAlt51Width4": value_distribution(51, 4),
      "state55Width3": value_distribution(55, 3),
      "state55Width4": value_distribution(55, 4),
    },
    "topCandidates": candidates[:50],
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("paths", nargs="+")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()
  paths = expand_paths(args.paths)
  samples, companion_samples = collect(paths)
  report = analyze(samples)
  report["companion"] = analyze_companion(companion_samples)
  report["recommendedControlPath"] = {
    "publishMeasuredObjects": True,
    "associateWithModelGeometryAndVelocity": True,
    "useBoundedVelocityFusionAfterAssociation": True,
    "allowStrictQualifiedLowSpeedRadarOnly": True,
    "allowIndependentHighwayRadarPromotion": False,
    "reason": "No object-local or companion OEM-selected-target qualifier passed route holdouts",
  }
  report["files"] = len(paths)
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
