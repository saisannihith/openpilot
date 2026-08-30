#!/usr/bin/env python3
"""Validate Carnival R0100 target qualification against factory SCC labels.

Offline only: decodes factory SCC_CONTROL and the ten-object 99110-R0100 bank
from the same raw CAN samples, then evaluates a complete-route holdout.
"""

from __future__ import annotations

import argparse
import capnp
import glob
import json
import math
import re
from collections import Counter, defaultdict
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
  carnival_radar_object_valid,
  decode_carnival_radar_object,
)
from opendbc.car.hyundai.values import CAR, DBC
from openpilot.tools.lib.logreader import LogReader, ReadMode


MAX_OBJECT_AGE_NS = int(0.15e9)
MAX_MODEL_AGE_NS = int(0.20e9)
MIN_TARGET_SAMPLES = 100
CARNIVAL_4TH_GEN_SCC_CONTROL_ADDR = 0x1A0


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
  return sorted(paths, key=lambda path: (route_name(path), segment_number(path), str(path)))


def percentile(values: list[float], q: float) -> float | None:
  return round(float(np.percentile(values, q)), 4) if values else None


def stats(values: list[float]) -> dict[str, Any]:
  return {
    "samples": len(values),
    "p50": percentile(values, 50),
    "p95": percentile(values, 95),
    "p99": percentile(values, 99),
    "max": round(max(values), 4) if values else None,
  }


class SccDecoder:
  def __init__(self) -> None:
    dbc = DBCReader(DBC[CAR.KIA_CARNIVAL_4TH_GEN][Bus.pt])
    self.signals = list(dbc.addr_to_msg[CARNIVAL_4TH_GEN_SCC_CONTROL_ADDR].sigs.values())

  def decode(self, dat: bytes) -> dict[str, float]:
    values = {}
    for sig in self.signals:
      raw = get_raw_value(dat, sig)
      if sig.is_signed:
        raw -= ((raw >> (sig.size - 1)) & 1) * (1 << sig.size)
      values[sig.name] = raw * sig.factor + sig.offset
    return values


@dataclass
class ObjectState:
  track_id: int
  time: int
  d_rel: float
  y_rel: float
  v_rel: float
  state: int
  state_alt: int
  quality: int
  metadata: int
  persistence: int
  distance_rate_residual: float | None


@dataclass
class ModelPath:
  time: int
  x: list[float]
  y: list[float]


def model_path_error(obj: ObjectState, model: ModelPath | None, now: int) -> float | None:
  if model is None or now - model.time > MAX_MODEL_AGE_NS or len(model.x) < 2 or len(model.x) != len(model.y):
    return None
  object_x = obj.d_rel + 1.52
  if object_x < model.x[0] or object_x > model.x[-1]:
    return None
  return abs(-obj.y_rel - float(np.interp(object_x, model.x, model.y)))


def choose_oem_match(objects: list[ObjectState], scc_d: float, scc_v: float) -> tuple[ObjectState | None, float, float]:
  if not objects:
    return None, math.inf, math.inf
  ranked = sorted(objects, key=lambda obj: (
    abs(obj.d_rel - scc_d) / max(0.5, 0.015 * max(scc_d, 1.0)) + abs(obj.v_rel - scc_v) / 0.5,
    abs(obj.d_rel - scc_d),
  ))
  best = ranked[0]
  d_error = abs(best.d_rel - scc_d)
  v_error = abs(best.v_rel - scc_v)
  if d_error > max(1.0, 0.025 * max(scc_d, 1.0)) or v_error > 1.25:
    return None, d_error, v_error
  return best, d_error, v_error


def candidate_passes(row: dict[str, Any], path_threshold: float, y_threshold: float,
                     min_persistence: int, max_rate_residual: float) -> bool:
  return bool(
    row["persistence"] >= min_persistence and
    row["pathError"] is not None and row["pathError"] <= path_threshold and
    abs(row["yRel"]) <= y_threshold and
    (row["distanceRateResidual"] is None or row["distanceRateResidual"] <= max_rate_residual)
  )


def select_candidate(rows: list[dict[str, Any]], path_threshold: float, y_threshold: float,
                     min_persistence: int, max_rate_residual: float,
                     previous_track_id: int | None) -> int | None:
  accepted = [row for row in rows if candidate_passes(
    row, path_threshold, y_threshold, min_persistence, max_rate_residual,
  )]
  if not accepted:
    return None

  def score(row: dict[str, Any]) -> tuple[float, float, int]:
    continuity_bonus = -0.35 if row["trackId"] == previous_track_id else 0.0
    rate_penalty = min(finite(row["distanceRateResidual"], 0.0), 4.0) * 0.10
    return (finite(row["pathError"], 99.0) + continuity_bonus + rate_penalty,
            abs(row["yRel"]), row["trackId"])

  return min(accepted, key=score)["trackId"]


def evaluate_rule(samples: list[dict[str, Any]], routes: set[str], path_threshold: float, y_threshold: float,
                  min_persistence: int, max_rate_residual: float) -> dict[str, Any]:
  selected = correct = target_samples = false_promotions = misses = switches = 0
  previous_by_route: dict[str, int | None] = {}
  considered = 0
  for sample in samples:
    if sample["route"] not in routes:
      continue
    considered += 1
    previous = previous_by_route.get(sample["route"])
    choice = select_candidate(
      sample["candidates"], path_threshold, y_threshold, min_persistence, max_rate_residual, previous,
    )
    target = sample["oemTrackId"]
    target_samples += target is not None
    if choice is not None:
      selected += 1
      correct += choice == target
      false_promotions += target is None or choice != target
      switches += previous is not None and choice != previous
      previous_by_route[sample["route"]] = choice
    else:
      misses += target is not None
      previous_by_route[sample["route"]] = None

  return {
    "samples": considered,
    "targetSamples": target_samples,
    "selectedSamples": selected,
    "correctSelections": correct,
    "recall": round(correct / max(target_samples, 1), 6),
    "precision": round(correct / max(selected, 1), 6),
    "falsePromotions": false_promotions,
    "misses": misses,
    "switches": switches,
  }


def scan(paths: list[Path]) -> dict[str, Any]:
  decoder = SccDecoder()
  objects: dict[int, ObjectState] = {}
  previous_raw: dict[int, tuple[int, float]] = {}
  latest_model: ModelPath | None = None
  current_route = ""
  samples: list[dict[str, Any]] = []
  group_counts: Counter[str] = Counter()
  group_matches: Counter[str] = Counter()
  d_errors: list[float] = []
  v_errors: list[float] = []
  files_by_route: Counter[str] = Counter()
  car_params_modes: dict[str, set[bool]] = defaultdict(set)

  for path in paths:
    route = route_name(path)
    files_by_route[route] += 1
    if current_route and route != current_route:
      objects.clear()
      previous_raw.clear()
      latest_model = None
    current_route = route

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      which = msg.which()
      now = int(msg.logMonoTime)
      if which == "carParams":
        car_params_modes[route].add(bool(msg.carParams.openpilotLongitudinalControl))
      elif which == "modelV2":
        position = msg.modelV2.position
        latest_model = ModelPath(now, [finite(value) for value in position.x], [finite(value) for value in position.y])
      elif which != "can":
        continue

      try:
        frames = [(int(frame.address), bytes(frame.dat), int(frame.src)) for frame in msg.can]
      except capnp.KjException:
        # A small number of StarPilot rlogs contain cached enum/union mismatches.
        # They are not CAN samples and must not abort the complete-route audit.
        continue
      for address, dat, src in frames:
        if src != 1 or not (CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR) or len(dat) != 32:
          continue
        for offset in (0, 128):
          obj = decode_carnival_radar_object(dat, offset)
          if not carnival_radar_object_valid(obj):
            continue
          previous = objects.get(obj.raw_track_id)
          persistence = previous.persistence + 1 if previous is not None and now - previous.time <= MAX_OBJECT_AGE_NS else 1
          raw_previous = previous_raw.get(obj.raw_track_id)
          rate_residual = None
          if raw_previous is not None:
            previous_time, previous_distance = raw_previous
            dt = (now - previous_time) / 1e9
            if 0.02 <= dt <= 0.20:
              rate_residual = abs((obj.d_rel - previous_distance) / dt - obj.v_rel)
          previous_raw[obj.raw_track_id] = (now, obj.d_rel)
          objects[obj.raw_track_id] = ObjectState(
            track_id=obj.raw_track_id, time=now, d_rel=obj.d_rel, y_rel=obj.y_rel, v_rel=obj.v_rel,
            state=obj.state, state_alt=obj.state_alt, quality=obj.quality_byte, metadata=obj.metadata_50_63,
            persistence=persistence, distance_rate_residual=rate_residual,
          )
      for track_id in [track_id for track_id, obj in objects.items() if now - obj.time > MAX_OBJECT_AGE_NS]:
        objects.pop(track_id, None)

      for address, dat, src in frames:
        if address != CARNIVAL_4TH_GEN_SCC_CONTROL_ADDR or src >= 128 or len(dat) != 32:
          continue
        scc = decoder.decode(dat)
        scc_d = finite(scc.get("ACC_ObjDist"), -1.0)
        scc_v = finite(scc.get("ACC_ObjRelSpd"), 0.0)
        key = "/".join(str(int(round(finite(scc.get(name), -1)))) for name in
                       ("ObjValid", "OBJ_STATUS", "ACCMode", "MainMode_ACC", "CRUISE_STANDSTILL"))
        group_counts[key] += 1
        fresh = [obj for obj in objects.values() if 0 <= now - obj.time <= MAX_OBJECT_AGE_NS]
        oem, d_error, v_error = choose_oem_match(fresh, scc_d, scc_v)
        if oem is not None:
          group_matches[key] += 1
          d_errors.append(d_error)
          v_errors.append(v_error)
        candidates = [{
          "trackId": obj.track_id, "dRel": obj.d_rel, "yRel": obj.y_rel, "vRel": obj.v_rel,
          "state": obj.state, "stateAlt": obj.state_alt, "quality": obj.quality,
          "metadata": obj.metadata, "persistence": obj.persistence,
          "distanceRateResidual": obj.distance_rate_residual,
          "pathError": model_path_error(obj, latest_model, now),
        } for obj in fresh]
        samples.append({
          "route": route, "time": now, "sccSource": src, "sccDistance": scc_d,
          "sccVelocity": scc_v, "sccGroup": key,
          "oemTrackId": oem.track_id if oem is not None else None,
          "candidates": candidates,
        })

  routes = sorted(files_by_route)
  train_routes = {routes[0]} if routes else set()
  holdout_routes = set(routes[1:]) or set(train_routes)
  grid = []
  for path_threshold in (0.55, 0.70, 0.85, 1.00, 1.20, 1.50):
    for y_threshold in (1.0, 1.25, 1.5, 1.8, 2.2):
      for min_persistence in (3, 5, 8, 12):
        for max_rate_residual in (1.0, 1.5, 2.0, 3.0):
          result = evaluate_rule(samples, train_routes, path_threshold, y_threshold, min_persistence, max_rate_residual)
          objective = result["precision"] * 100.0 + result["recall"] * 10.0 - result["falsePromotions"] * 0.01
          grid.append((objective, path_threshold, y_threshold, min_persistence, max_rate_residual, result))
  best = max(grid, default=(0.0, 0.85, 1.5, 3, 1.5, {}), key=lambda item: item[0])
  _, path_threshold, y_threshold, min_persistence, max_rate_residual, train_result = best
  holdout_result = evaluate_rule(samples, holdout_routes, path_threshold, y_threshold, min_persistence, max_rate_residual)
  route_sample_counts = Counter(sample["route"] for sample in samples)
  route_target_counts = Counter(sample["route"] for sample in samples if sample["oemTrackId"] is not None)
  active_groups = {
    key for key, count in group_counts.items()
    if count >= MIN_TARGET_SAMPLES and group_matches[key] / count >= 0.80
  }
  stock_scc = bool(car_params_modes) and all(modes == {False} for modes in car_params_modes.values())
  holdout_ready = bool(
    stock_scc and holdout_result["targetSamples"] >= MIN_TARGET_SAMPLES and
    holdout_result["precision"] >= 0.995 and holdout_result["recall"] >= 0.95 and
    holdout_result["falsePromotions"] <= max(2, int(0.001 * holdout_result["samples"]))
  )
  return {
    "qualificationComplete": True,
    "safeToDisableConfirmationOnly": holdout_ready,
    "files": len(paths), "routes": routes,
    "filesByRoute": dict(sorted(files_by_route.items())),
    "samplesByRoute": dict(sorted(route_sample_counts.items())),
    "oemTargetSamplesByRoute": dict(sorted(route_target_counts.items())),
    "carParamsOpenpilotLongitudinalControl": {route: sorted(modes) for route, modes in sorted(car_params_modes.items())},
    "sccFieldGroupFormat": "ObjValid/OBJ_STATUS/ACCMode/MainMode_ACC/CRUISE_STANDSTILL",
    "sccGroups": {key: {
      "samples": count, "matchedObjectSamples": group_matches[key],
      "matchRate": round(group_matches[key] / max(count, 1), 6),
    } for key, count in group_counts.most_common()},
    "groupsWithReliableObjectLabels": sorted(active_groups),
    "oemDistanceMatchError": stats(d_errors),
    "oemVelocityMatchError": stats(v_errors),
    "trainingRoutes": sorted(train_routes), "holdoutRoutes": sorted(holdout_routes),
    "candidateRule": {
      "pathErrorMax": path_threshold, "absYRelMax": y_threshold,
      "minPersistence": min_persistence, "distanceRateResidualMax": max_rate_residual,
      "selection": "minimum model-path error with bounded continuity preference",
    },
    "trainingResult": train_result, "holdoutResult": holdout_result,
    "failedGates": [name for name, passed in (
      ("stock_scc_mode", stock_scc),
      ("holdout_target_samples", holdout_result["targetSamples"] >= MIN_TARGET_SAMPLES),
      ("holdout_precision", holdout_result["precision"] >= 0.995),
      ("holdout_recall", holdout_result["recall"] >= 0.95),
      ("holdout_false_promotions", holdout_result["falsePromotions"] <= max(2, int(0.001 * holdout_result["samples"]))),
    ) if not passed],
    "verdict": "disable_confirmation_only" if holdout_ready else "retain_confirmation_only",
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()
  report = scan(expand_paths(args.logs))
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
