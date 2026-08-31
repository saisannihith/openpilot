#!/usr/bin/env python3
"""Audit unused Carnival R0100 object metadata against independent lead labels.

This is an offline research tool. It never changes radar publication or control.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from openpilot.tools.lib.logreader import LogReader, ReadMode
from opendbc.car.hyundai.radar_interface import carnival_radar_frame_valid


RADAR_TO_CAMERA = 1.52
MAX_RADAR_AGE = 0.12
MIN_MODEL_LEAD_PROB = 0.35
NIS_GATE = 11.345
PRIMARY_ADDR = 0x180
PRIMARY_SLOT = 1
PRIMARY_QUALITY = 0xFF
UNKNOWN_BITS = (*range(0, 32), 40, 41, *range(50, 64), 77, 89, 90, *range(102, 106), 114, 115)
METADATA_WINDOWS = tuple(
  (start, width)
  for width in range(2, 7)
  for start in range(50, 65 - width)
) + tuple(
  (start, width)
  for width in range(2, 9)
  for start in range(16, 33 - width)
)


@dataclass(frozen=True)
class RadarObject:
  t: float
  addr: int
  slot: int
  raw: int
  prefix_word: int
  prefix_byte2: int
  prefix_byte3: int
  valid_count: int
  status_40_41: int
  track_id: int
  metadata_50_63: int
  state_alt_candidate: int
  state_candidate: int
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  a_rel: float
  heartbeat: int


def extract(raw: int, start: int, size: int, signed: bool = False) -> int:
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


def decode(t: float, addr: int, dat: bytes) -> list[RadarObject]:
  raw_message = int.from_bytes(dat, "little", signed=False)
  objects = []
  for slot, offset in ((1, 0), (2, 128)):
    raw = (raw_message >> offset) & ((1 << 128) - 1)
    objects.append(RadarObject(
      t=t,
      addr=addr,
      slot=slot,
      raw=raw,
      prefix_word=extract(raw, 0, 32),
      prefix_byte2=extract(raw, 16, 8),
      prefix_byte3=extract(raw, 24, 8),
      valid_count=extract(raw, 32, 8),
      status_40_41=extract(raw, 40, 2),
      track_id=extract(raw, 42, 8),
      metadata_50_63=extract(raw, 50, 14),
      state_alt_candidate=extract(raw, 51, 4),
      state_candidate=extract(raw, 55, 3),
      d_rel=extract(raw, 64, 13) * 0.05,
      y_rel=extract(raw, 78, 11, True) * 0.05,
      v_rel=extract(raw, 91, 11, True) * 0.05 + 2.4,
      yv_rel=extract(raw, 106, 8) * 0.2 - 25.0,
      a_rel=extract(raw, 116, 8, True) * 0.1,
      heartbeat=extract(raw, 124, 4),
    ))
  return objects


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)$", path.parent.name)
  return int(match.group(1)) if match else -1


def iter_logs(root: Path, route_filters: list[str]) -> dict[str, list[Path]]:
  by_segment: dict[Path, list[Path]] = defaultdict(list)
  for path in root.rglob("*log.zst"):
    if re.search(r"--\d+$", path.parent.name):
      by_segment[path.parent].append(path)

  routes: dict[str, list[Path]] = defaultdict(list)
  for files in by_segment.values():
    rlogs = [path for path in files if path.name == "rlog.zst"]
    selected = rlogs or [path for path in files if path.name == "qlog.zst"]
    for path in selected[:1]:
      route = route_name(path)
      if not route_filters or any(token in route for token in route_filters):
        routes[route].append(path)
  for paths in routes.values():
    paths.sort(key=segment_number)
  return dict(sorted(routes.items()))


def base_valid(obj: RadarObject, now: float) -> bool:
  return (
    obj.track_id != 0 and
    0.5 <= obj.d_rel <= 220.0 and
    abs(obj.y_rel) <= 50.0 and
    abs(obj.v_rel) <= 60.0 and
    0.0 <= now - obj.t <= MAX_RADAR_AGE
  )


def update_group(groups: dict[str, dict[str, Counter]], name: str, obj: RadarObject) -> None:
  group = groups[name]
  group["samples"]["count"] += 1
  group["prefixWord"][obj.prefix_word] += 1
  group["prefixByte2"][obj.prefix_byte2] += 1
  group["prefixByte3"][obj.prefix_byte3] += 1
  group["validCount"][obj.valid_count] += 1
  group["status40_41"][obj.status_40_41] += 1
  group["heartbeat"][obj.heartbeat] += 1
  group["stateCandidate"][obj.state_candidate] += 1
  group["stateAltCandidate"][obj.state_alt_candidate] += 1
  group["metadata50_63"][obj.metadata_50_63] += 1
  for bit in UNKNOWN_BITS:
    group["bits"][bit] += (obj.raw >> bit) & 1
  for start, width in METADATA_WINDOWS:
    group["metadataWindows"][(start, width, extract(obj.raw, start, width))] += 1


def empty_groups() -> dict[str, dict[str, Counter]]:
  return defaultdict(lambda: defaultdict(Counter))


def nearest_model_match(candidates: list[RadarObject], model_d: float, model_y: float, model_v_rel: float, v_ego: float,
                        x_std: float, y_std: float, v_std: float) -> RadarObject | None:
  scored = score_model_matches(candidates, model_d, model_y, model_v_rel, v_ego, x_std, y_std, v_std)
  return min(scored, key=lambda item: item[0])[1] if scored else None


def score_model_matches(candidates: list[RadarObject], model_d: float, model_y: float, model_v_rel: float, v_ego: float,
                        x_std: float, y_std: float, v_std: float) -> list[tuple[float, RadarObject]]:
  scored = []
  for obj in candidates:
    if not (
      abs(obj.d_rel - model_d) < max(abs(model_d) * 0.22, 4.0) and
      abs(obj.y_rel + model_y) < max(1.2, 1.5 * max(y_std, 0.2)) and
      abs(obj.v_rel - model_v_rel) < max(4.0, 3.0 * max(v_std, 0.5)) and
      obj.v_rel + v_ego > -2.0
    ):
      continue
    score = (
      (obj.d_rel - model_d) ** 2 / (min(max(x_std, 0.75), 6.0) ** 2 + 0.25 ** 2) +
      (obj.y_rel + model_y) ** 2 / (min(max(y_std, 0.25), 1.5) ** 2 + 0.25 ** 2) +
      (obj.v_rel - model_v_rel) ** 2 / (min(max(v_std, 0.5), 3.0) ** 2 + 0.35 ** 2)
    )
    if math.isfinite(score) and score <= NIS_GATE:
      scored.append((score, obj))
  return scored


def nearest_scc_match(candidates: list[RadarObject], d_rel: float, v_rel: float) -> RadarObject | None:
  scored = []
  for obj in candidates:
    d_error = abs(obj.d_rel - d_rel)
    v_error = abs(obj.v_rel - v_rel)
    if d_error <= max(1.0, 0.03 * d_rel) and v_error <= 1.5:
      scored.append((d_error + 0.5 * v_error, obj))
  return min(scored, key=lambda item: item[0])[1] if scored else None


def counter_dict(counter: Counter) -> dict[str, int]:
  return {str(key): value for key, value in sorted(counter.items())}


def summarize_group(group: dict[str, Counter]) -> dict[str, Any]:
  samples = group["samples"]["count"]
  state_34 = group["stateCandidate"][3] + group["stateCandidate"][4]
  return {
    "samples": samples,
    "prefixWordTop": [
      {"value": value, "count": count}
      for value, count in group["prefixWord"].most_common(20)
    ],
    "prefixByte2": counter_dict(group["prefixByte2"]),
    "prefixByte3": counter_dict(group["prefixByte3"]),
    "validCountZeroRate": round(group["validCount"][0] / max(samples, 1), 6),
    "heartbeatZeroRate": round(group["heartbeat"][0] / max(samples, 1), 6),
    "state34Rate": round(state_34 / max(samples, 1), 6),
    "validCount": counter_dict(group["validCount"]),
    "status40_41": counter_dict(group["status40_41"]),
    "heartbeat": counter_dict(group["heartbeat"]),
    "stateCandidate": counter_dict(group["stateCandidate"]),
    "stateAltCandidate": counter_dict(group["stateAltCandidate"]),
    "metadata50_63Top": [
      {"value": value, "count": count}
      for value, count in group["metadata50_63"].most_common(20)
    ],
  }


def window_separation(groups: dict[str, dict[str, Counter]], positive: str, negative: str) -> list[dict[str, Any]]:
  positive_n = groups[positive]["samples"]["count"]
  negative_n = groups[negative]["samples"]["count"]
  results = []
  for start, width in METADATA_WINDOWS:
    positive_counts = Counter({
      value: count
      for (window_start, window_width, value), count in groups[positive]["metadataWindows"].items()
      if (window_start, window_width) == (start, width)
    })
    negative_counts = Counter({
      value: count
      for (window_start, window_width, value), count in groups[negative]["metadataWindows"].items()
      if (window_start, window_width) == (start, width)
    })
    values = set(positive_counts) | set(negative_counts)
    total_variation = 0.5 * sum(
      abs(positive_counts[value] / max(positive_n, 1) - negative_counts[value] / max(negative_n, 1))
      for value in values
    )
    positive_top = positive_counts.most_common(1)
    negative_top = negative_counts.most_common(1)
    results.append({
      "start": start,
      "width": width,
      "totalVariation": round(total_variation, 6),
      "positiveTop": positive_top[0] if positive_top else None,
      "negativeTop": negative_top[0] if negative_top else None,
    })
  return sorted(results, key=lambda item: item["totalVariation"], reverse=True)


def bit_separation(groups: dict[str, dict[str, Counter]], positive: str, negative: str) -> list[dict[str, Any]]:
  positive_n = groups[positive]["samples"]["count"]
  negative_n = groups[negative]["samples"]["count"]
  results = []
  for bit in UNKNOWN_BITS:
    positive_rate = groups[positive]["bits"][bit] / max(positive_n, 1)
    negative_rate = groups[negative]["bits"][bit] / max(negative_n, 1)
    results.append({
      "bit": bit,
      "positiveRate": round(positive_rate, 6),
      "negativeRate": round(negative_rate, 6),
      "absoluteDifference": round(abs(positive_rate - negative_rate), 6),
    })
  return sorted(results, key=lambda item: item["absoluteDifference"], reverse=True)


def analyze_route(paths: list[Path]) -> dict[str, Any]:
  groups = empty_groups()
  latest: dict[tuple[int, int], RadarObject] = {}
  previous: dict[int, RadarObject] = {}
  lifecycle = Counter()
  v_ego = 0.0
  model_events = scc_events = 0
  model_matches = scc_matches = 0
  latest_scc_reference: tuple[float, float, float] | None = None
  association_policy = Counter()
  frame_integrity = Counter()

  for index, path in enumerate(paths, start=1):
    print(f"  [{index}/{len(paths)}] {path.parent.name}", flush=True)
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      which = msg.which()
      now = float(msg.logMonoTime) / 1e9
      if which == "carState":
        v_ego = float(msg.carState.vEgo)
        continue

      if which == "can":
        scc_references = []
        batch: dict[int, RadarObject] = {}
        conflicts: set[int] = set()
        for can in msg.can:
          addr = int(can.address)
          dat = bytes(can.dat)
          if addr == 0x1A0 and len(dat) == 32 and int(can.src) < 128:
            raw = int.from_bytes(dat, "little", signed=False)
            scc_reference = (extract(raw, 24, 11) * 0.1, extract(raw, 35, 9) * 0.1 - 16.4)
            scc_references.append(scc_reference)
            latest_scc_reference = (now, *scc_reference)
          if int(can.src) != 1 or not (0x180 <= addr <= 0x184) or len(dat) != 32:
            continue
          frame_integrity["frames"] += 1
          if not carnival_radar_frame_valid(addr, dat):
            frame_integrity["crcInvalid"] += 1
            continue
          frame_integrity["crcValid"] += 1
          for obj in decode(now, addr, dat):
            latest[(addr, obj.slot)] = obj
            update_group(groups, "rawSlots", obj)
            if not base_valid(obj, now):
              update_group(groups, "inactiveSlots", obj)
              continue
            update_group(groups, "activeSlots", obj)
            update_group(groups, "allPublishedCandidates", obj)
            prior = batch.get(obj.track_id)
            if prior is not None and prior != obj:
              conflicts.add(obj.track_id)
            else:
              batch[obj.track_id] = obj

        for track_id in conflicts:
          batch.pop(track_id, None)
        for track_id, obj in batch.items():
          old = previous.get(track_id)
          if old is not None and 0.0 < obj.t - old.t <= 0.15:
            lifecycle["continuous"] += 1
            lifecycle["validCountIncrement"] += obj.valid_count == min(old.valid_count + 1, PRIMARY_QUALITY)
            lifecycle["validCountSame"] += obj.valid_count == old.valid_count
            lifecycle["status40_41Same"] += obj.status_40_41 == old.status_40_41
            lifecycle["heartbeatIncrement"] += ((obj.heartbeat - old.heartbeat) & 0xF) == 1
            lifecycle["heartbeatSame"] += obj.heartbeat == old.heartbeat
            lifecycle["stateCandidateSame"] += obj.state_candidate == old.state_candidate
            lifecycle["metadata50_63Same"] += obj.metadata_50_63 == old.metadata_50_63
            for start, width in METADATA_WINDOWS:
              lifecycle[f"window:{start}:{width}:same"] += extract(obj.raw, start, width) == extract(old.raw, start, width)
          previous[track_id] = obj

        candidates = [obj for obj in latest.values() if base_valid(obj, now)]
        for d_rel, v_rel in scc_references:
          if not (0.5 <= d_rel <= 200.0):
            continue
          scc_events += 1
          selected = nearest_scc_match(candidates, d_rel, v_rel)
          if selected is not None:
            scc_matches += 1
            update_group(groups, "stockSccSelected", selected)
          for obj in candidates:
            if selected is None or (obj.addr, obj.slot) != (selected.addr, selected.slot):
              update_group(groups, "stockSccOther", obj)
        continue

      if which != "modelV2" or not msg.modelV2.leadsV3:
        continue
      lead = msg.modelV2.leadsV3[0]
      if float(lead.prob) < MIN_MODEL_LEAD_PROB or not lead.x or not lead.y or not lead.v:
        continue
      model_events += 1
      candidates = [obj for obj in latest.values() if base_valid(obj, now)]
      model_d = float(lead.x[0]) - RADAR_TO_CAMERA
      model_y = float(lead.y[0])
      model_v_rel = float(lead.v[0]) - v_ego
      scored = score_model_matches(
        candidates, model_d, model_y, model_v_rel, v_ego,
        float(lead.xStd[0]), float(lead.yStd[0]), float(lead.vStd[0]),
      )
      selected = min(scored, key=lambda item: item[0])[1] if scored else None
      if selected is not None:
        model_matches += 1
        update_group(groups, "modelSelected", selected)
      for obj in candidates:
        if selected is None or (obj.addr, obj.slot) != (selected.addr, selected.slot):
          update_group(groups, "modelOther", obj)

      position = msg.modelV2.position
      path_x = list(position.x)
      path_y = list(position.y)
      if len(path_x) < 2 or len(path_x) != len(path_y):
        continue
      for obj in candidates:
        object_x = obj.d_rel + RADAR_TO_CAMERA
        if not (path_x[0] <= object_x <= path_x[-1]):
          continue
        residual = abs(-obj.y_rel - float(np.interp(object_x, path_x, path_y)))
        if residual <= 0.9:
          update_group(groups, "inPath", obj)
        elif residual >= 1.8:
          update_group(groups, "outOfPath", obj)

      if latest_scc_reference is not None and 0.0 <= now - latest_scc_reference[0] <= MAX_RADAR_AGE:
        _, scc_d_rel, scc_v_rel = latest_scc_reference
        if not (0.5 <= scc_d_rel <= 200.0):
          continue
        truth = nearest_scc_match(candidates, scc_d_rel, scc_v_rel)
        if truth is None or not scored:
          continue

        association_policy["truthSamples"] += 1
        best = min(scored, key=lambda item: item[0])[1]
        primary_candidates = [item for item in scored
                              if item[1].addr == PRIMARY_ADDR and item[1].slot == PRIMARY_SLOT and
                              item[1].valid_count == PRIMARY_QUALITY]
        current = min(primary_candidates, key=lambda item: item[0])[1] if primary_candidates else best

        truth_key = (truth.addr, truth.slot)
        best_key = (best.addr, best.slot)
        current_key = (current.addr, current.slot)
        association_policy["bestExact"] += best_key == truth_key
        association_policy["currentExact"] += current_key == truth_key
        association_policy["currentPrimarySelected"] += bool(primary_candidates)
        association_policy["primaryChangedBest"] += current_key != best_key
        association_policy["primaryChangedBestCorrectToWrong"] += best_key == truth_key and current_key != truth_key
        association_policy["primaryChangedBestWrongToCorrect"] += best_key != truth_key and current_key == truth_key

  continuous = lifecycle["continuous"]
  lifecycle_summary = {
    "continuousSamples": continuous,
    "validCountIncrementRate": round(lifecycle["validCountIncrement"] / max(continuous, 1), 6),
    "validCountSameRate": round(lifecycle["validCountSame"] / max(continuous, 1), 6),
    "status40_41SameRate": round(lifecycle["status40_41Same"] / max(continuous, 1), 6),
    "heartbeatIncrementRate": round(lifecycle["heartbeatIncrement"] / max(continuous, 1), 6),
    "heartbeatSameRate": round(lifecycle["heartbeatSame"] / max(continuous, 1), 6),
    "stateCandidateSameRate": round(lifecycle["stateCandidateSame"] / max(continuous, 1), 6),
    "metadata50_63SameRate": round(lifecycle["metadata50_63Same"] / max(continuous, 1), 6),
    "mostStableMetadataWindows": sorted((
      {
        "start": start,
        "width": width,
        "sameRate": round(lifecycle[f"window:{start}:{width}:same"] / max(continuous, 1), 6),
      }
      for start, width in METADATA_WINDOWS
    ), key=lambda item: item["sameRate"], reverse=True)[:20],
  }
  return {
    "files": len(paths),
    "modelEvents": model_events,
    "modelMatches": model_matches,
    "modelMatchRate": round(model_matches / max(model_events, 1), 6),
    "stockSccEvents": scc_events,
    "stockSccMatches": scc_matches,
    "stockSccMatchRate": round(scc_matches / max(scc_events, 1), 6),
    "frameIntegrity": dict(frame_integrity),
    "associationPolicy": {
      "truthSamples": association_policy["truthSamples"],
      "currentPrimaryPreferenceExact": association_policy["currentExact"],
      "currentPrimaryPreferenceExactRate": round(
        association_policy["currentExact"] / max(association_policy["truthSamples"], 1), 6),
      "bestInnovationExact": association_policy["bestExact"],
      "bestInnovationExactRate": round(
        association_policy["bestExact"] / max(association_policy["truthSamples"], 1), 6),
      "currentPrimarySelected": association_policy["currentPrimarySelected"],
      "primaryChangedBest": association_policy["primaryChangedBest"],
      "primaryChangedBestCorrectToWrong": association_policy["primaryChangedBestCorrectToWrong"],
      "primaryChangedBestWrongToCorrect": association_policy["primaryChangedBestWrongToCorrect"],
    },
    "groups": {name: summarize_group(group) for name, group in sorted(groups.items())},
    "lifecycle": lifecycle_summary,
    "modelSelectedVsOtherBits": bit_separation(groups, "modelSelected", "modelOther")[:15],
    "stockSccSelectedVsOtherBits": bit_separation(groups, "stockSccSelected", "stockSccOther")[:15],
    "inPathVsOutOfPathBits": bit_separation(groups, "inPath", "outOfPath")[:15],
    "activeVsInactiveWindows": window_separation(groups, "activeSlots", "inactiveSlots")[:20],
    "modelSelectedVsOtherWindows": window_separation(groups, "modelSelected", "modelOther")[:20],
    "stockSccSelectedVsOtherWindows": window_separation(groups, "stockSccSelected", "stockSccOther")[:20],
    "inPathVsOutOfPathWindows": window_separation(groups, "inPath", "outOfPath")[:20],
  }


def aggregate(routes: dict[str, dict[str, Any]]) -> dict[str, Any]:
  state_gate_evidence = []
  for route, result in routes.items():
    groups = result["groups"]
    model_selected = groups.get("modelSelected", {})
    scc_selected = groups.get("stockSccSelected", {})
    state_gate_evidence.append({
      "route": route,
      "modelSelectedSamples": model_selected.get("samples", 0),
      "modelSelectedState34Rate": model_selected.get("state34Rate"),
      "stockSccSelectedSamples": scc_selected.get("samples", 0),
      "stockSccSelectedState34Rate": scc_selected.get("state34Rate"),
    })
  model_routes = [row for row in state_gate_evidence if row["modelSelectedSamples"] >= 100]
  substantial_routes = [
    result for result in routes.values()
    if result["groups"].get("activeSlots", {}).get("samples", 0) >= 1000
  ]
  state_like_field_located = bool(
    substantial_routes and
    all(result["lifecycle"]["stateCandidateSameRate"] >= 0.99 for result in substantial_routes) and
    all(
      result["groups"]["inactiveSlots"]["stateCandidate"].get("0", 0) /
      max(result["groups"]["inactiveSlots"]["samples"], 1) >= 0.99
      for result in substantial_routes
    )
  )
  mrr30_state_gate_compatible = bool(
    model_routes and
    all((row["modelSelectedState34Rate"] or 0.0) >= 0.995 for row in model_routes)
  )
  state_candidate_ready = state_like_field_located and mrr30_state_gate_compatible
  if state_candidate_ready:
    verdict = "state_like_field_found_and_mrr30_state_gate_supported"
    state_reason = "States 3/4 cover independently selected leads consistently across substantial routes."
  elif state_like_field_located:
    verdict = "state_like_field_found_but_do_not_apply_mrr30_state_gate"
    state_reason = "The field separates active slots, but states 3/4 do not cover independently selected leads consistently."
  else:
    verdict = "no_cross_route_mrr30_compatible_state_gate"
    state_reason = "No MRR30-compatible state gate is stable across the analyzed routes."
  reason = " ".join((
    "R0100 supplies openpilot's required dRel/yRel/vRel point contract.",
    state_reason,
    "Classification and OEM lane assignment are not RadarPoint fields.",
  ))
  return {
    "stateGateEvidence": state_gate_evidence,
    "activeSlotSamples": sum(
      result["groups"].get("activeSlots", {}).get("samples", 0)
      for result in routes.values()
    ),
    "modelSelectedSamples": sum(row["modelSelectedSamples"] for row in state_gate_evidence),
    "stateLikeFieldLocated": state_like_field_located,
    "stateLikeField": "little-endian bits 55..57; bit 58 was constant in substantial routes",
    "mrr30StateGateCompatible": mrr30_state_gate_compatible,
    "stateCandidateReady": state_candidate_ready,
    "openpilotMinimumRadarContractComplete": True,
    "classificationDecoded": False,
    "laneAssignmentDecoded": False,
    "verdict": verdict,
    "reason": reason,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--route", action="append", default=[])
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  route_paths = iter_logs(args.log_root, args.route)
  routes = {}
  for route, paths in route_paths.items():
    print(f"Analyzing {route} ({len(paths)} segments)", flush=True)
    routes[route] = analyze_route(paths)
  report = {
    "layout": {
      "frameHeader": [
        "CRC16@0 over the complete 32-byte frame plus CAN address, Hyundai CAN-FD XOR 0x9F5B",
        "rollingCounter@16 unsigned 8-bit; 99.994% consecutive increments in fresh routes",
      ],
      "provenKinematics": ["trackId", "dRel", "yRel", "vRel"],
      "decodedDynamics": ["yvRel@106 unsigned 8-bit * 0.2 - 25.0", "aRel@116 signed 8-bit * 0.1"],
      "decodedLifecycle": ["validCount@32 unsigned 8-bit saturating at 255", "heartbeat@124"],
      "unresolvedMetadata": "slot-1 class byte 24..31, slot-2 prefix 128..159, object bits 40..41, bits 50..63, and sparse gaps 77, 89..90, 102..105, 114..115",
      "openpilotRequiredRadarPointFields": ["dRel", "yRel", "vRel"],
      "openpilotOptionalRadarPointFields": ["aRel", "yvRel"],
      "notInRadarPointSchema": ["classification", "OEM lane assignment"],
    },
    "aggregate": aggregate(routes),
    "routes": routes,
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
