#!/usr/bin/env python3
"""Validate Carnival radar objects against independent model leads.

Radar candidates are selected by longitudinal distance only. Lateral and velocity
fields are then scored independently, avoiding circular validation.
"""

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


RADAR_TO_CAMERA = 1.52
MAX_RADAR_AGE = 0.12
MIN_LEAD_PROB = 0.35
MAX_DISTANCE_ERROR = 8.0
NIS_GATE = 11.345
PREFERRED_NIS_GATE = 14.156
SWITCH_SCORE_MARGIN = 3.841


@dataclass(frozen=True)
class RadarObject:
  t: float
  addr: int
  slot: int
  track_id: int
  state: int
  state_alt: int
  distance: float
  lateral: float
  velocity: float
  heartbeat: int


def extract(raw: int, start: int, size: int, signed: bool = False) -> int:
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


def decode(t: float, addr: int, dat: bytes) -> list[RadarObject]:
  raw = int.from_bytes(dat, "little", signed=False)
  objects = []
  for slot, offset in ((1, 0), (2, 128)):
    objects.append(RadarObject(
      t=t,
      addr=addr,
      slot=slot,
      track_id=extract(raw, offset + 42, 8),
      state=extract(raw, offset + 55, 4),
      state_alt=extract(raw, offset + 51, 4),
      distance=extract(raw, offset + 64, 13) * 0.05,
      lateral=extract(raw, offset + 78, 11, True) * 0.05,
      velocity=extract(raw, offset + 91, 11, True) * 0.05 + 2.4,
      heartbeat=extract(raw, offset + 124, 4),
    ))
  return objects


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
  return ordered[index]


def stats(values: list[float]) -> dict[str, Any]:
  return {
    "samples": len(values),
    "mean": None if not values else round(mean(values), 4),
    "p50": None if not values else round(percentile(values, 50) or 0.0, 4),
    "p95": None if not values else round(percentile(values, 95) or 0.0, 4),
    "p99": None if not values else round(percentile(values, 99) or 0.0, 4),
  }


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def iter_rlogs(root: Path) -> dict[str, list[Path]]:
  routes: dict[str, list[Path]] = defaultdict(list)
  for path in root.rglob("rlog.zst"):
    routes[route_name(path)].append(path)
  for paths in routes.values():
    paths.sort(key=lambda path: int(path.parent.name.rsplit("--", 1)[-1]))
  return routes


def valid(obj: RadarObject, model_time: float) -> bool:
  # The radar's own validity contract from the decoded object layout. The
  # previously inferred state/state_alt bits overlap unrelated payload data.
  # The 4-bit counter wraps through zero, so zero is a valid sample.
  return obj.track_id != 0 and 0.5 <= obj.distance <= 220.0 and 0.0 <= model_time - obj.t <= MAX_RADAR_AGE


def normalized_innovation(obj: RadarObject, model_d: float, model_y: float, model_v_rel: float,
                          x_std: float, y_std: float, v_std: float) -> float:
  residuals = (
    (obj.distance - model_d, min(max(x_std, 0.75), 6.0), 0.25),
    (obj.lateral + model_y, min(max(y_std, 0.25), 1.5), 0.25),
    (obj.velocity - model_v_rel, min(max(v_std, 0.5), 3.0), 0.35),
  )
  return sum(error ** 2 / (model_sigma ** 2 + radar_sigma ** 2)
             for error, model_sigma, radar_sigma in residuals)


def analyze_route(paths: list[Path]) -> dict[str, Any]:
  latest: dict[tuple[int, int], RadarObject] = {}
  distance_errors: list[float] = []
  lateral_errors: list[float] = []
  velocity_errors: list[float] = []
  strict_lateral_errors: list[float] = []
  strict_velocity_errors: list[float] = []
  primary_distance_errors: list[float] = []
  primary_lateral_errors: list[float] = []
  runtime_distance_corrections: list[float] = []
  runtime_lateral_residuals: list[float] = []
  runtime_velocity_residuals: list[float] = []
  selected_channels: Counter[str] = Counter()
  selected_states: Counter[str] = Counter()
  selected_track_ids: Counter[int] = Counter()
  lead_frames = matched_frames = 0
  v_ego = 0.0
  last_selected: RadarObject | None = None
  continuity_frames = track_id_continuity = slot_continuity = 0
  runtime_matches = primary_runtime_matches = 0
  runtime_continuity = runtime_handoffs = 0
  last_runtime: tuple[float, RadarObject] | None = None
  nis_matches = nis_continuity = nis_handoffs = 0
  nis_scores: list[float] = []
  nis_distance_residuals: list[float] = []
  nis_lateral_residuals: list[float] = []
  nis_velocity_residuals: list[float] = []
  last_nis: tuple[float, RadarObject] | None = None

  for path in paths:
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      which = msg.which()
      t = float(msg.logMonoTime) / 1e9
      if which == "carState":
        v_ego = float(msg.carState.vEgo)
      elif which == "can":
        for can in msg.can:
          addr = int(can.address)
          dat = bytes(can.dat)
          if int(can.src) == 1 and 0x180 <= addr <= 0x184 and len(dat) == 32:
            for obj in decode(t, addr, dat):
              latest[(addr, obj.slot)] = obj
      elif which == "modelV2" and msg.modelV2.leadsV3:
        lead = msg.modelV2.leadsV3[0]
        if float(lead.prob) < MIN_LEAD_PROB or not lead.x or not lead.y or not lead.v:
          continue
        lead_frames += 1
        model_d = float(lead.x[0]) - RADAR_TO_CAMERA
        model_y = float(lead.y[0])
        model_v_rel = float(lead.v[0]) - v_ego
        candidates = [obj for obj in latest.values() if valid(obj, t)]
        if not candidates:
          continue

        scored_candidates = []
        for obj in candidates:
          position_sane = (
            abs(obj.distance - model_d) < max(abs(model_d) * 0.22, 4.0)
            and abs(obj.lateral + model_y) < max(1.2, 1.5 * max(float(lead.yStd[0]), 0.2))
            and abs(obj.velocity - model_v_rel) < max(4.0, 3.0 * max(float(lead.vStd[0]), 0.5))
          )
          if not position_sane:
            continue
          score = normalized_innovation(
            obj, model_d, model_y, model_v_rel,
            float(lead.xStd[0]), float(lead.yStd[0]), float(lead.vStd[0]),
          )
          preferred = last_nis is not None and obj.track_id == last_nis[1].track_id
          gate = PREFERRED_NIS_GATE if preferred else NIS_GATE
          if math.isfinite(score) and score <= gate and obj.velocity + v_ego > -2.0:
            scored_candidates.append((score, obj))
        if scored_candidates:
          score, nis = min(scored_candidates, key=lambda candidate: candidate[0])
          preferred = next((candidate for candidate in scored_candidates
                            if last_nis is not None and candidate[1].track_id == last_nis[1].track_id), None)
          if preferred is not None and preferred[0] <= score + SWITCH_SCORE_MARGIN:
            score, nis = preferred
          nis_matches += 1
          nis_scores.append(score)
          nis_distance_residuals.append(abs(nis.distance - model_d))
          nis_lateral_residuals.append(abs(nis.lateral + model_y))
          nis_velocity_residuals.append(abs(nis.velocity - model_v_rel))
          if last_nis is not None and 0.0 < t - last_nis[0] <= 0.12:
            nis_continuity += 1
            if nis.track_id != last_nis[1].track_id:
              nis_handoffs += 1
          last_nis = (t, nis)

        runtime_candidates = [obj for obj in candidates if (
          abs(obj.distance - model_d) < max(abs(model_d) * 0.22, 4.0) and
          abs(obj.lateral + model_y) < max(1.2, 1.5 * max(float(lead.yStd[0]), 0.2)) and
          abs(obj.velocity - model_v_rel) < max(4.0, 3.0 * max(float(lead.vStd[0]), 0.5))
        )]
        if runtime_candidates:
          runtime = min(runtime_candidates, key=lambda obj: (
            abs(obj.distance - model_d),
            abs(obj.lateral + model_y),
          ))
          runtime_matches += 1
          runtime_distance_corrections.append(abs(runtime.distance - model_d))
          runtime_lateral_residuals.append(abs(runtime.lateral + model_y))
          runtime_velocity_residuals.append(abs(runtime.velocity - model_v_rel))
          if last_runtime is not None and 0.0 < t - last_runtime[0] <= 0.12:
            previous = last_runtime[1]
            if abs(runtime.distance - previous.distance) <= 5.0:
              runtime_continuity += 1
              if runtime.track_id != previous.track_id:
                runtime_handoffs += 1
          last_runtime = (t, runtime)

        primary_for_gate = latest.get((0x180, 1))
        if primary_for_gate is not None and valid(primary_for_gate, t):
          if (abs(primary_for_gate.distance - model_d) < max(abs(model_d) * 0.22, 4.0) and
              abs(primary_for_gate.lateral + model_y) < max(1.2, 1.5 * max(float(lead.yStd[0]), 0.2)) and
              abs(primary_for_gate.velocity - model_v_rel) < max(4.0, 3.0 * max(float(lead.vStd[0]), 0.5))):
            primary_runtime_matches += 1

        x_std = max(float(lead.xStd[0]), 2.0)
        v_std = max(float(lead.vStd[0]), 1.0)
        selected = min(candidates, key=lambda obj: (
          abs(obj.distance - model_d) / x_std + abs(obj.velocity - model_v_rel) / v_std,
          abs(obj.distance - model_d),
        ))
        distance_error = abs(selected.distance - model_d)
        velocity_error = abs(selected.velocity - model_v_rel)
        if (distance_error > max(MAX_DISTANCE_ERROR, abs(model_d) * 0.25) or
            velocity_error > max(8.0, 3.0 * v_std)):
          continue

        matched_frames += 1
        distance_errors.append(distance_error)
        lateral_errors.append(abs(selected.lateral + model_y))
        velocity_errors.append(velocity_error)
        if distance_error <= max(3.0, abs(model_d) * 0.10) and velocity_error <= max(3.0, 2.0 * v_std):
          strict_lateral_errors.append(abs(selected.lateral + model_y))
          strict_velocity_errors.append(velocity_error)
        selected_channels[f"0x{selected.addr:x}.{selected.slot}"] += 1
        selected_states[f"{selected.state}/{selected.state_alt}"] += 1
        selected_track_ids[selected.track_id] += 1

        primary = latest.get((0x180, 1))
        if primary is not None and valid(primary, t):
          primary_distance_errors.append(abs(primary.distance - model_d))
          primary_lateral_errors.append(abs(primary.lateral + model_y))

        if last_selected is not None and 0.0 < selected.t - last_selected.t <= MAX_RADAR_AGE:
          if abs(selected.distance - last_selected.distance) <= 5.0:
            continuity_frames += 1
            if (selected.addr, selected.slot) == (last_selected.addr, last_selected.slot):
              slot_continuity += 1
            if selected.track_id == last_selected.track_id:
              track_id_continuity += 1
        last_selected = selected

  runtime_lateral_p95 = percentile(runtime_lateral_residuals, 95)
  runtime_velocity_p95 = percentile(runtime_velocity_residuals, 95)
  runtime_handoff_rate = runtime_handoffs / max(runtime_continuity, 1)
  runtime_integration_ready = (
    runtime_matches >= 1000 and
    runtime_lateral_p95 is not None and runtime_lateral_p95 <= 1.2 and
    runtime_velocity_p95 is not None and runtime_velocity_p95 <= 4.0 and
    runtime_handoff_rate <= 0.01
  )
  return {
    "files": len(paths),
    "leadFrames": lead_frames,
    "matchedFrames": matched_frames,
    "matchCoverage": round(matched_frames / max(lead_frames, 1), 4),
    "distanceError": stats(distance_errors),
    "lateralErrorIndependentSelection": stats(lateral_errors),
    "velocityErrorIndependentSelection": stats(velocity_errors),
    "strictAssociationLateralError": stats(strict_lateral_errors),
    "strictAssociationVelocityError": stats(strict_velocity_errors),
    "primaryOnlyDistanceError": stats(primary_distance_errors),
    "primaryOnlyLateralError": stats(primary_lateral_errors),
    "runtimeGate": {
      "matches": runtime_matches,
      "coverage": round(runtime_matches / max(lead_frames, 1), 4),
      "primaryOnlyMatches": primary_runtime_matches,
      "primaryOnlyCoverage": round(primary_runtime_matches / max(lead_frames, 1), 4),
      "distanceCorrection": stats(runtime_distance_corrections),
      "lateralResidual": stats(runtime_lateral_residuals),
      "velocityResidual": stats(runtime_velocity_residuals),
      "continuousFrames": runtime_continuity,
      "trackIdHandoffs": runtime_handoffs,
      "trackIdHandoffRate": round(runtime_handoff_rate, 4),
      "integrationReady": runtime_integration_ready,
    },
    "normalizedInnovationGate": {
      "matches": nis_matches,
      "coverage": round(nis_matches / max(lead_frames, 1), 4),
      "score": stats(nis_scores),
      "distanceResidual": stats(nis_distance_residuals),
      "lateralResidual": stats(nis_lateral_residuals),
      "velocityResidual": stats(nis_velocity_residuals),
      "continuousFrames": nis_continuity,
      "trackIdHandoffs": nis_handoffs,
      "trackIdHandoffRate": round(nis_handoffs / max(nis_continuity, 1), 4),
    },
    "selectedChannels": dict(selected_channels.most_common()),
    "selectedStates": dict(selected_states.most_common()),
    "selectedTrackIds": len(selected_track_ids),
    "continuity": {
      "frames": continuity_frames,
      "sameSlotRate": round(slot_continuity / max(continuity_frames, 1), 4),
      "samePublicTrackIdRate": round(track_id_continuity / max(continuity_frames, 1), 4),
    },
    "lateralFieldReady": len(strict_lateral_errors) >= 500 and (percentile(strict_lateral_errors, 95) or 999.0) <= 1.5,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("log_root", type=Path)
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()

  results = {}
  for route, paths in sorted(iter_rlogs(args.log_root).items()):
    print(f"Analyzing {route} ({len(paths)} segments)", flush=True)
    results[route] = analyze_route(paths)

  ready = [route for route, result in results.items() if result["lateralFieldReady"]]
  runtime_ready = [route for route, result in results.items() if result["runtimeGate"]["integrationReady"]]
  report = {
    "selection": "fresh track-id/heartbeat-valid slot selected by distance and independently verified velocity; lateral excluded from matching",
    "thresholds": {
      "minimumLeadProbability": MIN_LEAD_PROB,
      "maximumRadarAgeSeconds": MAX_RADAR_AGE,
      "maximumDistanceErrorMeters": MAX_DISTANCE_ERROR,
      "lateralP95ReadyMeters": 1.5,
    },
    "readyRoutes": ready,
    "runtimeIntegrationReadyRoutes": runtime_ready,
    "routes": results,
    "status": "pass" if runtime_ready and len(runtime_ready) == len(results) else "fail",
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.write_text(output + "\n")
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
