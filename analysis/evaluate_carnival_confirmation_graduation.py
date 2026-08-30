#!/usr/bin/env python3
"""Replay the literal removal of Carnival R0100 confirmationOnly safeguards.

This tool is intentionally offline. It compares the current model-first policy
with the behavior produced when the same R0100 tracks enter radard as ordinary
radar points. It never changes the runtime or vehicle parameters.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.radard import (
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MAX,
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN,
  KalmanParams,
  Track,
  get_lead,
)
from openpilot.tools.lib.logreader import LogReader, ReadMode


DT_RADAR = 0.05
MODEL_LEAD_THRESHOLD = 0.35
FUTURE_MIN_SECONDS = 0.25
FUTURE_MAX_SECONDS = 0.60


def finite(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if math.isfinite(result) else default


def extract(raw: int, start: int, size: int) -> int:
  return (raw >> start) & ((1 << size) - 1)


def is_carnival_track(track_id: int) -> bool:
  return CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MIN <= track_id <= CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_MAX


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


def clone_model(model: Any) -> SimpleNamespace | None:
  leads = list(model.leadsV3)
  if not leads:
    return None

  cloned_leads = []
  for lead in leads[:2]:
    cloned_leads.append(SimpleNamespace(
      prob=finite(lead.prob),
      x=[finite(value) for value in lead.x],
      y=[finite(value) for value in lead.y],
      v=[finite(value) for value in lead.v],
      a=[finite(value) for value in lead.a],
      xStd=[finite(value) for value in lead.xStd],
      yStd=[finite(value) for value in lead.yStd],
      vStd=[finite(value) for value in lead.vStd],
    ))

  position = model.position
  meta = model.meta
  velocity = model.velocity
  return SimpleNamespace(
    leadsV3=cloned_leads,
    position=SimpleNamespace(
      x=[finite(value) for value in position.x],
      y=[finite(value) for value in position.y],
    ),
    meta=SimpleNamespace(
      laneChangeState=int(meta.laneChangeState.raw),
      laneChangeDirection=int(meta.laneChangeDirection.raw),
    ),
    velocity=SimpleNamespace(x=[finite(value) for value in velocity.x]),
  )


def new_track(track_id: int, d_rel: float, y_rel: float, v_rel: float, v_ego: float,
              confirmation_only: bool) -> Track:
  track = Track(track_id, v_ego + v_rel, KalmanParams(DT_RADAR))
  track.confirmationOnly = confirmation_only
  track.update(d_rel, y_rel, v_rel, v_ego + v_rel, True)
  return track


def update_tracks(tracks: dict[int, Track], points: list[tuple[int, float, float, float, bool]],
                  v_ego: float, confirmation_only: bool) -> None:
  seen = set()
  for track_id, d_rel, y_rel, v_rel, measured in points:
    seen.add(track_id)
    track = tracks.get(track_id)
    if track is None:
      tracks[track_id] = new_track(track_id, d_rel, y_rel, v_rel, v_ego, confirmation_only)
    else:
      track.confirmationOnly = confirmation_only
      track.update(d_rel, y_rel, v_rel, v_ego + v_rel, measured)
  for track_id in set(tracks) - seen:
    tracks.pop(track_id, None)


def state_from_policy(tracks: dict[int, Track], lead: Any, model: Any, v_ego: float, standstill: bool,
                      preferred_track_id: int, filtered_lead_prob: float) -> dict[str, Any]:
  model_v_ego = model.velocity.x[0] if model.velocity.x else v_ego
  return get_lead(
    v_ego=v_ego,
    ready=True,
    tracks=tracks,
    lead_msg=lead,
    model_v_ego=model_v_ego,
    model_data=model,
    standstill=standstill,
    starpilot_plan=SimpleNamespace(increasedStoppedDistance=0.0),
    starpilot_toggles=SimpleNamespace(
      lead_detection_probability=MODEL_LEAD_THRESHOLD,
      human_lane_changes=False,
    ),
    low_speed_override=True,
    lead_prob=filtered_lead_prob,
    preferred_track_id=preferred_track_id,
  )


def percentile(values: list[float], value: float) -> float | None:
  return round(float(np.percentile(values, value)), 4) if values else None


def stats(values: list[float]) -> dict[str, Any]:
  return {
    "samples": len(values),
    "p50": percentile(values, 50),
    "p95": percentile(values, 95),
    "p99": percentile(values, 99),
    "max": round(max(values), 4) if values else None,
  }


def future_rate(observations: list[tuple[int, float]], now: int, distance: float) -> float | None:
  future = next(((time, d_rel) for time, d_rel in observations
                 if FUTURE_MIN_SECONDS <= (time - now) / 1e9 <= FUTURE_MAX_SECONDS), None)
  if future is None:
    return None
  return (future[1] - distance) / ((future[0] - now) / 1e9)


def categorical_gate_report(samples: list[dict[str, Any]], key: str, holdout_route: str) -> dict[str, Any]:
  def labeled(rows: list[dict[str, Any]], accepted: bool) -> list[int]:
    return [int(row[key]) for row in rows if key in row and
            ((row["currentTrackId"] == row["trackId"]) if accepted else
             (not is_carnival_track(row["currentTrackId"])))]

  train = [row for row in samples if row["route"] != holdout_route]
  holdout = [row for row in samples if row["route"] == holdout_route]
  train_accepted = labeled(train, True)
  train_extra = labeled(train, False)
  holdout_accepted = labeled(holdout, True)
  holdout_extra = labeled(holdout, False)
  values = sorted(set(train_accepted) | set(train_extra))

  best: tuple[float, int, frozenset[int]] | None = None
  for mask in range(1, 1 << len(values)):
    allowed = frozenset(value for index, value in enumerate(values) if mask & (1 << index))
    recall = sum(value in allowed for value in train_accepted) / max(len(train_accepted), 1)
    if recall < 0.995:
      continue
    rejection = sum(value not in allowed for value in train_extra) / max(len(train_extra), 1)
    candidate = (rejection, -len(allowed), allowed)
    if best is None or candidate[:2] > best[:2]:
      best = candidate

  allowed = best[2] if best is not None else frozenset(values)
  return {
    "field": key,
    "holdoutRoute": holdout_route,
    "allowedValues": sorted(allowed),
    "trainAcceptedSamples": len(train_accepted),
    "trainExtraSamples": len(train_extra),
    "trainAcceptedRecall": round(sum(value in allowed for value in train_accepted) / max(len(train_accepted), 1), 6),
    "trainExtraRejection": round(sum(value not in allowed for value in train_extra) / max(len(train_extra), 1), 6),
    "holdoutAcceptedSamples": len(holdout_accepted),
    "holdoutExtraSamples": len(holdout_extra),
    "holdoutAcceptedRecall": round(sum(value in allowed for value in holdout_accepted) / max(len(holdout_accepted), 1), 6),
    "holdoutExtraRejection": round(sum(value not in allowed for value in holdout_extra) / max(len(holdout_extra), 1), 6),
    "trainAcceptedDistribution": dict(sorted(Counter(train_accepted).items())),
    "trainExtraDistribution": dict(sorted(Counter(train_extra).items())),
    "holdoutAcceptedDistribution": dict(sorted(Counter(holdout_accepted).items())),
    "holdoutExtraDistribution": dict(sorted(Counter(holdout_extra).items())),
  }


def numeric_gate_report(samples: list[dict[str, Any]], key: str, holdout_route: str) -> dict[str, Any]:
  def labeled(rows: list[dict[str, Any]], accepted: bool) -> list[int]:
    return [int(row[key]) for row in rows if key in row and
            ((row["currentTrackId"] == row["trackId"]) if accepted else
             (not is_carnival_track(row["currentTrackId"])))]

  train = [row for row in samples if row["route"] != holdout_route]
  holdout = [row for row in samples if row["route"] == holdout_route]
  train_accepted, train_extra = labeled(train, True), labeled(train, False)
  holdout_accepted, holdout_extra = labeled(holdout, True), labeled(holdout, False)
  candidates = []
  for threshold in sorted(set(train_accepted + train_extra)):
    for direction in ("at_least", "at_most"):
      passes = ((lambda value, bound=threshold: value >= bound) if direction == "at_least"
                else (lambda value, bound=threshold: value <= bound))
      recall = sum(passes(value) for value in train_accepted) / max(len(train_accepted), 1)
      if recall >= 0.995:
        rejection = sum(not passes(value) for value in train_extra) / max(len(train_extra), 1)
        candidates.append((rejection, direction, threshold))
  _, direction, threshold = max(candidates, default=(0.0, "at_least", 0))
  passes = ((lambda value: value >= threshold) if direction == "at_least"
            else (lambda value: value <= threshold))
  return {
    "field": key,
    "direction": direction,
    "threshold": threshold,
    "trainAcceptedSamples": len(train_accepted),
    "trainExtraSamples": len(train_extra),
    "trainAcceptedRecall": round(sum(passes(value) for value in train_accepted) / max(len(train_accepted), 1), 6),
    "trainExtraRejection": round(sum(not passes(value) for value in train_extra) / max(len(train_extra), 1), 6),
    "holdoutAcceptedSamples": len(holdout_accepted),
    "holdoutExtraSamples": len(holdout_extra),
    "holdoutAcceptedRecall": round(sum(passes(value) for value in holdout_accepted) / max(len(holdout_accepted), 1), 6),
    "holdoutExtraRejection": round(sum(not passes(value) for value in holdout_extra) / max(len(holdout_extra), 1), 6),
  }


def scan(paths: list[Path]) -> dict[str, Any]:
  current_tracks: dict[int, Track] = {}
  graduated_tracks: dict[int, Track] = {}
  generations: dict[int, int] = defaultdict(int)
  active_ids: set[int] = set()
  observations: dict[tuple[str, int, int], list[tuple[int, float]]] = defaultdict(list)
  latest_metadata: dict[int, dict[str, int]] = {}
  latest_model = None
  v_ego = 0.0
  standstill = False
  current_preferred = -1
  graduated_preferred = -1
  lead_prob_filter = FirstOrderFilter(0.0, 0.2, DT_RADAR)
  actual_confirmation_samples = 0
  current_replay_matches = 0
  samples = []
  current_route = ""
  graduated_switches = 0
  previous_graduated = -1

  for path in paths:
    route = route_name(path)
    if current_route and route != current_route:
      current_tracks.clear()
      graduated_tracks.clear()
      generations.clear()
      active_ids.clear()
      latest_metadata.clear()
      latest_model = None
      current_preferred = graduated_preferred = previous_graduated = -1
      lead_prob_filter = FirstOrderFilter(0.0, 0.2, DT_RADAR)
    current_route = route

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      which = msg.which()
      now = int(msg.logMonoTime)
      if which == "carState":
        v_ego = finite(msg.carState.vEgo)
        standstill = bool(msg.carState.standstill)
      elif which == "modelV2":
        latest_model = clone_model(msg.modelV2)
      elif which == "can":
        for can in msg.can:
          if int(can.src) != 1 or not (0x180 <= int(can.address) <= 0x184) or len(can.dat) != 32:
            continue
          packed = int.from_bytes(bytes(can.dat), "little", signed=False)
          for offset in (0, 128):
            raw = (packed >> offset) & ((1 << 128) - 1)
            raw_track_id = extract(raw, 42, 8)
            if raw_track_id == 0:
              continue
            metadata = {
              "metadataTime": now,
              "state3": extract(raw, 55, 3),
              "state4": extract(raw, 55, 4),
              "stateAlt": extract(raw, 51, 4),
              "qualityByte": extract(raw, 32, 8),
              "bit40": extract(raw, 40, 1),
              "bit41": extract(raw, 41, 1),
              "metadata50_63": extract(raw, 50, 14),
            }
            metadata.update({f"bit{bit}": extract(raw, bit, 1) for bit in (*range(32, 42), *range(59, 64))})
            latest_metadata[0xC4100 + raw_track_id] = metadata
      elif which == "liveTracks":
        points = []
        next_active_ids = set()
        for point in msg.liveTracks.points:
          track_id = int(point.trackId)
          if not is_carnival_track(track_id):
            continue
          next_active_ids.add(track_id)
          if track_id not in active_ids:
            generations[track_id] += 1
          d_rel = finite(point.dRel)
          y_rel = finite(point.yRel)
          v_rel = finite(point.vRel)
          points.append((track_id, d_rel, y_rel, v_rel, bool(point.measured)))
          observations[(route, track_id, generations[track_id])].append((now, d_rel))
        active_ids = next_active_ids
        update_tracks(current_tracks, points, v_ego, True)
        update_tracks(graduated_tracks, points, v_ego, False)
      elif which == "radarState" and latest_model is not None and latest_model.leadsV3:
        lead = latest_model.leadsV3[0]
        if not lead.x or not lead.y or not lead.v:
          continue
        lead_prob = finite(lead.prob)
        if lead_prob > lead_prob_filter.x:
          lead_prob_filter.x = lead_prob
        else:
          lead_prob_filter.update(lead_prob)
        filtered_lead_prob = float(lead_prob_filter.x)
        current_state = state_from_policy(current_tracks, lead, latest_model, v_ego, standstill,
                                          current_preferred, filtered_lead_prob)
        graduated_state = state_from_policy(graduated_tracks, lead, latest_model, v_ego, standstill,
                                            graduated_preferred, filtered_lead_prob)
        current_preferred = int(current_state.get("radarTrackId", -1))
        graduated_preferred = int(graduated_state.get("radarTrackId", -1))

        actual = msg.radarState.leadOne
        actual_track_id = int(actual.radarTrackId) if actual.status and actual.radar else -1
        if is_carnival_track(actual_track_id):
          actual_confirmation_samples += 1
          current_replay_matches += current_preferred == actual_track_id

        graduated_track_id = graduated_preferred
        if not is_carnival_track(graduated_track_id):
          previous_graduated = -1
          continue
        if previous_graduated != -1 and previous_graduated != graduated_track_id:
          graduated_switches += 1
        previous_graduated = graduated_track_id

        generation = generations.get(graduated_track_id, 0)
        model_v_ego = latest_model.velocity.x[0] if latest_model.velocity.x else v_ego
        model_v_rel = finite(lead.v[0]) - model_v_ego
        sample = {
          "route": route,
          "time": now,
          "trackId": graduated_track_id,
          "generation": generation,
          "dRel": finite(graduated_state.get("dRel")),
          "yRel": finite(graduated_state.get("yRel")),
          "graduatedVRel": finite(graduated_state.get("vRel")),
          "modelVRel": model_v_rel,
          "modelProb": finite(lead.prob),
          "currentTrackId": current_preferred,
          "actualTrackId": actual_track_id,
          "vEgo": v_ego,
        }
        metadata = latest_metadata.get(graduated_track_id)
        if metadata is not None and 0 <= now - metadata["metadataTime"] <= int(0.12e9):
          sample.update({key: value for key, value in metadata.items() if key != "metadataTime"})
        samples.append(sample)

  raw_errors = []
  model_errors = []
  harmful_deltas = []
  future_samples = []
  extra_without_current = []
  different_target = []
  for sample in samples:
    future = future_rate(
      observations[(sample["route"], sample["trackId"], sample["generation"])],
      sample["time"], sample["dRel"],
    )
    if future is not None:
      raw_error = abs(sample["graduatedVRel"] - future)
      model_error = abs(sample["modelVRel"] - future)
      raw_errors.append(raw_error)
      model_errors.append(model_error)
      harmful_deltas.append(raw_error - model_error)
      future_samples.append(sample)
    if not is_carnival_track(sample["currentTrackId"]):
      extra_without_current.append(sample)
    elif sample["currentTrackId"] != sample["trackId"]:
      different_target.append(sample)

  harmful = [delta for delta in harmful_deltas if delta > 0.5]
  replay_fidelity = current_replay_matches / max(actual_confirmation_samples, 1)
  route_sample_counts = Counter(sample["route"] for sample in samples)
  holdout_route = max(route_sample_counts, key=route_sample_counts.get) if route_sample_counts else ""
  metadata_gate_reports = [
    categorical_gate_report(samples, key, holdout_route)
    for key in ("state3", "state4", "stateAlt", *(f"bit{bit}" for bit in (*range(32, 42), *range(59, 64))))
  ] if holdout_route else []
  metadata_numeric_reports = [
    numeric_gate_report(samples, key, holdout_route)
    for key in ("qualityByte",)
  ] if holdout_route else []
  graduation_ready = bool(
    actual_confirmation_samples >= 100 and
    replay_fidelity >= 0.95 and
    len(raw_errors) >= 100 and
    (percentile(raw_errors, 95) or math.inf) < (percentile(model_errors, 95) or 0.0) and
    not harmful and
    not extra_without_current and
    not different_target and
    graduated_switches == 0
  )
  return {
    "qualificationComplete": True,
    "safeToDisableConfirmationOnly": graduation_ready,
    "files": len(paths),
    "routes": sorted({route_name(path) for path in paths}),
    "actualConfirmationSamples": actual_confirmation_samples,
    "currentReplayMatches": current_replay_matches,
    "currentReplayFidelity": round(replay_fidelity, 6),
    "graduatedSelectionSamples": len(samples),
    "graduatedTargetSwitches": graduated_switches,
    "extraSelectionsWithoutCurrentQualification": len(extra_without_current),
    "differentTargetSelections": len(different_target),
    "rawVelocityFutureError": stats(raw_errors),
    "modelVelocityFutureError": stats(model_errors),
    "rawMinusModelError": stats(harmful_deltas),
    "harmfulVelocitySamplesOver0_5": len(harmful),
    "metadataQualification": {
      "holdoutSelection": "complete route with the most counterfactual samples",
      "holdoutRoute": holdout_route,
      "gates": metadata_gate_reports,
      "numericGates": metadata_numeric_reports,
    },
    "harmfulExamples": [
      {
        key: value for key, value in sample.items()
        if key in ("route", "trackId", "dRel", "yRel", "graduatedVRel", "modelVRel", "modelProb", "vEgo")
      }
      for sample, delta in zip(future_samples, harmful_deltas, strict=True)
      if delta > 0.5
    ][:20],
    "graduationReady": graduation_ready,
    "verdict": "disable_confirmation_only" if graduation_ready else "retain_confirmation_only",
    "controlDecision": (
      "R0100 may enter generic radar control"
      if graduation_ready else
      "Keep R0100 model-associated; do not expose raw velocity or generic radar-only selection to control"
    ),
    "failedGates": [
      name for name, passed in (
        ("replay_fidelity", actual_confirmation_samples >= 100 and replay_fidelity >= 0.95),
        ("raw_velocity_future_error", len(raw_errors) >= 100 and
         (percentile(raw_errors, 95) or math.inf) < (percentile(model_errors, 95) or 0.0)),
        ("no_harmful_velocity_samples", not harmful),
        ("no_extra_radar_only_selections", not extra_without_current),
        ("same_target_selection", not different_target),
        ("no_target_switches", graduated_switches == 0),
      ) if not passed
    ],
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
  return 0 if report["graduationReady"] else 2


if __name__ == "__main__":
  raise SystemExit(main())
