#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import glob
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

from opendbc.car.hyundai.radar_interface import (
  CARNIVAL_4TH_GEN_CONFIRMATION_MAX_GAP,
  CARNIVAL_4TH_GEN_CONFIRMATION_MIN_PERSIST,
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_BASE,
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE,
  CARNIVAL_4TH_GEN_OBJECT_BUS,
  CARNIVAL_4TH_GEN_OBJECT_END_ADDR,
  CARNIVAL_4TH_GEN_OBJECT_LEN,
  CARNIVAL_4TH_GEN_OBJECT_START_ADDR,
  CarnivalRadarObject,
  carnival_radar_object_valid,
  decode_carnival_radar_object,
)
from openpilot.tools.lib.logreader import LogReader, ReadMode


CARNIVAL_TRACK_ID_MIN = 0xC4100
CARNIVAL_TRACK_ID_MAX = 0xC41FF
RADAR_TO_CAMERA = 1.52
REACQUIRE_MIN_FRAMES = 8
REACQUIRE_MAX_V_EGO = 4.0
REACQUIRE_MAX_DISTANCE = 25.0
REACQUIRE_PATH_OFFSET = 1.1
RADAR_ONLY_MIN_FRAMES = 20
RADAR_ONLY_MAX_V_EGO = 3.0
RADAR_ONLY_MAX_DISTANCE = 18.0
RADAR_ONLY_PATH_OFFSET = 0.65
RADAR_ONLY_MAX_ABS_Y = 1.25
RADAR_ONLY_PATH_MIN_FRAMES = 8
RADAR_ONLY_MIN_V_LEAD = -1.5
RADAR_ONLY_MAX_V_LEAD = 6.0
RADAR_ONLY_DISTANCE_RATE_FRAMES = 8
RADAR_ONLY_DISTANCE_RATE_MAX_RESIDUAL = 1.5
MODEL_LEAD_PROBABILITY = 0.35
MODEL_MATCH_DISTANCE_SCALE = 0.22
MODEL_MATCH_DISTANCE_FLOOR = 4.0
MODEL_MATCH_Y_STD_SCALE = 1.5
MODEL_MATCH_Y_FLOOR = 1.2
MODEL_MATCH_V_STD_SCALE = 3.0
MODEL_MATCH_V_FLOOR = 4.0


def safe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def safe_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except Exception:
    return default
  return result if math.isfinite(result) else default


def safe_int(value: Any, default: int = -1) -> int:
  try:
    return int(value)
  except Exception:
    return default


def is_carnival_track(track_id: int) -> bool:
  return CARNIVAL_TRACK_ID_MIN <= track_id <= CARNIVAL_TRACK_ID_MAX


def expand_paths(patterns: list[str]) -> list[Path]:
  paths = []
  for pattern in patterns:
    matches = [Path(path) for path in glob.glob(pattern)]
    paths.extend(matches if matches else [Path(pattern)])
  return sorted({path.resolve() for path in paths if path.is_file()}, key=lambda path: (route_name(path), segment_number(path)))


def route_name(path: Path) -> str:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 2:
      return "--".join(parts[:2])
  return path.parent.name


def segment_number(path: Path) -> int:
  for name in (path.parent.name, path.name):
    parts = name.split("--")
    if len(parts) >= 3:
      try:
        return int(parts[2].split(".", 1)[0])
      except ValueError:
        pass
  return -1


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
  return round(ordered[index], 4)


def summary(values: list[float]) -> dict[str, float | int | None]:
  return {
    "count": len(values),
    "p50": percentile(values, 50.0),
    "p90": percentile(values, 90.0),
    "p95": percentile(values, 95.0),
    "p99": percentile(values, 99.0),
    "max": round(max(values), 4) if values else None,
  }


def interpolate_path_y(path_x: list[float], path_y: list[float], object_x: float) -> float | None:
  if len(path_x) < 2 or len(path_x) != len(path_y) or object_x < path_x[0] or object_x > path_x[-1]:
    return None
  upper = bisect.bisect_right(path_x, object_x)
  if upper <= 0:
    return path_y[0]
  if upper >= len(path_x):
    return path_y[-1]
  lower = upper - 1
  span = path_x[upper] - path_x[lower]
  if span <= 1e-6:
    return path_y[lower]
  alpha = (object_x - path_x[lower]) / span
  return path_y[lower] + alpha * (path_y[upper] - path_y[lower])


def track_matches_model(track: dict[str, float | int], model_lead: dict[str, float], v_ego: float) -> bool:
  model_distance = model_lead["x"] - RADAR_TO_CAMERA
  return (abs(float(track["dRel"]) - model_distance) < max(abs(model_distance) * MODEL_MATCH_DISTANCE_SCALE,
                                                            MODEL_MATCH_DISTANCE_FLOOR) and
          abs(float(track["yRel"]) + model_lead["y"]) < max(MODEL_MATCH_Y_FLOOR,
                                                               MODEL_MATCH_Y_STD_SCALE * max(model_lead["yStd"], 0.2)) and
          abs(float(track["vRel"]) + v_ego - model_lead["v"]) < max(MODEL_MATCH_V_FLOOR,
                                                                      MODEL_MATCH_V_STD_SCALE * max(model_lead["vStd"], 0.5)))


def qualify(track_id: int, track: dict[str, float | int], v_ego: float, path_x: list[float], path_y: list[float],
            previously_selected: bool, path_frames: dict[int, int]) -> tuple[str, float] | None:
  d_rel = float(track["dRel"])
  path_at_object = interpolate_path_y(path_x, path_y, d_rel + RADAR_TO_CAMERA)
  if path_at_object is None:
    path_frames[track_id] = 0
    return None
  path_offset = abs(-float(track["yRel"]) - path_at_object)
  age = int(track["age"])
  observed_v_rel = track.get("observedVRel")
  distance_rate_sane = (observed_v_rel is not None and
                        abs(float(observed_v_rel) - float(track["vRel"])) <= RADAR_ONLY_DISTANCE_RATE_MAX_RESIDUAL)

  if previously_selected:
    path_frames[track_id] = 0
    if (age >= REACQUIRE_MIN_FRAMES and v_ego < REACQUIRE_MAX_V_EGO and
        0.75 < d_rel < REACQUIRE_MAX_DISTANCE and path_offset < REACQUIRE_PATH_OFFSET and
        distance_rate_sane):
      return "reacquire", path_offset
    return None

  v_lead = v_ego + float(track["vRel"])
  path_aligned = (age >= RADAR_ONLY_MIN_FRAMES and v_ego < RADAR_ONLY_MAX_V_EGO and
                  0.75 < d_rel < RADAR_ONLY_MAX_DISTANCE and abs(float(track["yRel"])) < RADAR_ONLY_MAX_ABS_Y and
                  path_offset < RADAR_ONLY_PATH_OFFSET and RADAR_ONLY_MIN_V_LEAD < v_lead < RADAR_ONLY_MAX_V_LEAD)
  path_frames[track_id] = path_frames.get(track_id, 0) + 1 if path_aligned else 0
  if path_frames[track_id] >= RADAR_ONLY_PATH_MIN_FRAMES and distance_rate_sane:
    return "new", path_offset
  return None


def live_track_continuous(previous: dict[str, float | int] | None, mono_time: int,
                          d_rel: float, y_rel: float, v_rel: float) -> bool:
  if previous is None:
    return False
  dt = (mono_time - int(previous["monoTime"])) / 1e9
  return (0.0 <= dt <= 0.2 and
          abs(d_rel - float(previous["dRel"])) <= max(1.5, 60.0 * dt) and
          abs(y_rel - float(previous["yRel"])) <= max(1.0, 20.0 * dt) and
          abs(v_rel - float(previous["vRel"])) <= 8.0)


def raw_object_continuous(previous: tuple[int, float, float, float] | None, mono_time: int,
                          obj: CarnivalRadarObject) -> bool:
  if previous is None:
    return False
  previous_time, previous_distance, previous_lateral, previous_velocity = previous
  dt = (mono_time - previous_time) / 1e9
  return (0.0 <= dt <= CARNIVAL_4TH_GEN_CONFIRMATION_MAX_GAP and
          abs(obj.d_rel - previous_distance) <= max(1.5, 60.0 * dt) and
          abs(obj.y_rel - previous_lateral) <= max(1.0, 20.0 * dt) and
          abs(obj.v_rel - previous_velocity) <= 8.0)


def causal_confirmation_age(times: list[int], mono_time: int) -> float | None:
  if not times:
    return None
  index = bisect.bisect_left(times, mono_time)
  return (times[index] - mono_time) / 1e9 if index < len(times) else None


def scan(paths: list[Path]) -> dict[str, Any]:
  latest_v_ego = 0.0
  latest_path_x: list[float] = []
  latest_path_y: list[float] = []
  latest_model_prob = 0.0
  latest_model_lead: dict[str, float] = {}
  latest_tracks: dict[int, dict[str, float | int]] = {}
  track_generations: dict[int, int] = {}
  radar_only_path_frames: dict[int, int] = {}
  distance_histories: dict[tuple[int, int], list[float]] = {}
  raw_previous: dict[int, tuple[int, float, float, float]] = {}
  raw_persist: dict[int, int] = {}
  raw_published: dict[int, tuple[int, float, float, float]] = {}
  raw_can_seen = False
  raw_can_routes: set[str] = set()
  simulated_previous: tuple[int, int] | None = None
  current_route = ""
  samples: list[dict[str, Any]] = []
  confirmations: dict[tuple[str, int, int], list[int]] = defaultdict(list)
  simulated_switches = 0
  identity_resets = 0

  for path in paths:
    route = route_name(path)
    if current_route and route != current_route:
      latest_tracks = {}
      track_generations = {}
      radar_only_path_frames = {}
      distance_histories = {}
      raw_previous = {}
      raw_persist = {}
      raw_published = {}
      raw_can_seen = False
      simulated_previous = None
    current_route = route

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      which = msg.which()
      mono_time = int(msg.logMonoTime)
      if which == "carState":
        latest_v_ego = safe_float(safe_attr(msg.carState, "vEgo", 0.0))
      elif which == "modelV2":
        model = msg.modelV2
        position = safe_attr(model, "position")
        latest_path_x = [safe_float(value) for value in safe_attr(position, "x", [])]
        latest_path_y = [safe_float(value) for value in safe_attr(position, "y", [])]
        leads = list(safe_attr(model, "leadsV3", []))
        latest_model_prob = safe_float(safe_attr(leads[0], "prob", 0.0)) if leads else 0.0
        latest_model_lead = {}
        if leads:
          lead = leads[0]
          x = list(safe_attr(lead, "x", []))
          y = list(safe_attr(lead, "y", []))
          v = list(safe_attr(lead, "v", []))
          x_std = list(safe_attr(lead, "xStd", []))
          y_std = list(safe_attr(lead, "yStd", []))
          v_std = list(safe_attr(lead, "vStd", []))
          if x and y and v:
            latest_model_lead = {
              "x": safe_float(x[0]),
              "y": safe_float(y[0]),
              "v": safe_float(v[0]),
              "xStd": safe_float(x_std[0], 2.0) if x_std else 2.0,
              "yStd": safe_float(y_std[0], 0.2) if y_std else 0.2,
              "vStd": safe_float(v_std[0], 0.5) if v_std else 0.5,
            }

        if latest_model_prob > MODEL_LEAD_PROBABILITY and latest_model_lead:
          if raw_can_seen:
            max_age_ns = int(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE * 1e9)
            for raw_track_id, (track_time, d_rel, y_rel, v_rel) in raw_published.items():
              if mono_time - track_time > max_age_ns:
                continue
              track_id = CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_BASE + raw_track_id
              track = {"dRel": d_rel, "yRel": y_rel, "vRel": v_rel}
              if track_matches_model(track, latest_model_lead, latest_v_ego):
                confirmations[(route, track_id, track_generations[raw_track_id])].append(mono_time)
          else:
            for track_id, track in latest_tracks.items():
              if track_matches_model(track, latest_model_lead, latest_v_ego):
                confirmations[(route, track_id, int(track["generation"]))].append(mono_time)
      elif which == "can":
        batch_objects: dict[int, CarnivalRadarObject] = {}
        conflicting_ids: set[int] = set()
        for can in safe_attr(msg, "can", []):
          address = safe_int(safe_attr(can, "address", -1))
          dat = bytes(safe_attr(can, "dat", b""))
          if (safe_int(safe_attr(can, "src", -1)) != CARNIVAL_4TH_GEN_OBJECT_BUS or
              not CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR or
              len(dat) != CARNIVAL_4TH_GEN_OBJECT_LEN):
            continue
          raw_can_seen = True
          raw_can_routes.add(route)
          for bit_offset in (0, 128):
            obj = decode_carnival_radar_object(dat, bit_offset)
            if not carnival_radar_object_valid(obj):
              continue
            previous_in_batch = batch_objects.get(obj.raw_track_id)
            if previous_in_batch is not None and previous_in_batch != obj:
              conflicting_ids.add(obj.raw_track_id)
            else:
              batch_objects[obj.raw_track_id] = obj

        for raw_track_id in conflicting_ids:
          batch_objects.pop(raw_track_id, None)
          raw_published.pop(raw_track_id, None)
          raw_previous.pop(raw_track_id, None)
          raw_persist.pop(raw_track_id, None)
          track_generations[raw_track_id] = track_generations.get(raw_track_id, 0) + 1
          identity_resets += 1

        for raw_track_id, obj in batch_objects.items():
          previous = raw_previous.get(raw_track_id)
          continuous = raw_object_continuous(previous, mono_time, obj)
          if not continuous:
            if previous is not None:
              identity_resets += 1
            raw_published.pop(raw_track_id, None)
            track_generations[raw_track_id] = track_generations.get(raw_track_id, 0) + 1
          raw_persist[raw_track_id] = raw_persist.get(raw_track_id, 0) + 1 if continuous else 1
          raw_previous[raw_track_id] = (mono_time, obj.d_rel, obj.y_rel, obj.v_rel)
          if raw_persist[raw_track_id] >= CARNIVAL_4TH_GEN_CONFIRMATION_MIN_PERSIST:
            raw_published[raw_track_id] = (mono_time, obj.d_rel, obj.y_rel, obj.v_rel)

        max_age_ns = int(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE * 1e9)
        for raw_track_id, (track_time, *_) in list(raw_published.items()):
          if mono_time - track_time > max_age_ns:
            raw_published.pop(raw_track_id, None)
            raw_previous.pop(raw_track_id, None)
            raw_persist.pop(raw_track_id, None)
      elif which == "liveTracks" and not raw_can_seen:
        previous_tracks = latest_tracks
        current_ids = set()
        latest_tracks = {}
        for point in safe_attr(msg.liveTracks, "points", []):
          track_id = safe_int(safe_attr(point, "trackId", -1))
          if not is_carnival_track(track_id):
            continue
          current_ids.add(track_id)
          d_rel = safe_float(safe_attr(point, "dRel", 0.0))
          y_rel = safe_float(safe_attr(point, "yRel", 0.0))
          v_rel = safe_float(safe_attr(point, "vRel", 0.0))
          previous = previous_tracks.get(track_id)
          continuous = live_track_continuous(previous, mono_time, d_rel, y_rel, v_rel)
          if not continuous:
            if previous is not None:
              identity_resets += 1
            track_generations[track_id] = track_generations.get(track_id, 0) + 1
            radar_only_path_frames[track_id] = 0
          latest_tracks[track_id] = {
            "dRel": d_rel,
            "yRel": y_rel,
            "vRel": v_rel,
            "age": int(previous["age"]) + 1 if continuous else 1,
            "generation": track_generations[track_id],
            "monoTime": mono_time,
          }
        radar_only_path_frames = {
          track_id: radar_only_path_frames.get(track_id, 0)
          for track_id in current_ids
        }
      elif which == "radarState":
        if raw_can_seen:
          max_age_ns = int(CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE * 1e9)
          for raw_track_id, (track_time, *_) in list(raw_published.items()):
            if mono_time - track_time > max_age_ns:
              raw_published.pop(raw_track_id, None)
              raw_previous.pop(raw_track_id, None)
              raw_persist.pop(raw_track_id, None)

          previous_tracks = latest_tracks
          latest_tracks = {}
          current_ids = set()
          for raw_track_id, (track_time, d_rel, y_rel, v_rel) in raw_published.items():
            track_id = CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_BASE + raw_track_id
            current_ids.add(track_id)
            generation = track_generations[raw_track_id]
            previous = previous_tracks.get(track_id)
            same_generation = previous is not None and int(previous["generation"]) == generation
            if not same_generation:
              radar_only_path_frames[track_id] = 0
            latest_tracks[track_id] = {
              "dRel": d_rel,
              "yRel": y_rel,
              "vRel": v_rel,
              "age": int(previous["age"]) + 1 if same_generation else 1,
              "generation": generation,
              "monoTime": track_time,
            }
          radar_only_path_frames = {
            track_id: radar_only_path_frames.get(track_id, 0)
            for track_id in current_ids
          }

        lead = safe_attr(msg.radarState, "leadOne")
        old_status = bool(safe_attr(lead, "status", False))
        old_track_id = safe_int(safe_attr(lead, "radarTrackId", -1))
        current_generations = set()
        for track_id, track in latest_tracks.items():
          generation = int(track["generation"])
          key = (track_id, generation)
          current_generations.add(key)
          history = distance_histories.setdefault(key, [])
          history.append(float(track["dRel"]))
          del history[:-RADAR_ONLY_DISTANCE_RATE_FRAMES]
          if len(history) >= RADAR_ONLY_DISTANCE_RATE_FRAMES:
            track["observedVRel"] = ((history[-1] - history[0]) /
                                     ((len(history) - 1) * 0.05))
        distance_histories = {key: value for key, value in distance_histories.items() if key in current_generations}
        model_matches = []
        if latest_model_prob > MODEL_LEAD_PROBABILITY and latest_model_lead:
          model_matches = [
            (track_id, track)
            for track_id, track in latest_tracks.items()
            if track_matches_model(track, latest_model_lead, latest_v_ego)
          ]
        model_track = None
        if model_matches:
          model_distance = latest_model_lead["x"] - RADAR_TO_CAMERA
          model_track = min(model_matches, key=lambda item: (
            abs(float(item[1]["dRel"]) - model_distance),
            abs(float(item[1]["yRel"]) + latest_model_lead["y"]),
          ))
        baseline_status = latest_model_prob > MODEL_LEAD_PROBABILITY and bool(latest_model_lead)
        baseline_distance = (float(model_track[1]["dRel"]) if model_track is not None
                             else latest_model_lead.get("x", 999.0) - RADAR_TO_CAMERA)
        baseline_track = ((model_track[0], int(model_track[1]["generation"]))
                          if model_track is not None else None)

        candidates = []
        for track_id, track in latest_tracks.items():
          generation = int(track["generation"])
          result = qualify(track_id, track, latest_v_ego, latest_path_x, latest_path_y,
                           (track_id, generation) == simulated_previous, radar_only_path_frames)
          if result is not None:
            tier, path_offset = result
            candidates.append((float(track["dRel"]), track_id, tier, path_offset, track))
        if not candidates:
          simulated_previous = baseline_track
          continue

        _, track_id, tier, path_offset, track = min(candidates)
        generation = int(track["generation"])
        selected = (track_id, generation)
        would_override = not baseline_status or float(track["dRel"]) < baseline_distance
        if would_override:
          if simulated_previous is not None and simulated_previous != selected:
            simulated_switches += 1
          simulated_previous = selected
        else:
          simulated_previous = baseline_track

        samples.append({
          "route": route,
          "monoTime": mono_time,
          "trackId": track_id,
          "generation": generation,
          "tier": tier,
          "wouldOverride": would_override,
          "oldLeadStatus": old_status,
          "oldLeadTrackId": old_track_id,
          "modelProb": round(latest_model_prob, 4),
          "vEgo": round(latest_v_ego, 4),
          "dRel": round(float(track["dRel"]), 4),
          "yRel": round(float(track["yRel"]), 4),
          "vRel": round(float(track["vRel"]), 4),
          "observedVRel": None if track.get("observedVRel") is None else round(float(track["observedVRel"]), 4),
          "vLead": round(latest_v_ego + float(track["vRel"]), 4),
          "age": int(track["age"]),
          "pathOffset": round(path_offset, 4),
        })

  for sample in samples:
    age = causal_confirmation_age(
      confirmations[(sample["route"], sample["trackId"], sample["generation"])], sample["monoTime"])
    sample["causalVisionConfirmationSeconds"] = None if age is None else round(age, 3)

  events = []
  for sample in samples:
    if not sample["wouldOverride"]:
      continue
    if (events and events[-1]["route"] == sample["route"] and events[-1]["trackId"] == sample["trackId"] and
        events[-1]["generation"] == sample["generation"] and
        events[-1]["tier"] == sample["tier"] and sample["monoTime"] - events[-1]["endMonoTime"] <= 150_000_000):
      event = events[-1]
      event["endMonoTime"] = sample["monoTime"]
      event["frames"] += 1
      event["minimumDistance"] = min(event["minimumDistance"], sample["dRel"])
      event["maximumPathOffset"] = max(event["maximumPathOffset"], sample["pathOffset"])
      age = sample["causalVisionConfirmationSeconds"]
      if age is not None:
        event["causalVisionConfirmationSeconds"] = min(event["causalVisionConfirmationSeconds"], age)
    else:
      age = sample["causalVisionConfirmationSeconds"]
      events.append({
        "route": sample["route"],
        "trackId": sample["trackId"],
        "generation": sample["generation"],
        "tier": sample["tier"],
        "startMonoTime": sample["monoTime"],
        "endMonoTime": sample["monoTime"],
        "frames": 1,
        "startModelProb": sample["modelProb"],
        "startVEgo": sample["vEgo"],
        "startVLead": sample["vLead"],
        "startVRel": sample["vRel"],
        "startObservedVRel": sample["observedVRel"],
        "startYRel": sample["yRel"],
        "oldLeadStatus": sample["oldLeadStatus"],
        "oldLeadTrackId": sample["oldLeadTrackId"],
        "minimumDistance": sample["dRel"],
        "maximumPathOffset": sample["pathOffset"],
        "causalVisionConfirmationSeconds": age if age is not None else 999.0,
      })

  for event in events:
    event["durationSeconds"] = round((event["endMonoTime"] - event["startMonoTime"]) / 1e9, 3)
    if event["causalVisionConfirmationSeconds"] == 999.0:
      event["causalVisionConfirmationSeconds"] = None

  overrides = [sample for sample in samples if sample["wouldOverride"]]
  new_overrides = [sample for sample in overrides if sample["tier"] == "new"]
  reacquires = [sample for sample in overrides if sample["tier"] == "reacquire"]
  confirmed_within_two = [sample for sample in new_overrides
                          if sample["causalVisionConfirmationSeconds"] is not None and
                          sample["causalVisionConfirmationSeconds"] <= 2.0]
  new_events = [event for event in events if event["tier"] == "new"]
  supported_new_events = [event for event in new_events
                          if event["causalVisionConfirmationSeconds"] is not None and
                          event["causalVisionConfirmationSeconds"] <= 2.0]

  return {
    "filesScanned": len(paths),
    "routes": sorted({route_name(path) for path in paths}),
    "rawCanRoutes": sorted(raw_can_routes),
    "qualifiedFrames": len(samples),
    "overrideFrames": len(overrides),
    "newRadarOnlyFrames": len(new_overrides),
    "reacquireFrames": len(reacquires),
    "identityResets": identity_resets,
    "simulatedTrackSwitches": simulated_switches,
    "newRadarOnlyVisionConfirmedWithinTwoSeconds": len(confirmed_within_two),
    "newRadarOnlyVisionConfirmationRate": round(len(confirmed_within_two) / max(len(new_overrides), 1), 4),
    "eventCount": len(events),
    "newRadarOnlyEventCount": len(new_events),
    "newRadarOnlyEventsConfirmedWithinTwoSeconds": len(supported_new_events),
    "newRadarOnlyEventConfirmationRate": round(len(supported_new_events) / max(len(new_events), 1), 4),
    "overrideDistance": summary([sample["dRel"] for sample in overrides]),
    "overridePathOffset": summary([sample["pathOffset"] for sample in overrides]),
    "overrideLeadSpeed": summary([sample["vLead"] for sample in overrides]),
    "events": events[:100],
    "status": "pass" if len(supported_new_events) == len(new_events) and simulated_switches == 0 else "review",
  }


def main() -> int:
  parser = argparse.ArgumentParser(description="Replay Carnival low-speed radar-only lead qualification.")
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out")
  args = parser.parse_args()
  report = scan(expand_paths(args.logs))
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
