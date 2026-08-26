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


def qualify(track_id: int, track: dict[str, float | int], v_ego: float, path_x: list[float], path_y: list[float],
            previously_selected: bool, path_frames: dict[int, int]) -> tuple[str, float] | None:
  d_rel = float(track["dRel"])
  path_at_object = interpolate_path_y(path_x, path_y, d_rel + RADAR_TO_CAMERA)
  if path_at_object is None:
    path_frames[track_id] = 0
    return None
  path_offset = abs(-float(track["yRel"]) - path_at_object)
  age = int(track["age"])

  if previously_selected:
    path_frames[track_id] = 0
    if (age >= REACQUIRE_MIN_FRAMES and v_ego < REACQUIRE_MAX_V_EGO and
        0.75 < d_rel < REACQUIRE_MAX_DISTANCE and path_offset < REACQUIRE_PATH_OFFSET):
      return "reacquire", path_offset
    return None

  v_lead = v_ego + float(track["vRel"])
  path_aligned = (age >= RADAR_ONLY_MIN_FRAMES and v_ego < RADAR_ONLY_MAX_V_EGO and
                  0.75 < d_rel < RADAR_ONLY_MAX_DISTANCE and abs(float(track["yRel"])) < RADAR_ONLY_MAX_ABS_Y and
                  path_offset < RADAR_ONLY_PATH_OFFSET and RADAR_ONLY_MIN_V_LEAD < v_lead < RADAR_ONLY_MAX_V_LEAD)
  path_frames[track_id] = path_frames.get(track_id, 0) + 1 if path_aligned else 0
  if path_frames[track_id] >= RADAR_ONLY_PATH_MIN_FRAMES:
    return "new", path_offset
  return None


def nearest_confirmation_age(times: list[int], mono_time: int) -> float | None:
  if not times:
    return None
  index = bisect.bisect_left(times, mono_time)
  candidates = []
  if index < len(times):
    candidates.append(abs(times[index] - mono_time))
  if index > 0:
    candidates.append(abs(times[index - 1] - mono_time))
  return min(candidates) / 1e9 if candidates else None


def scan(paths: list[Path]) -> dict[str, Any]:
  latest_v_ego = 0.0
  latest_path_x: list[float] = []
  latest_path_y: list[float] = []
  latest_model_prob = 0.0
  latest_tracks: dict[int, dict[str, float | int]] = {}
  track_ages: dict[int, int] = {}
  radar_only_path_frames: dict[int, int] = {}
  simulated_previous_id = -1
  current_route = ""
  samples: list[dict[str, Any]] = []
  confirmations: dict[tuple[str, int], list[int]] = defaultdict(list)
  simulated_switches = 0

  for path in paths:
    route = route_name(path)
    if current_route and route != current_route:
      latest_tracks = {}
      track_ages = {}
      radar_only_path_frames = {}
      simulated_previous_id = -1
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
      elif which == "liveTracks":
        current_ids = set()
        latest_tracks = {}
        for point in safe_attr(msg.liveTracks, "points", []):
          track_id = safe_int(safe_attr(point, "trackId", -1))
          if not is_carnival_track(track_id):
            continue
          current_ids.add(track_id)
          latest_tracks[track_id] = {
            "dRel": safe_float(safe_attr(point, "dRel", 0.0)),
            "yRel": safe_float(safe_attr(point, "yRel", 0.0)),
            "vRel": safe_float(safe_attr(point, "vRel", 0.0)),
            "age": track_ages.get(track_id, 0) + 1,
          }
        track_ages = {track_id: track_ages.get(track_id, 0) + 1 for track_id in current_ids}
        radar_only_path_frames = {
          track_id: radar_only_path_frames.get(track_id, 0)
          for track_id in current_ids
        }
      elif which == "radarState":
        lead = safe_attr(msg.radarState, "leadOne")
        old_status = bool(safe_attr(lead, "status", False))
        old_track_id = safe_int(safe_attr(lead, "radarTrackId", -1))
        if old_status and bool(safe_attr(lead, "radar", False)) and is_carnival_track(old_track_id):
          confirmations[(route, old_track_id)].append(mono_time)

        candidates = []
        for track_id, track in latest_tracks.items():
          result = qualify(track_id, track, latest_v_ego, latest_path_x, latest_path_y,
                           track_id == simulated_previous_id, radar_only_path_frames)
          if result is not None:
            tier, path_offset = result
            candidates.append((float(track["dRel"]), track_id, tier, path_offset, track))
        if not candidates:
          simulated_previous_id = old_track_id if old_status and is_carnival_track(old_track_id) else -1
          continue

        _, track_id, tier, path_offset, track = min(candidates)
        old_d_rel = safe_float(safe_attr(lead, "dRel", 999.0))
        would_override = not old_status or float(track["dRel"]) < old_d_rel
        if would_override:
          if simulated_previous_id not in (-1, track_id):
            simulated_switches += 1
          simulated_previous_id = track_id
        else:
          simulated_previous_id = old_track_id if old_status and is_carnival_track(old_track_id) else -1

        samples.append({
          "route": route,
          "monoTime": mono_time,
          "trackId": track_id,
          "tier": tier,
          "wouldOverride": would_override,
          "oldLeadStatus": old_status,
          "oldLeadTrackId": old_track_id,
          "modelProb": round(latest_model_prob, 4),
          "vEgo": round(latest_v_ego, 4),
          "dRel": round(float(track["dRel"]), 4),
          "yRel": round(float(track["yRel"]), 4),
          "vRel": round(float(track["vRel"]), 4),
          "vLead": round(latest_v_ego + float(track["vRel"]), 4),
          "age": int(track["age"]),
          "pathOffset": round(path_offset, 4),
        })

  for sample in samples:
    age = nearest_confirmation_age(confirmations[(sample["route"], sample["trackId"])], sample["monoTime"])
    sample["nearestVisionConfirmationSeconds"] = None if age is None else round(age, 3)

  events = []
  for sample in samples:
    if not sample["wouldOverride"]:
      continue
    if (events and events[-1]["route"] == sample["route"] and events[-1]["trackId"] == sample["trackId"] and
        events[-1]["tier"] == sample["tier"] and sample["monoTime"] - events[-1]["endMonoTime"] <= 150_000_000):
      event = events[-1]
      event["endMonoTime"] = sample["monoTime"]
      event["frames"] += 1
      event["minimumDistance"] = min(event["minimumDistance"], sample["dRel"])
      event["maximumPathOffset"] = max(event["maximumPathOffset"], sample["pathOffset"])
      age = sample["nearestVisionConfirmationSeconds"]
      if age is not None:
        event["nearestVisionConfirmationSeconds"] = min(event["nearestVisionConfirmationSeconds"], age)
    else:
      age = sample["nearestVisionConfirmationSeconds"]
      events.append({
        "route": sample["route"],
        "trackId": sample["trackId"],
        "tier": sample["tier"],
        "startMonoTime": sample["monoTime"],
        "endMonoTime": sample["monoTime"],
        "frames": 1,
        "startModelProb": sample["modelProb"],
        "startVEgo": sample["vEgo"],
        "startVLead": sample["vLead"],
        "oldLeadStatus": sample["oldLeadStatus"],
        "oldLeadTrackId": sample["oldLeadTrackId"],
        "minimumDistance": sample["dRel"],
        "maximumPathOffset": sample["pathOffset"],
        "nearestVisionConfirmationSeconds": age if age is not None else 999.0,
      })

  for event in events:
    event["durationSeconds"] = round((event["endMonoTime"] - event["startMonoTime"]) / 1e9, 3)
    if event["nearestVisionConfirmationSeconds"] == 999.0:
      event["nearestVisionConfirmationSeconds"] = None

  overrides = [sample for sample in samples if sample["wouldOverride"]]
  new_overrides = [sample for sample in overrides if sample["tier"] == "new"]
  reacquires = [sample for sample in overrides if sample["tier"] == "reacquire"]
  confirmed_within_two = [sample for sample in new_overrides
                          if sample["nearestVisionConfirmationSeconds"] is not None and
                          sample["nearestVisionConfirmationSeconds"] <= 2.0]
  new_events = [event for event in events if event["tier"] == "new"]
  supported_new_events = [event for event in new_events
                          if event["nearestVisionConfirmationSeconds"] is not None and
                          event["nearestVisionConfirmationSeconds"] <= 2.0]

  return {
    "filesScanned": len(paths),
    "routes": sorted({route_name(path) for path in paths}),
    "qualifiedFrames": len(samples),
    "overrideFrames": len(overrides),
    "newRadarOnlyFrames": len(new_overrides),
    "reacquireFrames": len(reacquires),
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
    "status": "pass" if reacquires and len(supported_new_events) == len(new_events) and simulated_switches == 0 else "review",
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
