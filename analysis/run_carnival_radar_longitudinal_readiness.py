#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
import subprocess
from collections import defaultdict, deque
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode

CARNIVAL_CONFIRMATION_TRACK_ID_MIN = 0xC4100
CARNIVAL_CONFIRMATION_TRACK_ID_MAX = 0xC41FF


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
      except Exception:
        pass
  return -1


def is_confirmation_track(track_id: int) -> bool:
  return CARNIVAL_CONFIRMATION_TRACK_ID_MIN <= int(track_id) <= CARNIVAL_CONFIRMATION_TRACK_ID_MAX


def current_commit() -> str:
  repo_root = Path(__file__).resolve().parents[1]
  try:
    return subprocess.check_output(["git", "-C", str(repo_root), "rev-parse", "HEAD"], text=True).strip()
  except Exception:
    return "unknown"


def expand_log_paths(patterns: list[str]) -> list[Path]:
  paths: list[Path] = []
  for pattern in patterns:
    matches = [Path(path) for path in glob.glob(pattern)]
    paths.extend(matches if matches else [Path(pattern)])
  return sorted({path.resolve() for path in paths if path.is_file()})


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
  return round(ordered[index], 3)


def summarize_values(values: list[float]) -> dict[str, Any]:
  return {
    "count": len(values),
    "p50": percentile(values, 50.0),
    "p90": percentile(values, 90.0),
    "p95": percentile(values, 95.0),
    "p99": percentile(values, 99.0),
    "max": None if not values else round(max(values), 3),
  }


def distance_slope(history: deque[tuple[int, float]]) -> float | None:
  if len(history) < 8:
    return None
  span = (history[-1][0] - history[0][0]) / 1e9
  if span < 0.4:
    return None
  origin = history[0][0]
  xs = [(mono_time - origin) / 1e9 for mono_time, _ in history]
  ys = [distance for _, distance in history]
  x_bar = sum(xs) / len(xs)
  y_bar = sum(ys) / len(ys)
  denominator = sum((x - x_bar) ** 2 for x in xs)
  if denominator <= 1e-9:
    return None
  slope = sum((x - x_bar) * (y - y_bar) for x, y in zip(xs, ys, strict=True)) / denominator
  return slope if math.isfinite(slope) and abs(slope) <= 70.0 else None


def lead_dict(lead: Any, path: Path, mono_time: int, start_ns: int) -> dict[str, Any]:
  return {
    "route": route_name(path),
    "segment": segment_number(path),
    "t": round((mono_time - start_ns) / 1e9, 2),
    "trackId": safe_int(safe_attr(lead, "radarTrackId", -1)),
    "dRel": round(safe_float(safe_attr(lead, "dRel", 0.0)), 3),
    "yRel": round(safe_float(safe_attr(lead, "yRel", 0.0)), 3),
    "vRel": round(safe_float(safe_attr(lead, "vRel", 0.0)), 3),
    "vLead": round(safe_float(safe_attr(lead, "vLead", 0.0)), 3),
    "aLeadK": round(safe_float(safe_attr(lead, "aLeadK", 0.0)), 3),
    "modelProb": round(safe_float(safe_attr(lead, "modelProb", 0.0)), 3),
  }


def scan_radar(paths: list[Path]) -> dict[str, Any]:
  confirmation_point_samples = 0
  live_track_frames = 0
  confirmation_track_frames = 0
  centered_confirmation_track_frames = 0
  cut_in_candidate_frames = 0
  radar_state_frames = 0
  confirmation_lead_frames = 0
  confirmation_lead_with_live_track_frames = 0
  confirmation_track_without_state_lead_frames = 0
  low_speed_confirmation_stop_frames = 0
  raw_vs_state_vrel_errors: list[float] = []
  raw_vs_state_vlead_errors: list[float] = []
  raw_vs_derived_vrel_errors: list[float] = []
  matched_raw_vs_derived_vrel_errors: list[float] = []
  matched_model_vs_derived_vrel_errors: list[float] = []
  confirmation_examples: list[dict[str, Any]] = []
  low_speed_stop_examples: list[dict[str, Any]] = []
  raw_velocity_error_examples: list[dict[str, Any]] = []
  distance_histories: dict[tuple[str, int, int], deque[tuple[int, float]]] = defaultdict(deque)
  latest_derived_vrel: dict[int, tuple[int, float]] = {}
  live_track_ages: dict[int, int] = {}
  previous_lateral: dict[tuple[str, int], tuple[int, float]] = {}
  confirmation_track_ids: set[int] = set()
  metadata: list[dict[str, Any]] = []

  for path in paths:
    start_ns: int | None = None
    latest_live_tracks: dict[int, Any] = {}
    latest_v_ego = 0.0

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      which = msg.which()
      mono_time = int(msg.logMonoTime)
      if start_ns is None and which in ("carState", "radarState", "liveTracks"):
        start_ns = mono_time
      if which == "initData":
        init_data = msg.initData
        metadata.append({
          "route": route_name(path),
          "segment": segment_number(path),
          "gitCommit": str(safe_attr(init_data, "gitCommit", "unknown")),
          "gitBranch": str(safe_attr(init_data, "gitBranch", "unknown")),
          "gitRemote": str(safe_attr(init_data, "gitRemote", "unknown")),
          "dirty": bool(safe_attr(init_data, "dirty", False)),
        })
      elif which == "carState":
        latest_v_ego = safe_float(safe_attr(msg.carState, "vEgo", 0.0))
      elif which == "liveTracks":
        points = list(safe_attr(msg.liveTracks, "points", []))
        if points:
          live_track_frames += 1
        latest_live_tracks = {}
        previous_live_track_ids = set(live_track_ages)
        current_live_track_ids: set[int] = set()
        has_confirmation = False
        has_centered_confirmation = False
        has_cut_in_candidate = False
        for point in points:
          track_id = safe_int(safe_attr(point, "trackId", -1))
          latest_live_tracks[track_id] = point
          current_live_track_ids.add(track_id)
          if is_confirmation_track(track_id):
            confirmation_track_ids.add(track_id)
            has_confirmation = True
            confirmation_point_samples += 1
            key = (route_name(path), segment_number(path), track_id)
            d_rel = safe_float(safe_attr(point, "dRel", 0.0))
            y_rel = safe_float(safe_attr(point, "yRel", 0.0))
            if d_rel <= 100.0 and abs(y_rel) <= 1.15:
              has_centered_confirmation = True

            previous_y = previous_lateral.get(key)
            if previous_y is not None:
              prev_mono_time, prev_y_rel = previous_y
              dt = (mono_time - prev_mono_time) / 1e9
              lateral_closing = abs(y_rel) < abs(prev_y_rel) - 0.08
              if 0.02 <= dt <= 0.25 and 4.0 <= d_rel <= 90.0 and 0.45 <= abs(y_rel) <= 2.8 and lateral_closing:
                has_cut_in_candidate = True
            previous_lateral[key] = (mono_time, y_rel)

            history = distance_histories[key]
            if history:
              dt = (mono_time - history[-1][0]) / 1e9
              distance_jump = abs(d_rel - history[-1][1])
              if not (0.005 <= dt <= 0.25) or distance_jump > max(1.5, 60.0 * dt):
                history.clear()
            history.append((mono_time, d_rel))
            while history and (mono_time - history[0][0]) / 1e9 > 0.6:
              history.popleft()
            derived_vrel = distance_slope(history)
            if derived_vrel is not None:
              raw_vrel = safe_float(safe_attr(point, "vRel", 0.0))
              raw_vs_derived_vrel_errors.append(abs(raw_vrel - derived_vrel))
              latest_derived_vrel[track_id] = (mono_time, derived_vrel)
        live_track_ages = {
          track_id: live_track_ages.get(track_id, 0) + 1
          for track_id in current_live_track_ids
        }
        for track_id in previous_live_track_ids - current_live_track_ids:
          latest_derived_vrel.pop(track_id, None)
        if has_confirmation:
          confirmation_track_frames += 1
        if has_centered_confirmation:
          centered_confirmation_track_frames += 1
        if has_cut_in_candidate:
          cut_in_candidate_frames += 1
      elif which == "radarState" and start_ns is not None:
        radar_state_frames += 1
        has_live_confirmation = any(is_confirmation_track(track_id) for track_id in latest_live_tracks)
        has_state_confirmation = False
        for lead_name in ("leadOne", "leadTwo"):
          lead = safe_attr(msg.radarState, lead_name)
          if lead is None or not bool(safe_attr(lead, "status", False)):
            continue
          track_id = safe_int(safe_attr(lead, "radarTrackId", -1))
          if not (bool(safe_attr(lead, "radar", False)) and is_confirmation_track(track_id)):
            continue

          has_state_confirmation = True
          confirmation_lead_frames += 1
          live_track = latest_live_tracks.get(track_id)
          if live_track is not None:
            confirmation_lead_with_live_track_frames += 1
            raw_vrel = safe_float(safe_attr(live_track, "vRel", 0.0))
            raw_vlead = latest_v_ego + raw_vrel
            state_vrel = safe_float(safe_attr(lead, "vRel", 0.0))
            state_vlead = safe_float(safe_attr(lead, "vLead", 0.0))
            vrel_error = abs(raw_vrel - state_vrel)
            vlead_error = abs(raw_vlead - state_vlead)
            raw_vs_state_vrel_errors.append(vrel_error)
            raw_vs_state_vlead_errors.append(vlead_error)
            derived = latest_derived_vrel.get(track_id)
            if derived is not None and 0.0 <= (mono_time - derived[0]) / 1e9 <= 0.15:
              derived_vrel = derived[1]
              raw_error = abs(raw_vrel - derived_vrel)
              matched_raw_vs_derived_vrel_errors.append(raw_error)

              # Current runtime intentionally publishes model velocity for an
              # R0100-associated lead; raw velocity remains association evidence.
              model_vrel = state_vrel
              model_error = abs(model_vrel - derived_vrel)
              matched_model_vs_derived_vrel_errors.append(model_error)
            if vrel_error > 3.0 and len(raw_velocity_error_examples) < 12:
              example = lead_dict(lead, path, mono_time, start_ns)
              example.update({
                "lead": lead_name,
                "rawVRel": round(raw_vrel, 3),
                "rawVLead": round(raw_vlead, 3),
                "vRelError": round(vrel_error, 3),
                "vLeadError": round(vlead_error, 3),
              })
              raw_velocity_error_examples.append(example)

          if len(confirmation_examples) < 12:
            example = lead_dict(lead, path, mono_time, start_ns)
            example["lead"] = lead_name
            confirmation_examples.append(example)

          if latest_v_ego <= 3.8 and safe_float(safe_attr(lead, "dRel", 999.0)) <= 10.5 and safe_float(safe_attr(lead, "vLead", 99.0)) <= 1.0:
            low_speed_confirmation_stop_frames += 1
            if len(low_speed_stop_examples) < 12:
              example = lead_dict(lead, path, mono_time, start_ns)
              example["lead"] = lead_name
              example["vEgo"] = round(latest_v_ego, 3)
              low_speed_stop_examples.append(example)
        if has_live_confirmation and not has_state_confirmation:
          confirmation_track_without_state_lead_frames += 1

  software: dict[tuple[str, str, bool], int] = defaultdict(int)
  for item in metadata:
    software[(item["gitCommit"], item["gitBranch"], item["dirty"])] += 1

  distance_association_ready = confirmation_lead_frames > 0
  live_track_velocity_evidence_available = bool(raw_vs_state_vrel_errors or raw_vs_derived_vrel_errors)
  velocity_control_ready = (
    len(raw_vs_state_vrel_errors) >= 200 and
    (percentile(raw_vs_state_vrel_errors, 95.0) or 999.0) < 1.0 and
    (percentile(matched_raw_vs_derived_vrel_errors, 95.0) or 999.0) < 1.0
  )
  geometry_tandem_ready = (
    distance_association_ready and
    confirmation_lead_with_live_track_frames > 0 and
    low_speed_confirmation_stop_frames > 0
  )
  velocity_fusion_ready = geometry_tandem_ready
  tandem_ready = geometry_tandem_ready and velocity_fusion_ready
  visual_radar_track_ready = live_track_frames > 0 and confirmation_point_samples > 0
  unpromoted_radar_track_frames_observed = confirmation_track_without_state_lead_frames > 0
  cut_in_tracking_evidence = cut_in_candidate_frames > 0

  return {
    "filesScanned": len(paths),
    "routes": sorted({route_name(path) for path in paths}),
    "software": [
      {"gitCommit": commit, "gitBranch": branch, "dirty": dirty, "files": count}
      for (commit, branch, dirty), count in sorted(software.items())
    ],
    "liveTrackFrames": live_track_frames,
    "confirmationTrackFrames": confirmation_track_frames,
    "centeredConfirmationTrackFrames": centered_confirmation_track_frames,
    "cutInCandidateFrames": cut_in_candidate_frames,
    "confirmationPointSamples": confirmation_point_samples,
    "confirmationTrackIds": sorted(confirmation_track_ids),
    "radarStateFrames": radar_state_frames,
    "confirmationLeadFrames": confirmation_lead_frames,
    "confirmationLeadWithLiveTrackFrames": confirmation_lead_with_live_track_frames,
    "confirmationTrackWithoutStateLeadFrames": confirmation_track_without_state_lead_frames,
    "lowSpeedConfirmationStopFrames": low_speed_confirmation_stop_frames,
    "rawVsStateVRelError": summarize_values(raw_vs_state_vrel_errors),
    "rawVsStateVLeadError": summarize_values(raw_vs_state_vlead_errors),
    "rawVsDerivedVRelError": summarize_values(raw_vs_derived_vrel_errors),
    "matchedRawVsDerivedVRelError": summarize_values(matched_raw_vs_derived_vrel_errors),
    "matchedModelVsDerivedVRelError": summarize_values(matched_model_vs_derived_vrel_errors),
    "confirmationExamples": confirmation_examples,
    "lowSpeedStopExamples": low_speed_stop_examples,
    "rawVelocityErrorExamples": raw_velocity_error_examples,
    "distanceAssociationReady": distance_association_ready,
    "liveTrackVelocityEvidenceAvailable": live_track_velocity_evidence_available,
    "publishReady": distance_association_ready,
    "geometryTandemReady": geometry_tandem_ready,
    "velocityFusionReady": velocity_fusion_ready,
    "tandemReady": tandem_ready,
    "visualRadarTrackReady": visual_radar_track_ready,
    "unpromotedRadarTrackFramesObserved": unpromoted_radar_track_frames_observed,
    "cutInTrackingEvidence": cut_in_tracking_evidence,
    "velocityControlReady": velocity_control_ready,
    "velocityPromotionReady": velocity_control_ready,
    "boundedVelocityBlendReady": False,
    "readinessConclusion": (
      "radar_confirmed_geometry_model_velocity_only"
      if geometry_tandem_ready else
      "velocity_control_candidate"
      if velocity_control_ready else
      "distance_only_confirmation"
      if distance_association_ready else
      "insufficient_confirmation_data"
    ),
  }


def build_report(paths: list[Path], radar_only: bool = False,
                 qualification_report: Path | None = None) -> dict[str, Any]:
  if radar_only:
    long_summary = {
      "samples": 0,
      "leadDepartureOpportunities": [],
      "stopReleaseOpportunities": [],
      "noContextHighwayHardBrakes": [],
      "accelJumps": [],
    }
  else:
    from scan_longitudinal_quality import analyze as analyze_longitudinal
    from scan_longitudinal_quality import read_samples_and_metadata

    mode = ReadMode.AUTO_INTERACTIVE
    long_samples = []
    long_metadata = []
    for path in paths:
      samples, metadata = read_samples_and_metadata(path, mode)
      long_samples.extend(samples)
      if metadata is not None:
        long_metadata.append(metadata)

    long_summary = analyze_longitudinal(long_samples, long_metadata)
  radar_summary = scan_radar(paths)
  if qualification_report is not None:
    qualification = json.loads(qualification_report.read_text())["velocityControlQualification"]
    radar_summary["velocityControlQualification"] = qualification
    if qualification.get("actuationReady", False):
      radar_summary["boundedVelocityBlendReady"] = True
      radar_summary["velocityControlReady"] = True
      radar_summary["velocityFusionReady"] = True
      radar_summary["readinessConclusion"] = "model_matched_radar_velocity_ready"
      radar_summary["tandemReady"] = bool(radar_summary["distanceAssociationReady"])
  status = "pass" if radar_summary["tandemReady"] else "warn" if radar_summary["publishReady"] else "fail"
  return {
    "status": status,
    "currentCommit": current_commit(),
    "radarOnly": radar_only,
    "longitudinal": {
      "samples": long_summary.get("samples", 0),
      "leadDepartureOpportunities": long_summary.get("leadDepartureOpportunities", []),
      "stopReleaseOpportunities": long_summary.get("stopReleaseOpportunities", []),
      "noContextHighwayHardBrakes": long_summary.get("noContextHighwayHardBrakes", []),
      "accelJumps": long_summary.get("accelJumps", []),
    },
    "radar": radar_summary,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description="Carnival radar-track longitudinal readiness report.")
  parser.add_argument("logs", nargs="+", help="qlog paths/globs")
  parser.add_argument("--radar-only", action="store_true", help="Skip longitudinal sample parsing for faster rlog raw-track checks.")
  parser.add_argument("--max-files", type=int, default=0,
                      help="Limit scanned files after path expansion. 0 means all files.")
  parser.add_argument("--recent-first", action="store_true",
                      help="Scan newest files first after path expansion.")
  parser.add_argument("--out", help="Optional JSON output path")
  parser.add_argument("--qualification-report", type=Path,
                      help="Raw-CAN factory-SCC velocity qualification report")
  args = parser.parse_args()

  paths = expand_log_paths(args.logs)
  if args.recent_first:
    paths = sorted(paths, key=lambda path: path.stat().st_mtime, reverse=True)
  if args.max_files > 0:
    paths = paths[:args.max_files]
  report = build_report(paths, radar_only=args.radar_only, qualification_report=args.qualification_report)
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
