#!/usr/bin/env python3
"""Replay production radard and audit Carnival R0100 target/velocity behavior."""

from __future__ import annotations

import argparse
import bisect
import glob
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any

from openpilot.selfdrive.test.process_replay import replay_process_with_name
from openpilot.tools.lib.logreader import LogReader, ReadMode


R0100_TRACK_MIN = 0xC4100
R0100_TRACK_MAX = 0xC42FF
PRIMARY_TRACK_MIN = 0xC4200
PRIMARY_TRACK_MAX = 0xC42FF
RADAR_TO_CAMERA = 1.52


def is_r0100(track_id: int) -> bool:
  return R0100_TRACK_MIN <= track_id <= R0100_TRACK_MAX


def is_primary(track_id: int) -> bool:
  return PRIMARY_TRACK_MIN <= track_id <= PRIMARY_TRACK_MAX


def safe_float(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if math.isfinite(result) else default


def percentile(values: list[float], pct: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
  return round(ordered[index], 4)


def summary(values: list[float]) -> dict[str, Any]:
  return {
    "count": len(values),
    "p50": percentile(values, 50.0),
    "p95": percentile(values, 95.0),
    "p99": percentile(values, 99.0),
    "max": None if not values else round(max(values), 4),
  }


def latest_at(times: list[int], values: list[Any], mono_time: int, max_age_ns: int) -> Any | None:
  index = bisect.bisect_right(times, mono_time) - 1
  if index < 0 or mono_time - times[index] > max_age_ns:
    return None
  return values[index]


def future_rate(observations: list[tuple[int, float]], now: int, distance: float) -> tuple[float, float] | None:
  future = next(((time, d_rel) for time, d_rel in observations
                 if 0.25 <= (time - now) / 1e9 <= 0.60), None)
  if future is None:
    return None
  dt = (future[0] - now) / 1e9
  return (future[1] - distance) / dt, dt


def expand_paths(patterns: list[str]) -> list[Path]:
  paths: list[Path] = []
  for pattern in patterns:
    matches = [Path(path) for path in glob.glob(pattern)]
    paths.extend(matches if matches else [Path(pattern)])
  return sorted({path.resolve() for path in paths if path.is_file()})


def route_name(path: Path) -> str:
  parts = path.parent.name.split("--")
  return "--".join(parts[:2]) if len(parts) >= 2 else path.parent.name


def decode_scc_refs(messages: list[Any]) -> tuple[list[int], list[tuple[float, float]]]:
  times: list[int] = []
  values: list[tuple[float, float]] = []
  for msg in messages:
    if msg.which() != "can":
      continue
    for can in msg.can:
      if int(can.src) != 0 or int(can.address) != 0x1A0 or len(can.dat) != 32:
        continue
      raw = int.from_bytes(bytes(can.dat), "little", signed=False)
      d_rel = ((raw >> 24) & 0x7FF) * 0.1
      if not (0.5 <= d_rel <= 200.0):
        continue
      times.append(int(msg.logMonoTime))
      values.append((d_rel, ((raw >> 35) & 0x1FF) * 0.1 - 16.4))
  return times, values


def replay_path(path: Path, with_card: bool) -> dict[str, Any]:
  messages = list(LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False))
  services = Counter(msg.which() for msg in messages)
  required = {"carParams", "carState", "liveTracks", "modelV2"}
  missing = sorted(required - services.keys())
  if missing:
    return {"path": str(path), "status": "skip", "missing": missing}

  output_store: dict[str, dict[str, str]] = {}
  processes = ["card", "radard"] if with_card else ["radard"]
  process_output = replay_process_with_name(processes, messages, captured_output_store=output_store, disable_progress=True)
  replayed = [msg for msg in process_output if msg.which() == "radarState"]

  event_series: dict[str, tuple[list[int], list[Any]]] = {}
  for service in ("carState", "liveTracks", "modelV2"):
    generated = [msg for msg in process_output if msg.which() == service]
    values = generated if generated else [msg for msg in messages if msg.which() == service]
    event_series[service] = ([int(msg.logMonoTime) for msg in values], values)
  observations: dict[int, list[tuple[int, float]]] = {}
  for tracks_msg in event_series["liveTracks"][1]:
    now = int(tracks_msg.logMonoTime)
    for point in tracks_msg.liveTracks.points:
      track_id = int(point.trackId)
      if is_r0100(track_id):
        observations.setdefault(track_id, []).append((now, safe_float(point.dRel)))
  scc_times, scc_values = decode_scc_refs(messages)

  metrics: Counter[str] = Counter()
  corrections: list[float] = []
  raw_vs_scc: list[float] = []
  replay_vs_scc: list[float] = []
  model_vs_scc: list[float] = []
  scc_distance_residuals: list[float] = []
  authority_raw_vs_scc: list[float] = []
  authority_replay_vs_scc: list[float] = []
  authority_model_vs_scc: list[float] = []
  authority_acceleration: list[float] = []
  authority_model_acceleration: list[float] = []
  authority_acceleration_delta: list[float] = []
  full_raw_counterfactual_degradation: list[float] = []
  future_raw_errors: list[float] = []
  future_replay_errors: list[float] = []
  future_model_errors: list[float] = []
  future_accel_errors: list[float] = []
  future_replay_degradation: list[float] = []
  accel_magnitudes: list[float] = []
  switch_examples: list[dict[str, Any]] = []
  velocity_examples: list[dict[str, Any]] = []
  harmful_authority_examples: list[dict[str, Any]] = []
  full_raw_harmful_examples: list[dict[str, Any]] = []
  highway_radar_only_examples: list[dict[str, Any]] = []
  previous_r0100_id = -1

  for msg in replayed:
    mono_time = int(msg.logMonoTime)
    car_state_msg = latest_at(*event_series["carState"], mono_time, 150_000_000)
    tracks_msg = latest_at(*event_series["liveTracks"], mono_time, 150_000_000)
    model_msg = latest_at(*event_series["modelV2"], mono_time, 150_000_000)
    if car_state_msg is None or tracks_msg is None or model_msg is None:
      metrics["contextMissing"] += 1
      continue

    v_ego = safe_float(car_state_msg.carState.vEgo)
    model = model_msg.modelV2
    model_v_ego = safe_float(model.velocity.x[0]) if len(model.velocity.x) else v_ego
    live_tracks = {int(point.trackId): point for point in tracks_msg.liveTracks.points}

    for lead_index, lead in enumerate((msg.radarState.leadOne, msg.radarState.leadTwo)):
      if not lead.status:
        continue
      metrics["leadFrames"] += 1
      track_id = int(lead.radarTrackId)
      model_leads = list(model.leadsV3)
      model_lead = model_leads[lead_index] if lead_index < len(model_leads) else None
      model_prob = safe_float(model_lead.prob) if model_lead is not None else 0.0
      selected_model_prob = safe_float(lead.modelProb)
      model_v_rel = (safe_float(model_lead.v[0]) - model_v_ego
                     if model_lead is not None and len(model_lead.v) else None)

      if not is_r0100(track_id):
        continue
      metrics["r0100LeadFrames"] += 1
      if is_primary(track_id):
        metrics["primaryLeadFrames"] += 1
      if previous_r0100_id >= 0 and previous_r0100_id != track_id:
        metrics["r0100Switches"] += 1
        if len(switch_examples) < 20:
          switch_examples.append({"t": mono_time, "from": previous_r0100_id, "to": track_id,
                                  "dRel": round(safe_float(lead.dRel), 3)})
      previous_r0100_id = track_id

      point = live_tracks.get(track_id)
      if point is None:
        metrics["selectedTrackMissing"] += 1
        continue
      metrics["selectedTrackPresent"] += 1
      raw_v_rel = safe_float(point.vRel)
      replay_v_rel = safe_float(lead.vRel)
      raw_delta = abs(replay_v_rel - raw_v_rel)
      model_delta = None if model_v_rel is None else abs(replay_v_rel - model_v_rel)
      if raw_delta <= 0.03:
        metrics["rawVelocityCompatible"] += 1
      if model_delta is not None and model_delta <= 0.03:
        metrics["modelVelocityCompatible"] += 1
      distinct_model_raw_authority = bool(
        is_primary(track_id) and raw_delta <= 0.03 and model_delta is not None and
        abs(raw_v_rel - model_v_rel) > 0.03 and selected_model_prob > 1e-6
      )
      if is_primary(track_id) and raw_delta <= 0.03 and model_delta is not None and abs(raw_v_rel - model_v_rel) > 0.03:
        # get_lead preserves the filtered model probability for every model-associated
        # lead. Only the low-speed radar override calls get_RadarState() without a
        # probability and therefore publishes exactly zero.
        authority_kind = "modelMatchedRawAuthority" if selected_model_prob > 1e-6 else "lowSpeedRadarOnlyAuthority"
        metrics[authority_kind] += 1
        model_acceleration = (safe_float(model_lead.a[0])
                              if model_lead is not None and len(model_lead.a) else 0.0)
        replay_acceleration = safe_float(lead.aLeadK)
        authority_acceleration.append(abs(replay_acceleration))
        authority_model_acceleration.append(abs(model_acceleration))
        authority_acceleration_delta.append(abs(replay_acceleration - model_acceleration))
        if len(velocity_examples) < 20:
          velocity_examples.append({
            "t": mono_time,
            "leadIndex": lead_index,
            "trackId": track_id,
            "dRel": round(safe_float(lead.dRel), 3),
            "vEgo": round(v_ego, 3),
            "modelProb": round(model_prob, 3),
            "selectedModelProb": round(selected_model_prob, 3),
            "authorityKind": authority_kind,
            "rawVRel": round(raw_v_rel, 3),
            "modelVRel": round(model_v_rel, 3),
            "replayVRel": round(replay_v_rel, 3),
            "aLeadK": round(safe_float(lead.aLeadK), 3),
          })
      if model_delta is not None:
        corrections.append(abs(replay_v_rel - model_v_rel))
      accel_magnitudes.append(abs(safe_float(lead.aLeadK)))

      future = future_rate(observations.get(track_id, []), mono_time, safe_float(point.dRel))
      if future is not None and model_v_rel is not None:
        future_v_rel, future_dt = future
        raw_error = abs(raw_v_rel - future_v_rel)
        replay_error = abs(replay_v_rel - future_v_rel)
        model_error = abs(model_v_rel - future_v_rel)
        future_raw_errors.append(raw_error)
        future_replay_errors.append(replay_error)
        future_model_errors.append(model_error)
        degradation = replay_error - model_error
        future_replay_degradation.append(degradation)
        if degradation > 0.5:
          metrics["futureReplayHarmfulOver0_5"] += 1
        raw_a_rel = float(point.aRel)
        if math.isfinite(raw_a_rel):
          forecast_v_rel = raw_v_rel + 0.5 * raw_a_rel * future_dt
          forecast_error = abs(forecast_v_rel - future_v_rel)
          future_accel_errors.append(forecast_error)
          if forecast_error > model_error + 0.5:
            metrics["futureRadarAccelHarmfulOver0_5"] += 1

      if selected_model_prob <= 1e-6:
        metrics["radarOnlyFrames"] += 1
        if v_ego > 5.0:
          metrics["highwayRadarOnlyFrames"] += 1
          if len(highway_radar_only_examples) < 20:
            highway_radar_only_examples.append({
              "t": mono_time,
              "leadIndex": lead_index,
              "trackId": track_id,
              "dRel": round(safe_float(lead.dRel), 3),
              "vEgo": round(v_ego, 3),
              "instantModelProb": round(model_prob, 3),
              "selectedModelProb": round(selected_model_prob, 3),
            })

      if scc_times and lead_index == 0:
        index = bisect.bisect_left(scc_times, mono_time)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(scc_times)]
        if candidates:
          best = min(candidates, key=lambda i: abs(scc_times[i] - mono_time))
          scc_d_rel, scc_v_rel = scc_values[best]
          distance_residual = abs(safe_float(lead.dRel) - scc_d_rel)
          if abs(scc_times[best] - mono_time) <= 60_000_000:
            metrics["factorySccTimeMatches"] += 1
            scc_distance_residuals.append(distance_residual)
          if (abs(scc_times[best] - mono_time) <= 60_000_000 and
              distance_residual <= max(2.0, 0.03 * scc_d_rel)):
            metrics["factorySccMatches"] += 1
            raw_vs_scc.append(abs(raw_v_rel - scc_v_rel))
            replay_vs_scc.append(abs(replay_v_rel - scc_v_rel))
            if model_v_rel is not None:
              raw_error = abs(raw_v_rel - scc_v_rel)
              model_error = abs(model_v_rel - scc_v_rel)
              model_vs_scc.append(model_error)
              if is_primary(track_id):
                degradation = raw_error - model_error
                full_raw_counterfactual_degradation.append(degradation)
                if degradation > 0.5:
                  metrics["fullRawCounterfactualHarmfulOver0_5"] += 1
                  if len(full_raw_harmful_examples) < 20:
                    full_raw_harmful_examples.append({
                      "t": mono_time,
                      "leadIndex": lead_index,
                      "trackId": track_id,
                      "dRel": round(safe_float(lead.dRel), 3),
                      "sccDRel": round(scc_d_rel, 3),
                      "vEgo": round(v_ego, 3),
                      "selectedModelProb": round(selected_model_prob, 3),
                      "sccVRel": round(scc_v_rel, 3),
                      "rawVRel": round(raw_v_rel, 3),
                      "modelVRel": round(model_v_rel, 3),
                      "rawError": round(raw_error, 3),
                      "modelError": round(model_error, 3),
                    })
              if distinct_model_raw_authority:
                replay_error = abs(replay_v_rel - scc_v_rel)
                authority_raw_vs_scc.append(raw_error)
                authority_replay_vs_scc.append(replay_error)
                authority_model_vs_scc.append(model_error)
                if replay_error > model_error + 0.5:
                  metrics["harmfulAuthorityRegressionsOver0_5"] += 1
                  if len(harmful_authority_examples) < 20:
                    harmful_authority_examples.append({
                      "t": mono_time,
                      "leadIndex": lead_index,
                      "trackId": track_id,
                      "dRel": round(safe_float(lead.dRel), 3),
                      "sccDRel": round(scc_d_rel, 3),
                      "vEgo": round(v_ego, 3),
                      "instantModelProb": round(model_prob, 3),
                      "selectedModelProb": round(selected_model_prob, 3),
                      "sccVRel": round(scc_v_rel, 3),
                      "rawVRel": round(raw_v_rel, 3),
                      "modelVRel": round(model_v_rel, 3),
                      "replayVRel": round(replay_v_rel, 3),
                      "rawError": round(raw_error, 3),
                      "modelError": round(model_error, 3),
                      "replayError": round(replay_error, 3),
                    })

  stderr = "\n".join(output_store.get(process, {}).get("err", "") for process in processes)
  process_failed = "Traceback (most recent call last)" in stderr
  return {
    "path": str(path),
    "route": route_name(path),
    "status": "pass" if replayed and not process_failed else "fail",
    "processes": processes,
    "services": {name: services[name] for name in sorted(required)},
    "replayedRadarStates": len(replayed),
    "metrics": dict(metrics),
    "velocityCorrection": summary(corrections),
    "factorySccError": {
      "distanceResidual": summary(scc_distance_residuals),
      "raw": summary(raw_vs_scc),
      "replayed": summary(replay_vs_scc),
      "model": summary(model_vs_scc),
      "distinctModelMatchedRawAuthority": {
        "raw": summary(authority_raw_vs_scc),
        "replayed": summary(authority_replay_vs_scc),
        "model": summary(authority_model_vs_scc),
      },
      "fullRawPrimaryCounterfactualDegradation": summary(full_raw_counterfactual_degradation),
    },
    "futureVelocityError": {
      "raw": summary(future_raw_errors),
      "replayed": summary(future_replay_errors),
      "model": summary(future_model_errors),
      "rawWithDecodedAcceleration": summary(future_accel_errors),
      "replayMinusModel": summary(future_replay_degradation),
    },
    "accelerationMagnitude": summary(accel_magnitudes),
    "modelMatchedRawAuthorityAcceleration": {
      "replayedMagnitude": summary(authority_acceleration),
      "modelMagnitude": summary(authority_model_acceleration),
      "differenceMagnitude": summary(authority_acceleration_delta),
    },
    "switchExamples": switch_examples,
    "velocityExamples": velocity_examples,
    "harmfulAuthorityExamples": harmful_authority_examples,
    "fullRawHarmfulExamples": full_raw_harmful_examples,
    "highwayRadarOnlyExamples": highway_radar_only_examples,
    "stderr": stderr[-4000:],
  }


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out", type=Path)
  parser.add_argument("--radard-only", action="store_true",
                      help="Use recorded liveTracks instead of regenerating them from raw CAN")
  args = parser.parse_args()

  paths = expand_paths(args.logs)
  results = [replay_path(path, with_card=not args.radard_only) for path in paths]
  totals: Counter[str] = Counter()
  for result in results:
    totals.update(result.get("metrics", {}))

  process_pass = bool(results) and all(result["status"] in ("pass", "skip") for result in results)
  authority_truth_samples = sum(
    result.get("factorySccError", {}).get("distinctModelMatchedRawAuthority", {}).get("replayed", {}).get("count", 0)
    for result in results
  )
  authority_truth_routes = sum(
    result.get("factorySccError", {}).get("distinctModelMatchedRawAuthority", {}).get("replayed", {}).get("count", 0) > 0
    for result in results
  )
  acceptance = {
    "processReplayPass": process_pass,
    "primaryPublisherObserved": totals["primaryLeadFrames"] > 0,
    "independentFactoryTruthSamples": authority_truth_samples,
    "independentFactoryTruthRoutes": authority_truth_routes,
    "twoRouteAuthorityProof": authority_truth_routes >= 2 and authority_truth_samples >= 50,
    "harmfulAuthorityRegressionsOver0_5": totals["harmfulAuthorityRegressionsOver0_5"],
    "zeroHarmfulAuthorityRegressions": totals["harmfulAuthorityRegressionsOver0_5"] == 0,
    "highwayRadarOnlyFrames": totals["highwayRadarOnlyFrames"],
    "zeroHighwayRadarOnlyPromotions": totals["highwayRadarOnlyFrames"] == 0,
  }
  semantic_pass = all((
    acceptance["processReplayPass"],
    acceptance["primaryPublisherObserved"],
    acceptance["twoRouteAuthorityProof"],
    acceptance["zeroHarmfulAuthorityRegressions"],
    acceptance["zeroHighwayRadarOnlyPromotions"],
  ))

  report = {
    "status": "pass" if semantic_pass else "fail",
    "policy": {
      "velocityAuthority": "decoded raw R0100 primary track inside production qualification envelope",
      "fallback": "model velocity and acceleration atomically",
    },
    "files": len(results),
    "acceptance": acceptance,
    "totals": dict(totals),
    "results": results,
  }
  output = json.dumps(report, indent=2, sort_keys=True)
  if args.out:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(output + "\n")
  print(output)
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
