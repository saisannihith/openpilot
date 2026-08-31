#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
import zlib
from collections import defaultdict, deque
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from openpilot.tools.lib.logreader import LogReader, ReadMode


TRACK_MIN = 0xC4100
TRACK_MAX = 0xC41FF
HISTORY_SECONDS = 1.0
FUTURE_MIN_SECONDS = 0.25
FUTURE_MAX_SECONDS = 0.60


@dataclass(frozen=True)
class Policy:
  name: str
  min_track_frames: int
  min_selected_frames: int
  max_consensus_residual: float
  min_model_delta: float
  max_distance: float
  max_ttc: float
  blend_weight: float
  max_correction: float
  min_distance: float = 0.0
  bidirectional: bool = False
  max_model_residual: float = math.inf
  direct_radar: bool = False
  relative_accel_forecast: bool = False
  temporal_frames: int = 0
  max_temporal_std: float = math.inf
  min_sign_fraction: float = 0.0
  min_model_v_std: float = 0.0
  min_model_prob: float = 0.0


POLICIES = (
  Policy("broad", 8, 3, 2.0, 1.0, 110.0, 7.0, 0.35, 4.0),
  Policy("balanced", 12, 6, 1.5, 1.0, 90.0, 6.0, 0.35, 3.0),
  Policy("strict", 16, 8, 1.0, 1.5, 75.0, 5.0, 0.30, 2.5),
  Policy("very_strict", 20, 12, 0.75, 2.0, 60.0, 4.0, 0.25, 2.0),
  Policy("model_matched_far_observer", 8, 6, 0.75, 0.0, 60.0, math.inf, 0.35, 0.35,
         min_distance=40.0, bidirectional=True, max_model_residual=1.0),
  Policy("temporal_model_radar_observer", 12, 12, 0.60, 0.30, 80.0, math.inf, 0.25, 0.25,
         min_distance=15.0, bidirectional=True, max_model_residual=1.0,
         temporal_frames=12, max_temporal_std=0.25, min_sign_fraction=0.90),
  Policy("uncertainty_gated_observer", 12, 10, 0.60, 0.30, 80.0, math.inf, 0.20, 0.20,
         min_distance=15.0, bidirectional=True, max_model_residual=0.75,
         temporal_frames=10, max_temporal_std=0.20, min_sign_fraction=0.90,
         min_model_v_std=0.75, min_model_prob=0.75),
)


def finite(value, default=0.0):
  try:
    value = float(value)
  except Exception:
    return default
  return value if math.isfinite(value) else default


def percentile(values, pct):
  if not values:
    return None
  return round(float(np.percentile(values, pct)), 4)


def summarize(values):
  return {
    "count": len(values),
    "p50": percentile(values, 50),
    "p90": percentile(values, 90),
    "p95": percentile(values, 95),
    "max": round(max(values), 4) if values else None,
  }


def expand(patterns):
  paths = []
  for pattern in patterns:
    matches = glob.glob(pattern)
    paths.extend(matches if matches else [pattern])
  return sorted({Path(path).resolve() for path in paths if Path(path).is_file()})


def regression_rate(history):
  if len(history) < 8:
    return None
  times = np.asarray([(t - history[-1][0]) / 1e9 for t, _d in history], dtype=float)
  distances = np.asarray([d for _t, d in history], dtype=float)
  if times[-1] - times[0] < 0.30 or not np.isfinite(distances).all():
    return None
  slope = np.polyfit(times, distances, 1)[0]
  return float(slope) if math.isfinite(slope) else None


def future_rate(observations, now, distance):
  future = next(((t, d) for t, d in observations
                 if FUTURE_MIN_SECONDS <= (t - now) / 1e9 <= FUTURE_MAX_SECONDS), None)
  if future is None:
    return None
  dt = (future[0] - now) / 1e9
  return (future[1] - distance) / dt, dt


def extract(raw, start, size, signed=False):
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


def policy_output(policy, sample, temporal_state=None):
  if sample["trackFrames"] < policy.min_track_frames or sample["selectedFrames"] < policy.min_selected_frames:
    return None
  if sample["modelVStd"] < policy.min_model_v_std or sample["modelProb"] < policy.min_model_prob:
    return None
  if sample["observedVRel"] is None or abs(sample["rawVRel"] - sample["observedVRel"]) > policy.max_consensus_residual:
    return None
  if not policy.min_distance < sample["dRel"] <= policy.max_distance:
    return None

  model_vrel = sample["modelVRel"]
  if policy.temporal_frames:
    key = (sample["routeSegment"], sample["lead"], sample["trackId"])
    history = temporal_state.setdefault(key, deque(maxlen=policy.temporal_frames))
    if history and (sample["time"] - history[-1][0]) / 1e9 > 0.20:
      history.clear()
    consensus_vrel = 0.5 * (sample["rawVRel"] + sample["observedVRel"])
    residual = consensus_vrel - model_vrel
    if abs(residual) > policy.max_model_residual:
      history.clear()
      return None
    history.append((sample["time"], residual))
    if len(history) < policy.temporal_frames:
      return None
    residuals = np.asarray([item[1] for item in history], dtype=float)
    sign_fraction = max(np.mean(residuals >= 0.0), np.mean(residuals <= 0.0))
    if np.std(residuals) > policy.max_temporal_std or sign_fraction < policy.min_sign_fraction:
      return None
    filtered_residual = float(np.median(residuals))
    if abs(filtered_residual) < policy.min_model_delta:
      return None
    correction = float(np.clip(policy.blend_weight * filtered_residual,
                               -policy.max_correction, policy.max_correction))
    return float(model_vrel + correction)
  if policy.direct_radar:
    return float(sample["rawVRel"])
  if policy.bidirectional:
    if abs(sample["rawVRel"] - model_vrel) > policy.max_model_residual:
      return None
    correction = float(np.clip(policy.blend_weight * (sample["rawVRel"] - model_vrel),
                               -policy.max_correction, policy.max_correction))
    return float(model_vrel + correction)

  # Less-negative of two independent radar estimates is deliberately conservative.
  consensus_vrel = max(sample["rawVRel"], sample["observedVRel"])
  if consensus_vrel >= model_vrel - policy.min_model_delta:
    return None
  ttc = sample["dRel"] / max(-consensus_vrel, 0.1)
  if ttc > policy.max_ttc:
    return None

  correction = min(policy.max_correction, policy.blend_weight * (model_vrel - consensus_vrel))
  return model_vrel - correction


def learned_features(sample):
  observed = sample["observedVRel"]
  if observed is None:
    return None
  raw_residual = sample["rawVRel"] - sample["modelVRel"]
  observed_residual = observed - sample["modelVRel"]
  consensus_residual = 0.5 * (raw_residual + observed_residual)
  disagreement = sample["rawVRel"] - observed
  v_std = min(float(sample["modelVStd"]), 5.0)
  uncertainty = v_std * max(0.0, 1.0 - float(sample["modelProb"]))
  return np.asarray([
    raw_residual,
    observed_residual,
    consensus_residual,
    disagreement,
    abs(disagreement),
    float(sample["dRel"]) / 100.0,
    float(sample.get("yRel", 0.0)) / 3.0,
    v_std,
    float(sample["modelProb"]),
    float(sample["modelARel"]),
    float(sample.get("rawARel", 0.0)),
    min(math.log1p(float(sample["trackFrames"])), 8.0),
    min(math.log1p(float(sample["selectedFrames"])), 8.0),
    consensus_residual * v_std,
    consensus_residual * uncertainty,
    consensus_residual * float(sample["dRel"]) / 100.0,
  ], dtype=np.float64)


def evaluate_learned_observer(samples, observations):
  """Cross-validate a small self-supervised vision/radar residual observer by route.

  This is deliberately an offline proof path. A future radar range measurement is the label,
  while the feature vector contains only values available at the current frame.
  """
  rows = []
  for sample in samples:
    features = learned_features(sample)
    future_result = future_rate(
      observations[(sample["routeSegment"], sample["trackId"])], sample["time"], sample["dRel"],
    )
    if features is None or future_result is None:
      continue
    future, future_dt = future_result
    if (sample["trackFrames"] < 12 or sample["selectedFrames"] < 10 or
        sample["modelProb"] < 0.75 or not 5.0 < sample["dRel"] < 120.0 or
        abs(sample["rawVRel"] - sample["observedVRel"]) > 0.75):
      continue
    model_forecast = sample["modelVRel"] + 0.5 * sample["modelARel"] * future_dt
    target_correction = float(np.clip(future - model_forecast, -1.0, 1.0))
    fold = zlib.crc32(sample["routeSegment"].encode()) % 5
    rows.append((sample, features, target_correction, future, future_dt, fold))

  if len(rows) < 1000:
    return {"actuationReady": False, "reason": "insufficient_rows", "samples": len(rows)}

  predictions = np.zeros(len(rows), dtype=np.float64)
  coefficients = []
  for fold in range(5):
    train_idx = np.asarray([i for i, row in enumerate(rows) if row[5] != fold])
    test_idx = np.asarray([i for i, row in enumerate(rows) if row[5] == fold])
    if len(train_idx) < 100 or len(test_idx) == 0:
      continue
    x_train = np.vstack([rows[i][1] for i in train_idx])
    y_train = np.asarray([rows[i][2] for i in train_idx])
    mean = np.mean(x_train, axis=0)
    std = np.maximum(np.std(x_train, axis=0), 1e-3)
    x_norm = (x_train - mean) / std
    design = np.column_stack((np.ones(len(x_norm)), x_norm))
    ridge = np.eye(design.shape[1]) * 0.05
    ridge[0, 0] = 0.0
    beta = np.linalg.solve(design.T @ design + ridge, design.T @ y_train)
    x_test = (np.vstack([rows[i][1] for i in test_idx]) - mean) / std
    predictions[test_idx] = beta[0] + x_test @ beta[1:]
    coefficients.append({"fold": fold, "intercept": float(beta[0]), "weights": beta[1:].tolist()})

  model_errors, fused_errors, improvements, corrections = [], [], [], []
  events = defaultdict(list)
  for index, (sample, _features, _target, future, future_dt, _fold) in enumerate(rows):
    # A learned observer still has a hard authority bound. The model owns lead semantics and
    # acceleration; this estimates only a small velocity residual.
    correction = float(np.clip(predictions[index], -0.35, 0.35))
    fused_forecast = sample["modelVRel"] + correction + 0.5 * sample["modelARel"] * future_dt
    model_forecast = sample["modelVRel"] + 0.5 * sample["modelARel"] * future_dt
    model_error = abs(model_forecast - future)
    fused_error = abs(fused_forecast - future)
    improvement = model_error - fused_error
    model_errors.append(model_error)
    fused_errors.append(fused_error)
    improvements.append(improvement)
    corrections.append(correction)
    event_key = (sample["routeSegment"], sample["lead"], sample["trackId"], sample["time"] // int(1e9))
    events[event_key].append(improvement)

  event_improvements = [float(np.median(values)) for values in events.values()]
  event_regressions = sum(value < -0.10 for value in event_improvements)
  harmful_events = sum(value < -0.50 for value in event_improvements)
  mean_improvement = float(np.mean(improvements))
  ready = bool(
    len(event_improvements) >= 100 and mean_improvement > 0.05 and
    percentile(fused_errors, 95) < percentile(model_errors, 95) and
    event_regressions / max(len(event_improvements), 1) <= 0.01 and harmful_events == 0
  )
  return {
    "contract": "five-fold route-held-out; future radar range is label only",
    "samples": len(rows),
    "events": len(event_improvements),
    "correction": summarize(corrections),
    "modelFutureError": summarize(model_errors),
    "fusedFutureError": summarize(fused_errors),
    "meanImprovement": round(mean_improvement, 4),
    "improvementP05": percentile(improvements, 5),
    "eventRegressionsOver0_10": event_regressions,
    "eventRegressionFractionOver0_10": round(event_regressions / max(len(event_improvements), 1), 4),
    "harmfulEventsOver0_5": harmful_events,
    "actuationReady": ready,
    "coefficients": coefficients,
  }


def main():
  parser = argparse.ArgumentParser(description="Evaluate conservative R0100 velocity fusion policies")
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out")
  args = parser.parse_args()

  paths = expand(args.logs)
  samples = []
  observations = defaultdict(list)

  for path in paths:
    route_segment = path.parent.name
    histories = defaultdict(lambda: deque())
    track_frames = defaultdict(int)
    selected_track = {"leadOne": None, "leadTwo": None}
    selected_frames = defaultdict(int)
    latest = {}
    latest_dynamics = {}
    latest_model_vrel: list[float] = []
    latest_model_arel: list[float] = []
    latest_model_vstd: list[float] = []
    latest_model_prob: list[float] = []

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      now = int(msg.logMonoTime)
      which = msg.which()
      if which == "can":
        for can in msg.can:
          if int(can.src) != 1 or not 0x180 <= int(can.address) <= 0x184 or len(can.dat) != 32:
            continue
          packed = int.from_bytes(bytes(can.dat), "little", signed=False)
          for offset in (0, 128):
            raw = (packed >> offset) & ((1 << 128) - 1)
            raw_track_id = extract(raw, 42, 8)
            quality = extract(raw, 32, 8)
            state = extract(raw, 55, 3)
            if raw_track_id == 0 or quality == 0 or state == 0:
              continue
            latest_dynamics[raw_track_id] = {
              "time": now,
              "rawARel": extract(raw, 116, 8, True) * 0.1,
              "rawYVRel": extract(raw, 106, 8) * 0.2 - 25.0,
            }
      elif which == "liveTracks":
        seen = set()
        latest = {}
        for point in msg.liveTracks.points:
          track_id = int(point.trackId)
          if not TRACK_MIN <= track_id <= TRACK_MAX:
            continue
          seen.add(track_id)
          d_rel = finite(point.dRel)
          raw_vrel = finite(point.vRel)
          track_frames[track_id] += 1
          history = histories[track_id]
          history.append((now, d_rel))
          while history and (now - history[0][0]) / 1e9 > HISTORY_SECONDS:
            history.popleft()
          observations[(route_segment, track_id)].append((now, d_rel))
          latest[track_id] = {
            "time": now,
            "dRel": d_rel,
            "yRel": finite(point.yRel),
            "rawVRel": raw_vrel,
            "observedVRel": regression_rate(history),
            "trackFrames": track_frames[track_id],
          }
          dynamics = latest_dynamics.get(track_id & 0xFF)
          if dynamics is not None and 0 <= now - dynamics["time"] <= int(0.12e9):
            latest[track_id].update(dynamics)
        for track_id in list(track_frames):
          if track_id not in seen:
            track_frames[track_id] = 0
            histories[track_id].clear()
      elif which == "modelV2":
        model_v_ego = finite(msg.modelV2.velocity.x[0]) if len(msg.modelV2.velocity.x) else 0.0
        latest_model_vrel = [finite(lead.v[0]) - model_v_ego if len(lead.v) else 0.0
                             for lead in list(msg.modelV2.leadsV3)[:2]]
        latest_model_arel = [finite(lead.a[0]) if len(lead.a) else 0.0
                             for lead in list(msg.modelV2.leadsV3)[:2]]
        latest_model_vstd = [finite(lead.vStd[0], math.inf) if len(lead.vStd) else math.inf
                             for lead in list(msg.modelV2.leadsV3)[:2]]
        latest_model_prob = [finite(lead.prob) for lead in list(msg.modelV2.leadsV3)[:2]]
      elif which == "radarState":
        for lead_index, lead_name in enumerate(("leadOne", "leadTwo")):
          lead = getattr(msg.radarState, lead_name)
          track_id = int(lead.radarTrackId)
          if not lead.status or not lead.radar or track_id not in latest:
            selected_track[lead_name] = None
            continue
          if selected_track[lead_name] == track_id:
            selected_frames[(lead_name, track_id)] += 1
          else:
            selected_track[lead_name] = track_id
            selected_frames[(lead_name, track_id)] = 1
          sample = dict(latest[track_id])
          sample.update({
            "routeSegment": route_segment,
            "lead": lead_name,
            "trackId": track_id,
            "selectedFrames": selected_frames[(lead_name, track_id)],
            "modelVRel": (latest_model_vrel[lead_index]
                          if lead_index < len(latest_model_vrel) else finite(lead.vRel)),
            "modelARel": (latest_model_arel[lead_index]
                          if lead_index < len(latest_model_arel) else finite(lead.aLeadK)),
            "modelVStd": (latest_model_vstd[lead_index]
                          if lead_index < len(latest_model_vstd) else math.inf),
            "modelProb": (latest_model_prob[lead_index]
                          if lead_index < len(latest_model_prob) else finite(lead.modelProb)),
            "productionVRel": finite(lead.vRel),
          })
          samples.append(sample)

  reports = []
  for policy in POLICIES:
    model_errors = []
    fused_errors = []
    improvements = []
    corrections = []
    examples = []
    events = defaultdict(list)
    temporal_state = {}
    for sample in samples:
      fused = policy_output(policy, sample, temporal_state)
      if fused is None:
        continue
      future_result = future_rate(
        observations[(sample["routeSegment"], sample["trackId"])], sample["time"], sample["dRel"],
      )
      if future_result is None:
        continue
      future, future_dt = future_result
      if policy.relative_accel_forecast:
        if "rawARel" not in sample:
          continue
        # future_rate is the average relative velocity across the interval.
        # Under constant relative acceleration, its matching prediction is
        # v_now + 0.5 * a_rel * dt.
        fused += 0.5 * sample["rawARel"] * future_dt
      model_forecast = sample["modelVRel"] + 0.5 * sample["modelARel"] * future_dt
      fused_forecast = fused + 0.5 * sample["modelARel"] * future_dt
      model_error = abs(model_forecast - future)
      fused_error = abs(fused_forecast - future)
      improvement = model_error - fused_error
      model_errors.append(model_error)
      fused_errors.append(fused_error)
      improvements.append(improvement)
      corrections.append(sample["modelVRel"] - fused)
      event_key = (sample["routeSegment"], sample["lead"], sample["trackId"], sample["time"] // int(1e9))
      events[event_key].append(improvement)
      if improvement < -0.5 and len(examples) < 8:
        examples.append({
          "routeSegment": sample["routeSegment"],
          "trackId": sample["trackId"],
          "dRel": round(sample["dRel"], 3),
          "rawVRel": round(sample["rawVRel"], 3),
          "observedVRel": round(sample["observedVRel"], 3),
          "modelVRel": round(sample["modelVRel"], 3),
          "fusedVRel": round(fused, 3),
          "futureVRel": round(future, 3),
          "rawARel": round(sample.get("rawARel", math.nan), 3),
          "futureDt": round(future_dt, 3),
          "errorDelta": round(improvement, 3),
        })
    event_improvements = [float(np.median(values)) for values in events.values()]
    harmful = sum(value < -0.5 for value in improvements)
    harmful_events = sum(value < -0.5 for value in event_improvements)
    sample_regressions_0_05 = sum(value < -0.05 for value in improvements)
    sample_regressions_0_10 = sum(value < -0.10 for value in improvements)
    event_regressions_0_05 = sum(value < -0.05 for value in event_improvements)
    event_regressions_0_10 = sum(value < -0.10 for value in event_improvements)
    reports.append({
      "policy": asdict(policy),
      "samples": len(improvements),
      "events": len(event_improvements),
      "correction": summarize(corrections),
      "modelFutureError": summarize(model_errors),
      "fusedFutureError": summarize(fused_errors),
      "improvement": summarize(improvements),
      "meanImprovement": round(float(np.mean(improvements)), 4) if improvements else None,
      "improvementP05": percentile(improvements, 5),
      "sampleRegressionsOver0_05": sample_regressions_0_05,
      "sampleRegressionFractionOver0_05": round(sample_regressions_0_05 / max(len(improvements), 1), 4),
      "sampleRegressionsOver0_10": sample_regressions_0_10,
      "sampleRegressionFractionOver0_10": round(sample_regressions_0_10 / max(len(improvements), 1), 4),
      "eventRegressionsOver0_05": event_regressions_0_05,
      "eventRegressionFractionOver0_05": round(event_regressions_0_05 / max(len(event_improvements), 1), 4),
      "eventRegressionsOver0_10": event_regressions_0_10,
      "eventRegressionFractionOver0_10": round(event_regressions_0_10 / max(len(event_improvements), 1), 4),
      "harmfulSamplesOver0_5": harmful,
      "harmfulSampleFraction": round(harmful / max(len(improvements), 1), 4),
      "harmfulEventsOver0_5": harmful_events,
      "harmfulEventFraction": round(harmful_events / max(len(event_improvements), 1), 4),
      "harmfulExamples": examples,
      "actuationReady": bool(
        len(event_improvements) >= 100 and
        float(np.mean(improvements)) > 0.05 and
        percentile(fused_errors, 95) < percentile(model_errors, 95) and
        event_regressions_0_10 / max(len(event_improvements), 1) <= 0.01 and
        harmful_events == 0
      ),
    })

  def group_diagnostics():
    group = samples
    observed = [sample for sample in group if sample["observedVRel"] is not None]
    residuals = [abs(sample["rawVRel"] - sample["observedVRel"]) for sample in observed]
    model_deltas = [sample["modelVRel"] - max(sample["rawVRel"], sample["observedVRel"]) for sample in observed]
    return {
      "samples": len(group),
      "observedSamples": len(observed),
      "rawDistanceRateResidual": summarize(residuals),
      "modelClosingDelta": summarize(model_deltas),
      "trackFrames": summarize([sample["trackFrames"] for sample in group]),
      "selectedFrames": summarize([sample["selectedFrames"] for sample in group]),
    }

  report = {
    "files": len(paths),
    "matchedSamples": len(samples),
    "policies": reports,
    "trackGroups": {
      "r0100": group_diagnostics(),
    },
    "learnedObserver": evaluate_learned_observer(samples, observations),
  }
  report["conclusion"] = ("candidate_ready" if any(item["actuationReady"] for item in reports) or
                          report["learnedObserver"].get("actuationReady", False)
                          else "no_policy_actuation_ready")
  text = json.dumps(report, indent=2, sort_keys=True)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")


if __name__ == "__main__":
  main()
