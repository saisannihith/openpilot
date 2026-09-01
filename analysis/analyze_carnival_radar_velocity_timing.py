#!/usr/bin/env python3
"""Qualify Carnival R0100 velocity timing and track continuity from full routes."""
from __future__ import annotations

import argparse
import glob
import json
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from openpilot.tools.lib.logreader import LogReader, ReadMode


TRACK_MIN = 0xC4100
TRACK_MAX = 0xC41FF
MAX_TRACK_GAP = 0.20
LABEL_HALF_WINDOW = 0.30
LABEL_MIN_SPAN = 0.40
LABEL_MIN_POINTS = 8
LABEL_MAX_RMSE = 0.15
LAG_GRID = np.arange(-1.20, 0.3001, 0.05)


def finite(value, default=0.0):
  try:
    result = float(value)
  except Exception:
    return default
  return result if math.isfinite(result) else default


def percentile(values, pct):
  return None if len(values) == 0 else round(float(np.percentile(values, pct)), 4)


def summarize(values):
  values = np.asarray(values, dtype=np.float64)
  return {
    "count": int(len(values)),
    "mean": None if len(values) == 0 else round(float(np.mean(values)), 4),
    "p50": percentile(values, 50),
    "p90": percentile(values, 90),
    "p95": percentile(values, 95),
    "max": None if len(values) == 0 else round(float(np.max(values)), 4),
  }


def expand(patterns):
  paths = []
  for pattern in patterns:
    matches = glob.glob(pattern)
    paths.extend(matches if matches else [pattern])
  return sorted({Path(path).resolve() for path in paths if Path(path).is_file()})


def extract(raw, start, size, signed=False):
  value = (raw >> start) & ((1 << size) - 1)
  if signed and value & (1 << (size - 1)):
    value -= 1 << size
  return value


@dataclass
class TrackState:
  episode: int
  last_time: int
  d_rel: float
  y_rel: float
  v_rel: float
  age: int


@dataclass
class EpisodeSeries:
  base_time: int
  times: np.ndarray
  distances: np.ndarray
  prefix_x: np.ndarray
  prefix_y: np.ndarray
  prefix_xx: np.ndarray
  prefix_xy: np.ndarray
  prefix_yy: np.ndarray

  @classmethod
  def build(cls, observations):
    base_time = observations[0][0]
    times = np.asarray([(item[0] - base_time) / 1e9 for item in observations], dtype=np.float64)
    distances = np.asarray([item[1] for item in observations], dtype=np.float64)

    def prefix(values):
      return np.concatenate(([0.0], np.cumsum(values, dtype=np.float64)))

    return cls(base_time, times, distances, prefix(times), prefix(distances), prefix(times * times),
               prefix(times * distances), prefix(distances * distances))

  def _fit(self, start, end):
    left = int(np.searchsorted(self.times, start, side="left"))
    right = int(np.searchsorted(self.times, end, side="right"))
    count = right - left
    if count < LABEL_MIN_POINTS or self.times[right - 1] - self.times[left] < LABEL_MIN_SPAN:
      return None

    sum_x = self.prefix_x[right] - self.prefix_x[left]
    sum_y = self.prefix_y[right] - self.prefix_y[left]
    sum_xx = self.prefix_xx[right] - self.prefix_xx[left]
    sum_xy = self.prefix_xy[right] - self.prefix_xy[left]
    sum_yy = self.prefix_yy[right] - self.prefix_yy[left]
    centered_xx = sum_xx - sum_x * sum_x / count
    centered_xy = sum_xy - sum_x * sum_y / count
    centered_yy = max(0.0, sum_yy - sum_y * sum_y / count)
    if centered_xx <= 1e-9:
      return None
    slope = centered_xy / centered_xx
    residual_sse = max(0.0, centered_yy - centered_xy * centered_xy / centered_xx)
    rmse = math.sqrt(residual_sse / count)
    if not math.isfinite(slope) or rmse > LABEL_MAX_RMSE:
      return None
    mean_time = sum_x / count
    return float(slope), float(mean_time), float(rmse), count

  def centered_rate(self, time_ns, offset=0.0):
    center = (time_ns - self.base_time) / 1e9 + offset
    return self._fit(center - LABEL_HALF_WINDOW, center + LABEL_HALF_WINDOW)

  def trailing_rate(self, time_ns, window=0.60):
    end = (time_ns - self.base_time) / 1e9
    return self._fit(end - window, end)


def continuous(previous, now, d_rel, y_rel, v_rel):
  if previous is None:
    return False
  dt = (now - previous.last_time) / 1e9
  return (0.0 < dt <= MAX_TRACK_GAP and
          abs(d_rel - previous.d_rel) <= max(1.5, 60.0 * dt) and
          abs(y_rel - previous.y_rel) <= max(1.0, 20.0 * dt) and
          abs(v_rel - previous.v_rel) <= 8.0)


def extract_samples(paths):
  samples = []
  episode_observations = defaultdict(list)
  metadata_counts = defaultdict(int)
  episode_count = 0
  continuity_breaks = 0

  for path in paths:
    route_segment = path.parent.name
    track_states = {}
    track_episode_numbers = defaultdict(int)
    latest_tracks = {}
    latest_metadata = {}
    selected_episode = {"leadOne": None, "leadTwo": None}
    selected_frames = defaultdict(int)
    latest_model = []
    model_time = 0

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
            raw_id = extract(raw, 42, 8)
            valid_count = extract(raw, 32, 8)
            if raw_id == 0 or valid_count == 0:
              continue
            latest_metadata[raw_id] = {
              "time": now,
              "validCount": valid_count,
              "heartbeat": extract(raw, 124, 4),
              "state": extract(raw, 55, 3),
              "stateAlt": extract(raw, 51, 4),
              "metadata": extract(raw, 50, 14),
              "dRel": extract(raw, 64, 13) * 0.05,
              "yRel": extract(raw, 78, 11, True) * 0.05,
              "vRel": extract(raw, 91, 11, True) * 0.05 + 2.4,
            }
      elif which == "liveTracks":
        latest_tracks = {}
        for point in msg.liveTracks.points:
          track_id = int(point.trackId)
          if not TRACK_MIN <= track_id <= TRACK_MAX:
            continue
          raw_id = track_id & 0xFF
          d_rel = finite(point.dRel)
          y_rel = finite(point.yRel)
          v_rel = finite(point.vRel)
          previous = track_states.get(track_id)
          is_continuous = continuous(previous, now, d_rel, y_rel, v_rel)
          if not is_continuous:
            if previous is not None:
              continuity_breaks += 1
            track_episode_numbers[track_id] += 1
            episode_count += 1
          episode_key = (route_segment, track_id, track_episode_numbers[track_id])
          age = previous.age + 1 if is_continuous else 1
          track_states[track_id] = TrackState(track_episode_numbers[track_id], now, d_rel, y_rel, v_rel, age)
          episode_observations[episode_key].append((now, d_rel))
          metadata = latest_metadata.get(raw_id)
          if metadata is not None and (now - metadata["time"] <= int(0.12e9) and
                                       abs(metadata["dRel"] - d_rel) <= 0.1 and
                                       abs(metadata["yRel"] - y_rel) <= 0.1):
            metadata_counts[(metadata["state"], metadata["stateAlt"])] += 1
          else:
            metadata = None
          latest_tracks[track_id] = {
            "time": now,
            "episodeKey": episode_key,
            "trackId": track_id,
            "rawId": raw_id,
            "trackAge": age,
            "dRel": d_rel,
            "yRel": y_rel,
            "rawVRel": v_rel,
            "metadata": metadata,
          }
      elif which == "modelV2":
        model_time = now
        model_v_ego = finite(msg.modelV2.velocity.x[0]) if len(msg.modelV2.velocity.x) else 0.0
        model_a_ego = finite(msg.modelV2.acceleration.x[0]) if len(msg.modelV2.acceleration.x) else 0.0
        latest_model = []
        for lead in list(msg.modelV2.leadsV3)[:2]:
          lead_accel = finite(lead.a[0]) if len(lead.a) else 0.0
          latest_model.append({
            "vRel": finite(lead.v[0]) - model_v_ego if len(lead.v) else 0.0,
            "leadAccel": lead_accel,
            "egoAccel": model_a_ego,
            "aRel": lead_accel - model_a_ego,
            "vStd": finite(lead.vStd[0], math.inf) if len(lead.vStd) else math.inf,
            "prob": finite(lead.prob),
          })
      elif which == "radarState":
        for lead_index, lead_name in enumerate(("leadOne", "leadTwo")):
          lead = getattr(msg.radarState, lead_name)
          track_id = int(lead.radarTrackId)
          track = latest_tracks.get(track_id)
          if (not lead.status or not lead.radar or track is None or lead_index >= len(latest_model) or
              now - track["time"] > int(0.15e9) or now - model_time > int(0.15e9)):
            selected_episode[lead_name] = None
            continue
          episode_key = track["episodeKey"]
          selection_key = (lead_name, episode_key)
          if selected_episode[lead_name] == episode_key:
            selected_frames[selection_key] += 1
          else:
            selected_episode[lead_name] = episode_key
            selected_frames[selection_key] = 1
          sample = dict(track)
          sample.update({
            "routeSegment": route_segment,
            "lead": lead_name,
            "selectedFrames": selected_frames[selection_key],
            "modelVRel": latest_model[lead_index]["vRel"],
            "modelARel": latest_model[lead_index]["aRel"],
            "modelLeadAccel": latest_model[lead_index]["leadAccel"],
            "modelEgoAccel": latest_model[lead_index]["egoAccel"],
            "modelVStd": latest_model[lead_index]["vStd"],
            "modelProb": latest_model[lead_index]["prob"],
          })
          samples.append(sample)

  series = {key: EpisodeSeries.build(values) for key, values in episode_observations.items()
            if len(values) >= LABEL_MIN_POINTS}
  return samples, series, {
    "episodes": episode_count,
    "continuityBreaks": continuity_breaks,
    "metadataStatePairs": [
      {"state": key[0], "stateAlt": key[1], "samples": count}
      for key, count in sorted(metadata_counts.items(), key=lambda item: -item[1])
    ],
  }


def lag_report(samples, series, *, stable_only):
  reports = []
  for lag in LAG_GRID:
    raw_errors = []
    model_errors = []
    raw_residuals = []
    model_residuals = []
    for sample in samples:
      if stable_only and (sample["trackAge"] < 20 or sample["selectedFrames"] < 12 or sample["modelProb"] < 0.75):
        continue
      episode = series.get(sample["episodeKey"])
      if episode is None:
        continue
      label = episode.centered_rate(sample["time"], float(lag))
      if label is None:
        continue
      truth = label[0]
      raw_residuals.append(sample["rawVRel"] - truth)
      model_residuals.append(sample["modelVRel"] - truth)
      raw_errors.append(abs(sample["rawVRel"] - truth))
      model_errors.append(abs(sample["modelVRel"] - truth))
    reports.append({
      "lag": round(float(lag), 2),
      "samples": len(raw_errors),
      "rawBias": None if not raw_residuals else round(float(np.median(raw_residuals)), 4),
      "rawAbsError": summarize(raw_errors),
      "modelBias": None if not model_residuals else round(float(np.median(model_residuals)), 4),
      "modelAbsError": summarize(model_errors),
    })
  eligible = [item for item in reports if item["samples"] >= 1000 and item["rawAbsError"]["p95"] is not None]
  best = min(eligible, key=lambda item: (item["rawAbsError"]["p95"], item["rawAbsError"]["p50"])) if eligible else None
  return {"stableOnly": stable_only, "best": best, "sweep": reports}


def aligned_feature_report(samples, series, radar_lag):
  rows = []
  for sample in samples:
    if sample["trackAge"] < 20 or sample["selectedFrames"] < 12 or sample["modelProb"] < 0.75:
      continue
    episode = series.get(sample["episodeKey"])
    if episode is None:
      continue
    current = episode.centered_rate(sample["time"])
    trailing = episode.trailing_rate(sample["time"])
    future = episode.centered_rate(sample["time"], 0.40)
    if current is None or trailing is None or future is None:
      continue
    current_truth = current[0]
    future_truth = future[0]
    trailing_age = (sample["time"] - episode.base_time) / 1e9 - trailing[1]
    raw_current = sample["rawVRel"] - sample["modelARel"] * radar_lag
    trailing_current = trailing[0] + sample["modelARel"] * trailing_age
    model_future = sample["modelVRel"] + sample["modelARel"] * 0.40
    rows.append({
      "routeSegment": sample["routeSegment"],
      "event": (sample["routeSegment"], sample["lead"], sample["episodeKey"], sample["time"] // int(1e9)),
      "trackAge": sample["trackAge"],
      "selectedFrames": sample["selectedFrames"],
      "dRel": sample["dRel"],
      "yRel": sample["yRel"],
      "modelProb": sample["modelProb"],
      "modelVStd": sample["modelVStd"],
      "modelARel": sample["modelARel"],
      "modelLeadAccel": sample["modelLeadAccel"],
      "modelEgoAccel": sample["modelEgoAccel"],
      "modelVRel": sample["modelVRel"],
      "rawCurrent": raw_current,
      "trailingCurrent": trailing_current,
      "currentTruth": current_truth,
      "futureTruth": future_truth,
      "modelFuture": model_future,
      "rawTrailingResidual": abs(raw_current - trailing_current),
      "rawModelResidual": abs(raw_current - sample["modelVRel"]),
      "trailingModelResidual": abs(trailing_current - sample["modelVRel"]),
      "trailingAge": trailing_age,
    })
  return rows


def summarize_aligned(rows):
  return {
    "samples": len(rows),
    "rawCurrentError": summarize([abs(row["rawCurrent"] - row["currentTruth"]) for row in rows]),
    "trailingCurrentError": summarize([abs(row["trailingCurrent"] - row["currentTruth"]) for row in rows]),
    "modelCurrentError": summarize([abs(row["modelVRel"] - row["currentTruth"]) for row in rows]),
    "modelFutureError": summarize([abs(row["modelFuture"] - row["futureTruth"]) for row in rows]),
    "rawTrailingResidual": summarize([row["rawTrailingResidual"] for row in rows]),
    "trailingAge": summarize([row["trailingAge"] for row in rows]),
  }


def route_name(route_segment):
  return route_segment.rsplit("--", 1)[0]


def prediction_report(rows, candidate_values, selected, name):
  model_errors = []
  candidate_errors = []
  improvements = []
  corrections = []
  events = defaultdict(list)
  routes = defaultdict(list)
  for row, candidate, use in zip(rows, candidate_values, selected, strict=True):
    if not use:
      continue
    candidate_future = candidate + row["modelARel"] * 0.40
    model_error = abs(row["modelFuture"] - row["futureTruth"])
    candidate_error = abs(candidate_future - row["futureTruth"])
    improvement = model_error - candidate_error
    model_errors.append(model_error)
    candidate_errors.append(candidate_error)
    improvements.append(improvement)
    corrections.append(candidate - row["modelVRel"])
    events[row["event"]].append(improvement)
    routes[route_name(row["routeSegment"])].append((row["event"], improvement))

  event_improvements = [float(np.median(values)) for values in events.values()]

  def route_summary(items):
    grouped = defaultdict(list)
    for event, improvement in items:
      grouped[event].append(improvement)
    values = [float(np.median(group)) for group in grouped.values()]
    regressions = sum(value < -0.100001 for value in values)
    harmful = sum(value < -0.500001 for value in values)
    return {
      "events": len(values),
      "meanImprovement": None if not values else round(float(np.mean(values)), 4),
      "regressionsOver0_10": regressions,
      "regressionFractionOver0_10": round(regressions / max(len(values), 1), 4),
      "harmfulEventsOver0_5": harmful,
    }

  route_reports = {route: route_summary(items) for route, items in sorted(routes.items())}
  substantial_routes = [item for item in route_reports.values() if item["events"] >= 30]
  regressions = sum(value < -0.100001 for value in event_improvements)
  harmful = sum(value < -0.500001 for value in event_improvements)
  ready = bool(
    len(event_improvements) >= 100 and improvements and float(np.mean(improvements)) > 0.05 and
    percentile(candidate_errors, 95) < percentile(model_errors, 95) and
    regressions / max(len(event_improvements), 1) <= 0.01 and harmful == 0 and
    len(substantial_routes) >= 2 and all(
      item["meanImprovement"] > 0.0 and item["regressionFractionOver0_10"] <= 0.01 and
      item["harmfulEventsOver0_5"] == 0 for item in substantial_routes
    )
  )
  return {
    "name": name,
    "samples": len(improvements),
    "events": len(event_improvements),
    "coverage": round(len(improvements) / max(len(rows), 1), 4),
    "correction": summarize(np.abs(corrections)),
    "modelFutureError": summarize(model_errors),
    "candidateFutureError": summarize(candidate_errors),
    "improvement": summarize(improvements),
    "meanImprovement": None if not improvements else round(float(np.mean(improvements)), 4),
    "eventRegressionsOver0_10": regressions,
    "eventRegressionFractionOver0_10": round(regressions / max(len(event_improvements), 1), 4),
    "harmfulEventsOver0_5": harmful,
    "routes": route_reports,
    "actuationReady": ready,
  }


def evaluate_aligned_estimators(rows, validation_only=False):
  if not rows:
    return {"estimators": [], "boundedPolicies": [], "conclusion": "no_rows"}
  raw = np.asarray([row["rawCurrent"] for row in rows], dtype=np.float64)
  trailing = np.asarray([row["trailingCurrent"] for row in rows], dtype=np.float64)
  model = np.asarray([row["modelVRel"] for row in rows], dtype=np.float64)
  consensus = 0.25 * raw + 0.75 * trailing
  all_rows = np.ones(len(rows), dtype=bool)
  estimators = [
    prediction_report(rows, raw, all_rows, "aligned_raw"),
    prediction_report(rows, trailing, all_rows, "aligned_trailing_range_rate"),
    prediction_report(rows, 0.50 * raw + 0.50 * trailing, all_rows, "equal_consensus"),
    prediction_report(rows, consensus, all_rows, "trailing_weighted_consensus"),
  ]

  policies = []
  disagreement = np.abs(raw - trailing)
  disagreement_values = (0.75,) if validation_only else (0.35, 0.50, 0.75, 1.00)
  gain_values = (0.10,) if validation_only else (0.10, 0.20, 0.35, 0.50)
  correction_values = (0.10,) if validation_only else (0.10, 0.20, 0.35, 0.50)
  for source_name, radar_estimate, raw_weight in (
    ("consensus", consensus, 0.25),
    ("trailing", trailing, 0.0),
  ):
    residual = radar_estimate - model
    for max_disagreement in disagreement_values:
      for gain in gain_values:
        for max_correction in correction_values:
          selected = (disagreement <= max_disagreement) & (np.abs(residual) >= 0.20)
          correction = np.clip(gain * residual, -max_correction, max_correction)
          candidate = model + correction
          name = f"{source_name}_d{max_disagreement:.2f}_g{gain:.2f}_c{max_correction:.2f}"
          report = prediction_report(rows, candidate, selected, name)
          report["policy"] = {
            "source": source_name,
            "maxDisagreement": max_disagreement,
            "gain": gain,
            "maxCorrection": max_correction,
            "minModelResidual": 0.20,
            "rawWeight": raw_weight,
            "trailingWeight": 1.0 - raw_weight,
          }
          policies.append(report)
  policies.sort(key=lambda item: (
    not item["actuationReady"], item["eventRegressionFractionOver0_10"],
    -(item["meanImprovement"] or -math.inf), -item["samples"],
  ))
  return {
    "estimators": estimators,
    "boundedPolicies": policies,
    "conclusion": "candidate_ready" if any(item["actuationReady"] for item in policies) else "no_policy_actuation_ready",
  }


def main():
  parser = argparse.ArgumentParser(description="Analyze Carnival R0100 velocity timing and continuity")
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out", type=Path)
  parser.add_argument("--fixed-radar-lag", type=float)
  parser.add_argument("--validation-only", action="store_true")
  args = parser.parse_args()

  paths = expand(args.logs)
  samples, series, continuity = extract_samples(paths)
  if args.fixed_radar_lag is None:
    all_lags = lag_report(samples, series, stable_only=False)
    stable_lags = lag_report(samples, series, stable_only=True)
    best_lag = 0.0 if stable_lags["best"] is None else stable_lags["best"]["lag"]
  else:
    best_lag = float(args.fixed_radar_lag)
    all_lags = {"skipped": True, "fixedRadarLag": best_lag}
    stable_lags = {"skipped": True, "fixedRadarLag": best_lag}
  aligned_rows = aligned_feature_report(samples, series, best_lag)
  report = {
    "contract": "offline episode-aware qualification of a bounded model-first velocity correction",
    "files": len(paths),
    "selectedSamples": len(samples),
    "continuity": continuity,
    "allSamplesLag": all_lags,
    "stableSamplesLag": stable_lags,
    "aligned": summarize_aligned(aligned_rows),
    "fusion": evaluate_aligned_estimators(aligned_rows, args.validation_only),
  }
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.out is not None:
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(payload + "\n")


if __name__ == "__main__":
  main()
