#!/usr/bin/env python3
"""Audit whether R0100 address 0x180 slot 1 is the factory SCC lead."""

from __future__ import annotations

import argparse
import glob
import json
import re
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from analysis.evaluate_carnival_stock_scc_qualification import SccDecoder
from opendbc.car.hyundai.radar_interface import carnival_radar_object_valid, decode_carnival_radar_object
from openpilot.tools.lib.logreader import LogReader, ReadMode


MAX_OBJECT_AGE_NS = int(0.15e9)
MAX_MODEL_AGE_NS = int(0.20e9)
NO_TARGET_DISTANCE = 200.0
RADAR_TO_CAMERA = 1.52
NIS_GATE = 11.345


@dataclass(frozen=True)
class ModelLead:
  probability: float
  x: float
  y: float
  v: float
  x_std: float
  y_std: float
  v_std: float


@dataclass(frozen=True)
class ModelState:
  time: int
  v_ego: float
  leads: tuple[ModelLead, ...]
  path_x: tuple[float, ...]
  path_y: tuple[float, ...]


@dataclass(frozen=True)
class QualificationRow:
  route: str
  time: int
  track_id: int
  has_target: bool
  stable: bool
  cruise_enabled: bool
  state: int
  state_alt: int
  quality: int
  metadata: int
  lead0_qualified: bool
  lead1_qualified: bool
  d_rel: float
  y_rel: float
  v_rel: float
  model_v_ego: float
  lead0: ModelLead | None
  lead1: ModelLead | None
  path_error: float | None
  scc_distance: float
  scc_velocity: float
  primary_matches_oem: bool
  model_v_rel: float | None
  association_nis: float | None
  observed_v_rel: float | None


def false_promotion_audit(rows: list[QualificationRow], routes: set[str], predicate: Any) -> dict[str, Any]:
  eligible = [row for row in rows if row.route in routes and row.stable and row.cruise_enabled]
  future_targets: dict[tuple[str, int], list[int]] = defaultdict(list)
  for row in eligible:
    if row.has_target:
      future_targets[(row.route, row.track_id)].append(row.time)

  false_rows = [row for row in eligible if not row.has_target and predicate(row)]
  early_acquisitions = 0
  episodes = []
  current = None
  for row in false_rows:
    future = future_targets[(row.route, row.track_id)]
    becomes_target = any(row.time < target_time <= row.time + int(2.0e9) for target_time in future)
    early_acquisitions += int(becomes_target)
    if (current is None or current["route"] != row.route or current["trackId"] != row.track_id or
        row.time - current["endTime"] > int(0.15e9)):
      current = {
        "route": row.route,
        "trackId": row.track_id,
        "startTime": row.time,
        "endTime": row.time,
        "samples": 1,
        "earlyAcquisitionSamples": int(becomes_target),
        "firstSample": {
          "dRel": round(row.d_rel, 3),
          "yRel": round(row.y_rel, 3),
          "vRel": round(row.v_rel, 3),
          "pathError": round(row.path_error, 3) if row.path_error is not None else None,
          "modelVEgo": round(row.model_v_ego, 3),
          "lead0": row.lead0.__dict__ if row.lead0 is not None else None,
          "lead1": row.lead1.__dict__ if row.lead1 is not None else None,
        },
      }
      episodes.append(current)
    else:
      current["endTime"] = row.time
      current["samples"] += 1
      current["earlyAcquisitionSamples"] += int(becomes_target)
  for episode in episodes:
    episode["durationS"] = round((episode["endTime"] - episode["startTime"]) / 1e9, 3)
  return {
    "samples": len(false_rows),
    "earlyAcquisitionSamples": early_acquisitions,
    "persistentSamples": len(false_rows) - early_acquisitions,
    "episodes": episodes,
  }


def route_name(path: Path) -> str:
  if "route45-stock-scc-engagement-error" in path.parts:
    return "route45-stock-scc-engagement-error"
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)$", path.parent.name)
  return int(match.group(1)) if match else -1


def expand_paths(patterns: list[str]) -> list[Path]:
  paths: set[Path] = set()
  for pattern in patterns:
    matches = glob.glob(pattern, recursive=True)
    paths.update(Path(match).resolve() for match in (matches or [pattern]) if Path(match).is_file())
  return sorted(paths, key=lambda path: (route_name(path), segment_number(path), str(path)))


def stats(values: list[float]) -> dict[str, Any]:
  return {
    "samples": len(values),
    "p50": round(float(np.percentile(values, 50)), 4) if values else None,
    "p95": round(float(np.percentile(values, 95)), 4) if values else None,
    "p99": round(float(np.percentile(values, 99)), 4) if values else None,
    "max": round(max(values), 4) if values else None,
  }


def lead_qualifies(obj: Any, model: ModelState | None, now: int, lead_index: int) -> tuple[bool, float]:
  if model is None or now - model.time > MAX_MODEL_AGE_NS or lead_index >= len(model.leads):
    return False, float("inf")
  lead = model.leads[lead_index]
  if lead.probability <= 0.35:
    return False, float("inf")

  vision_distance = lead.x - RADAR_TO_CAMERA
  model_v_rel = lead.v - model.v_ego
  d_residual = obj.d_rel - vision_distance
  y_residual = obj.y_rel + lead.y
  v_residual = obj.v_rel - model_v_rel
  dist_sane = abs(d_residual) < max(abs(vision_distance) * 0.22, 4.0)
  lat_sane = abs(y_residual) < max(1.2, 1.5 * max(lead.y_std, 0.2))
  vel_sane = abs(v_residual) < max(4.0, 3.0 * max(lead.v_std, 0.5))
  physical_speed_sane = obj.v_rel + model.v_ego > -2.0
  model_std = np.asarray([
    np.clip(lead.x_std, 0.75, 6.0),
    np.clip(lead.y_std, 0.25, 1.5),
    np.clip(lead.v_std, 0.5, 3.0),
  ])
  radar_std = np.asarray([0.25, 0.25, 0.35])
  residual = np.asarray([d_residual, y_residual, v_residual])
  nis = float(np.sum(residual ** 2 / (model_std ** 2 + radar_std ** 2)))
  return bool(dist_sane and lat_sane and vel_sane and physical_speed_sane and nis <= NIS_GATE), nis


def model_path_error(obj: Any, model: ModelState | None, now: int) -> float | None:
  if model is None or now - model.time > MAX_MODEL_AGE_NS or len(model.path_x) < 2:
    return None
  object_x = obj.d_rel + RADAR_TO_CAMERA
  if object_x < model.path_x[0] or object_x > model.path_x[-1]:
    return None
  return abs(-obj.y_rel - float(np.interp(object_x, model.path_x, model.path_y)))


def evaluate_qualification_rule(rows: list[QualificationRow], routes: set[str], name: str,
                                predicate: Any) -> dict[str, Any]:
  selected = positives = true_positives = false_positives = 0
  samples = 0
  for row in rows:
    if row.route not in routes or not row.stable or not row.cruise_enabled:
      continue
    samples += 1
    positives += int(row.has_target)
    prediction = bool(predicate(row))
    selected += int(prediction)
    true_positives += int(prediction and row.has_target)
    false_positives += int(prediction and not row.has_target)
  return {
    "name": name,
    "samples": samples,
    "positives": positives,
    "selected": selected,
    "truePositives": true_positives,
    "falsePositives": false_positives,
    "falseNegatives": positives - true_positives,
    "precision": round(true_positives / max(selected, 1), 6),
    "recall": round(true_positives / max(positives, 1), 6),
  }


def evaluate_production_primary_contract(rows: list[QualificationRow], routes: set[str]) -> dict[str, Any]:
  """Replay the production three-frame object lifecycle plus model association."""
  samples = positives = selected = true_positives = false_positives = 0
  persistence: dict[str, int] = defaultdict(int)
  previous: dict[str, QualificationRow] = {}
  for row in sorted((candidate for candidate in rows if candidate.route in routes),
                    key=lambda candidate: (candidate.route, candidate.time)):
    qualified_primary = row.quality == 255
    prev = previous.get(row.route)
    dt = (row.time - prev.time) / 1e9 if prev is not None else float("inf")
    continuous = bool(
      prev is not None and row.track_id == prev.track_id and
      qualified_primary == (prev.quality == 255) and 0.0 <= dt <= 0.15 and
      abs(row.d_rel - prev.d_rel) <= max(1.5, 60.0 * dt) and
      abs(row.y_rel - prev.y_rel) <= max(1.0, 20.0 * dt) and
      abs(row.v_rel - prev.v_rel) <= 8.0
    )
    persistence[row.route] = persistence[row.route] + 1 if continuous else 1
    previous[row.route] = row

    if not row.stable or not row.cruise_enabled:
      continue
    prediction = bool(
      qualified_primary and persistence[row.route] >= 3 and
      (row.lead0_qualified or row.lead1_qualified)
    )
    samples += 1
    positives += int(row.has_target)
    selected += int(prediction)
    true_positives += int(prediction and row.has_target)
    false_positives += int(prediction and not row.has_target)

  return {
    "name": "production_primary_lifecycle_plus_any_model_lead",
    "samples": samples,
    "positives": positives,
    "selected": selected,
    "truePositives": true_positives,
    "falsePositives": false_positives,
    "falseNegatives": positives - true_positives,
    "precision": round(true_positives / max(selected, 1), 6),
    "recall": round(true_positives / max(positives, 1), 6),
  }


def evaluate_velocity_policy(rows: list[QualificationRow], routes: set[str], policy: dict[str, float]) -> dict[str, Any]:
  selected = correct = false_promotions = harmful = 0
  false_reasons: Counter[str] = Counter()
  false_examples: list[dict[str, Any]] = []
  model_errors: list[float] = []
  raw_errors: list[float] = []
  fused_errors: list[float] = []
  blend_errors: dict[float, list[float]] = {0.3: [], 0.5: [], 1.0: []}
  blend_harmful: Counter[float] = Counter()
  blend_harmful_examples: dict[float, list[dict[str, Any]]] = defaultdict(list)
  persistence: dict[str, int] = defaultdict(int)
  previous: dict[str, QualificationRow] = {}

  for row in sorted((candidate for candidate in rows if candidate.route in routes),
                    key=lambda candidate: (candidate.route, candidate.time)):
    prev = previous.get(row.route)
    dt = (row.time - prev.time) / 1e9 if prev is not None else float("inf")
    continuous = bool(
      prev is not None and row.track_id == prev.track_id and
      row.quality == 255 and prev.quality == 255 and 0.0 <= dt <= 0.15 and
      abs(row.d_rel - prev.d_rel) <= max(1.5, 60.0 * dt) and
      abs(row.y_rel - prev.y_rel) <= max(1.0, 20.0 * dt) and
      abs(row.v_rel - prev.v_rel) <= 4.0
    )
    persistence[row.route] = persistence[row.route] + 1 if continuous else 1
    previous[row.route] = row

    if not row.stable or row.model_v_rel is None or row.association_nis is None or row.observed_v_rel is None:
      continue
    distance_rate_residual = abs(row.v_rel - row.observed_v_rel)
    candidate_strategy = str(policy.get("candidateStrategy", "conservative"))
    if candidate_strategy == "raw":
      radar_v_rel = row.v_rel
    elif candidate_strategy == "rate":
      radar_v_rel = row.observed_v_rel
    elif candidate_strategy == "average":
      radar_v_rel = 0.5 * (row.v_rel + row.observed_v_rel)
    elif candidate_strategy == "median":
      radar_v_rel = float(np.median([row.v_rel, row.observed_v_rel, row.model_v_rel]))
    else:
      radar_v_rel = max(row.v_rel, row.observed_v_rel)
    consensus = abs(radar_v_rel - row.model_v_rel)
    prediction = bool(
      row.quality == 255 and
      persistence[row.route] >= int(policy["minPersistence"]) and
      row.d_rel <= policy["maxDistance"] and
      abs(row.y_rel) <= policy["maxAbsY"] and
      row.path_error is not None and row.path_error <= policy["maxPathError"] and
      row.association_nis <= policy["maxNis"] and
      distance_rate_residual <= policy["maxRateResidual"] and
      consensus <= policy["maxConsensus"]
    )
    if not prediction:
      continue

    selected += 1
    if not row.primary_matches_oem:
      false_promotions += 1
      distance_tolerance = max(1.0, 0.025 * max(row.scc_distance, 1.0))
      if not row.has_target:
        reason = "noOemTarget"
      elif abs(row.d_rel - row.scc_distance) > distance_tolerance:
        reason = "distanceMismatch"
      else:
        reason = "velocityMismatch"
      false_reasons[reason] += 1
      if len(false_examples) < 12:
        false_examples.append({
          "route": row.route,
          "time": row.time,
          "reason": reason,
          "radarDistance": round(row.d_rel, 3),
          "oemDistance": round(row.scc_distance, 3),
          "radarVelocity": round(row.v_rel, 3),
          "distanceRateVelocity": round(row.observed_v_rel, 3),
          "modelVelocity": round(row.model_v_rel, 3),
          "oemVelocity": round(row.scc_velocity, 3),
          "associationNis": round(row.association_nis, 3),
        })
      continue

    correct += 1
    model_error = abs(row.model_v_rel - row.scc_velocity)
    raw_error = abs(row.v_rel - row.scc_velocity)
    correction = float(np.clip(radar_v_rel - row.model_v_rel, -policy["maxCorrection"], policy["maxCorrection"]))
    closing_full_authority = (
      policy.get("blendMode") == "closingFull" and row.model_v_rel <= 0.0 and radar_v_rel <= 0.0
    )
    effective_blend_weight = 1.0 if closing_full_authority else policy["blendWeight"]
    fused_v_rel = row.model_v_rel + effective_blend_weight * correction
    fused_error = abs(fused_v_rel - row.scc_velocity)
    model_errors.append(model_error)
    raw_errors.append(raw_error)
    fused_errors.append(fused_error)
    harmful += int(fused_error - model_error > 0.5)
    for weight, errors in blend_errors.items():
      alternative_error = abs(row.model_v_rel + weight * correction - row.scc_velocity)
      errors.append(alternative_error)
      is_harmful = alternative_error - model_error > 0.5
      blend_harmful[weight] += int(is_harmful)
      if is_harmful and len(blend_harmful_examples[weight]) < 100:
        blend_harmful_examples[weight].append({
          "route": row.route,
          "time": row.time,
          "distance": round(row.d_rel, 3),
          "lateral": round(row.y_rel, 3),
          "radarVelocity": round(row.v_rel, 3),
          "distanceRateVelocity": round(row.observed_v_rel, 3),
          "candidateVelocity": round(radar_v_rel, 3),
          "modelVelocity": round(row.model_v_rel, 3),
          "oemVelocity": round(row.scc_velocity, 3),
          "modelError": round(model_error, 3),
          "candidateError": round(alternative_error, 3),
          "associationNis": round(row.association_nis, 3),
          "pathError": None if row.path_error is None else round(row.path_error, 3),
        })

  return {
    "selected": selected,
    "correct": correct,
    "falsePromotions": false_promotions,
    "falseReasons": dict(false_reasons),
    "falseExamples": false_examples,
    "precision": round(correct / max(selected, 1), 6),
    "modelError": stats(model_errors),
    "rawRadarError": stats(raw_errors),
    "fusedError": stats(fused_errors),
    "harmfulSamplesOver0_5": harmful,
    "harmfulFraction": round(harmful / max(correct, 1), 6),
    "blendAlternatives": {
      str(weight): {
        "error": stats(errors),
        "harmfulSamplesOver0_5": blend_harmful[weight],
        "harmfulFraction": round(blend_harmful[weight] / max(correct, 1), 6),
        "harmfulExamples": blend_harmful_examples[weight],
      }
      for weight, errors in blend_errors.items()
    },
  }


def velocity_policy_sweep(rows: list[QualificationRow], routes: list[str]) -> dict[str, Any]:
  policies = []
  for min_persistence in (8,):
    for max_distance in (40.0, 60.0):
      for max_abs_y in (0.75, 1.0):
        for max_path_error in (0.35, 0.5):
          for max_nis in (3.0, 5.991):
            for max_rate_residual in (0.25, 0.5, 0.75):
              for max_consensus in (0.25, 0.5, 0.75):
                for blend_weight in (0.2, 0.3, 0.5):
                  policy = {
                    "candidateStrategy": "conservative",
                    "blendMode": "fixed",
                    "minPersistence": min_persistence,
                    "maxDistance": max_distance,
                    "maxAbsY": max_abs_y,
                    "maxPathError": max_path_error,
                    "maxNis": max_nis,
                    "maxRateResidual": max_rate_residual,
                    "maxConsensus": max_consensus,
                    "maxCorrection": max_consensus,
                    "blendWeight": blend_weight,
                  }
                  folds = []
                  for holdout in routes:
                    result = evaluate_velocity_policy(rows, {holdout}, policy)
                    folds.append({"route": holdout, **result})
                  aggregate = evaluate_velocity_policy(rows, set(routes), policy)
                  positive_folds = [fold for fold in folds if fold["correct"] >= 100]
                  negative_fold_selections = sum(fold["selected"] for fold in folds if fold["correct"] == 0)
                  fold_improvements = [
                    fold["modelError"]["p95"] - fold["fusedError"]["p95"]
                    for fold in positive_folds
                    if fold["modelError"]["p95"] is not None and fold["fusedError"]["p95"] is not None
                  ]
                  ready = bool(
                    aggregate["correct"] >= 500 and aggregate["harmfulSamplesOver0_5"] == 0 and
                    len(fold_improvements) == len(positive_folds) and len(positive_folds) >= 2 and
                    min(fold_improvements) >= 0.0 and negative_fold_selections == 0
                  )
                  improvement = (
                    (aggregate["modelError"]["p95"] or 0.0) - (aggregate["fusedError"]["p95"] or 0.0)
                  )
                  worst_fold_improvement = min(fold_improvements, default=-10.0)
                  score = (
                    int(ready) * 1_000_000 + worst_fold_improvement * 10_000 + improvement * 1000 +
                    aggregate["correct"] * 0.001 - aggregate["harmfulSamplesOver0_5"] * 10_000
                  )
                  policies.append((score, ready, policy, aggregate, folds))

  best = max(policies, key=lambda item: item[0])
  _, ready, policy, aggregate, folds = best
  return {
    "actuationReady": ready,
    "policy": policy,
    "aggregate": aggregate,
    "folds": folds,
    "evaluatedPolicies": len(policies),
  }


def velocity_policy_report(rows: list[QualificationRow], routes: list[str], policy: dict[str, float]) -> dict[str, Any]:
  folds = [
    {"route": route, **evaluate_velocity_policy(rows, {route}, policy)}
    for route in routes
  ]
  aggregate = evaluate_velocity_policy(rows, set(routes), policy)
  positive_folds = [fold for fold in folds if fold["correct"] >= 100]
  negative_fold_selections = sum(fold["selected"] for fold in folds if fold["correct"] == 0)
  fold_improvements = [
    fold["modelError"]["p95"] - fold["fusedError"]["p95"]
    for fold in positive_folds
    if fold["modelError"]["p95"] is not None and fold["fusedError"]["p95"] is not None
  ]
  ready = bool(
    aggregate["correct"] >= 500 and aggregate["harmfulSamplesOver0_5"] == 0 and
    len(fold_improvements) == len(positive_folds) and len(positive_folds) >= 2 and
    min(fold_improvements) >= 0.0 and negative_fold_selections == 0
  )
  return {
    "actuationReady": ready,
    "policy": policy,
    "aggregate": aggregate,
    "folds": folds,
    "evaluatedPolicies": 1,
  }


def analyze(paths: list[Path], velocity_policy: dict[str, float] | None = None) -> dict[str, Any]:
  decoder = SccDecoder()
  current_route = ""
  primary: tuple[int, Any] | None = None
  counts: dict[str, Counter[str]] = defaultdict(Counter)
  distance_errors: dict[str, list[float]] = defaultdict(list)
  velocity_errors: dict[str, list[float]] = defaultdict(list)
  target_states: dict[str, Counter[str]] = defaultdict(Counter)
  no_target_states: dict[str, Counter[str]] = defaultdict(Counter)
  target_quality: dict[str, list[float]] = defaultdict(list)
  no_target_quality: dict[str, list[float]] = defaultdict(list)
  matched_states: dict[str, Counter[str]] = defaultdict(Counter)
  rejected_states: dict[str, Counter[str]] = defaultdict(Counter)
  matched_quality: dict[str, Counter[int]] = defaultdict(Counter)
  rejected_quality: dict[str, Counter[int]] = defaultdict(Counter)
  association_counts: dict[str, Counter[str]] = defaultdict(Counter)
  association_nis: dict[str, list[float]] = defaultdict(list)
  latest_model: ModelState | None = None
  qualification_rows: list[QualificationRow] = []
  runtime_velocity_rows: list[QualificationRow] = []
  primary_distance_history: dict[int, deque[tuple[int, float]]] = defaultdict(lambda: deque(maxlen=8))
  runtime_distance_history: dict[int, deque[tuple[int, float]]] = defaultdict(lambda: deque(maxlen=8))
  runtime_previous_primary: tuple[int, Any] | None = None
  latest_scc: tuple[int, float, float] | None = None
  runtime_target_status: bool | None = None
  runtime_target_status_frames = 0
  target_status: bool | None = None
  target_status_frames = 0
  last_qualification_key: tuple[int, bool] | None = None
  cruise_enabled = False

  for path in paths:
    route = route_name(path)
    if route != current_route:
      primary = None
      latest_model = None
      primary_distance_history.clear()
      runtime_distance_history.clear()
      runtime_previous_primary = None
      latest_scc = None
      runtime_target_status = None
      runtime_target_status_frames = 0
      target_status = None
      target_status_frames = 0
      last_qualification_key = None
      cruise_enabled = False
      current_route = route

    for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
      if msg.which() == "modelV2":
        model = msg.modelV2
        leads = tuple(ModelLead(
          probability=float(model.leadsV3[index].prob),
          x=float(model.leadsV3[index].x[0]),
          y=float(model.leadsV3[index].y[0]),
          v=float(model.leadsV3[index].v[0]),
          x_std=float(model.leadsV3[index].xStd[0]),
          y_std=float(model.leadsV3[index].yStd[0]),
          v_std=float(model.leadsV3[index].vStd[0]),
        ) for index in range(min(2, len(model.leadsV3))))
        latest_model = ModelState(
          int(msg.logMonoTime),
          float(model.velocity.x[0]),
          leads,
          tuple(float(value) for value in model.position.x),
          tuple(float(value) for value in model.position.y),
        )
        now = int(msg.logMonoTime)
        if (primary is not None and latest_scc is not None and
            0 <= now - primary[0] <= MAX_OBJECT_AGE_NS and
            0 <= now - latest_scc[0] <= MAX_OBJECT_AGE_NS):
          obj = primary[1]
          _, scc_distance, scc_velocity = latest_scc
          has_target = scc_distance < NO_TARGET_DISTANCE
          if has_target == runtime_target_status:
            runtime_target_status_frames += 1
          else:
            runtime_target_status = has_target
            runtime_target_status_frames = 1

          qualification_results = [lead_qualifies(obj, latest_model, now, lead_index) for lead_index in range(2)]
          qualifies = [result[0] for result in qualification_results]
          qualified_indices = [index for index, result in enumerate(qualification_results) if result[0]]
          best_lead_index = min(qualified_indices, key=lambda index: qualification_results[index][1]) if qualified_indices else None
          model_v_rel = None
          row_association_nis = None
          if best_lead_index is not None:
            model_v_rel = latest_model.leads[best_lead_index].v - latest_model.v_ego
            row_association_nis = qualification_results[best_lead_index][1]

          primary_time = primary[0]
          previous_time, previous_obj = runtime_previous_primary if runtime_previous_primary is not None else (0, None)
          continuous = bool(
            previous_obj is not None and obj.raw_track_id == previous_obj.raw_track_id and
            0 <= primary_time - previous_time <= MAX_OBJECT_AGE_NS and
            abs(obj.d_rel - previous_obj.d_rel) <= max(1.5, 60.0 * (primary_time - previous_time) / 1e9) and
            abs(obj.y_rel - previous_obj.y_rel) <= 1.0 and
            abs(obj.v_rel - previous_obj.v_rel) <= 4.0
          )
          history = runtime_distance_history[obj.raw_track_id]
          if not continuous:
            history.clear()
          if not history or history[-1][0] != primary_time:
            history.append((primary_time, obj.d_rel))
          observed_v_rel = None
          if len(history) >= 8:
            times = np.asarray([(sample_time - history[0][0]) / 1e9 for sample_time, _ in history], dtype=float)
            distances = np.asarray([distance for _, distance in history], dtype=float)
            if times[-1] - times[0] >= 0.15:
              observed_v_rel = float(np.polyfit(times, distances, 1)[0])

          distance_error = abs(obj.d_rel - scc_distance)
          velocity_error = abs(obj.v_rel - scc_velocity)
          row = QualificationRow(
            route=route,
            time=now,
            track_id=obj.raw_track_id,
            has_target=has_target,
            stable=runtime_target_status_frames >= 12,
            cruise_enabled=cruise_enabled,
            state=obj.state,
            state_alt=obj.state_alt,
            quality=obj.quality_byte,
            metadata=obj.metadata_50_63,
            lead0_qualified=qualifies[0],
            lead1_qualified=qualifies[1],
            d_rel=obj.d_rel,
            y_rel=obj.y_rel,
            v_rel=obj.v_rel,
            model_v_ego=latest_model.v_ego,
            lead0=latest_model.leads[0] if len(latest_model.leads) > 0 else None,
            lead1=latest_model.leads[1] if len(latest_model.leads) > 1 else None,
            path_error=model_path_error(obj, latest_model, now),
            scc_distance=scc_distance,
            scc_velocity=scc_velocity,
            primary_matches_oem=bool(
              has_target and distance_error <= max(1.0, 0.025 * max(scc_distance, 1.0)) and velocity_error <= 1.25
            ),
            model_v_rel=model_v_rel,
            association_nis=row_association_nis,
            observed_v_rel=observed_v_rel,
          )
          runtime_velocity_rows.append(row)
          runtime_previous_primary = (primary_time, obj)
        continue
      if msg.which() == "carState":
        cruise_enabled = bool(msg.carState.cruiseState.enabled)
        continue
      if msg.which() != "can":
        continue
      now = int(msg.logMonoTime)
      frames = [(int(frame.address), bytes(frame.dat), int(frame.src)) for frame in msg.can]

      for address, dat, src in frames:
        if address == 0x180 and src == 1 and len(dat) == 32:
          obj = decode_carnival_radar_object(dat, 0)
          primary = (now, obj) if carnival_radar_object_valid(obj) else None

      for address, dat, src in frames:
        if address != 0x1A0 or src != 0 or len(dat) != 32:
          continue
        scc = decoder.decode(dat)
        scc_distance = float(scc["ACC_ObjDist"])
        scc_velocity = float(scc["ACC_ObjRelSpd"])
        latest_scc = (now, scc_distance, scc_velocity)
        has_target = scc_distance < NO_TARGET_DISTANCE
        if has_target == target_status:
          target_status_frames += 1
        else:
          target_status = has_target
          target_status_frames = 1
        prefix = "target" if has_target else "noTarget"
        counts[route][f"{prefix}Frames"] += 1

        fresh = primary is not None and 0 <= now - primary[0] <= MAX_OBJECT_AGE_NS
        counts[route][f"{prefix}PrimaryFresh"] += int(fresh)
        if not fresh:
          continue

        obj = primary[1]
        qualification_results = [lead_qualifies(obj, latest_model, now, lead_index) for lead_index in range(2)]
        qualifies = [result[0] for result in qualification_results]
        path_error = model_path_error(obj, latest_model, now)
        qualified_indices = [index for index, result in enumerate(qualification_results) if result[0]]
        best_lead_index = min(qualified_indices, key=lambda index: qualification_results[index][1]) if qualified_indices else None
        model_v_rel = None
        row_association_nis = None
        if best_lead_index is not None and latest_model is not None:
          model_v_rel = latest_model.leads[best_lead_index].v - latest_model.v_ego
          row_association_nis = qualification_results[best_lead_index][1]
        distance_error = abs(obj.d_rel - scc_distance)
        velocity_error = abs(obj.v_rel - scc_velocity)
        primary_matches_oem = bool(
          has_target and distance_error <= max(1.0, 0.025 * max(scc_distance, 1.0)) and velocity_error <= 1.25
        )
        qualification_key = (primary[0], has_target)
        if qualification_key != last_qualification_key:
          history = primary_distance_history[obj.raw_track_id]
          history.append((primary[0], obj.d_rel))
          observed_v_rel = None
          if len(history) >= 8:
            times = np.asarray([(sample_time - history[0][0]) / 1e9 for sample_time, _ in history], dtype=float)
            distances = np.asarray([distance for _, distance in history], dtype=float)
            if times[-1] - times[0] >= 0.15:
              observed_v_rel = float(np.polyfit(times, distances, 1)[0])
          qualification_rows.append(QualificationRow(
            route=route,
            time=now,
            track_id=obj.raw_track_id,
            has_target=has_target,
            stable=target_status_frames >= 30,
            cruise_enabled=cruise_enabled,
            state=obj.state,
            state_alt=obj.state_alt,
            quality=obj.quality_byte,
            metadata=obj.metadata_50_63,
            lead0_qualified=qualifies[0],
            lead1_qualified=qualifies[1],
            d_rel=obj.d_rel,
            y_rel=obj.y_rel,
            v_rel=obj.v_rel,
            model_v_ego=latest_model.v_ego if latest_model is not None else 0.0,
            lead0=latest_model.leads[0] if latest_model is not None and len(latest_model.leads) > 0 else None,
            lead1=latest_model.leads[1] if latest_model is not None and len(latest_model.leads) > 1 else None,
            path_error=path_error,
            scc_distance=scc_distance,
            scc_velocity=scc_velocity,
            primary_matches_oem=primary_matches_oem,
            model_v_rel=model_v_rel,
            association_nis=row_association_nis,
            observed_v_rel=observed_v_rel,
          ))
          last_qualification_key = qualification_key
        state_key = f"{obj.state}/{obj.state_alt}"
        if has_target:
          target_states[route][state_key] += 1
          target_quality[route].append(float(obj.quality_byte))
          distance_errors[route].append(distance_error)
          velocity_errors[route].append(velocity_error)
          matched = primary_matches_oem
          counts[route]["targetPrimaryMatched"] += int(matched)
          (matched_states if matched else rejected_states)[route][state_key] += 1
          (matched_quality if matched else rejected_quality)[route][obj.quality_byte] += 1
        else:
          no_target_states[route][state_key] += 1
          no_target_quality[route].append(float(obj.quality_byte))
          rejected_states[route][state_key] += 1
          rejected_quality[route][obj.quality_byte] += 1

        for accepted, nis in qualification_results:
          if accepted:
            association_nis[route].append(nis)
        label = bool(has_target and matched)
        prediction = any(qualifies)
        association_counts[route]["samples"] += 1
        association_counts[route]["positives"] += int(label)
        association_counts[route]["predictions"] += int(prediction)
        association_counts[route]["truePositives"] += int(prediction and label)
        association_counts[route]["falsePositives"] += int(prediction and not label)
        association_counts[route]["falseNegatives"] += int(not prediction and label)
        association_counts[route]["lead0Predictions"] += int(qualifies[0])
        association_counts[route]["lead1Predictions"] += int(qualifies[1])

  routes = sorted(counts)
  train_routes = {routes[0]} if routes else set()
  holdout_routes = set(routes[1:]) or set(train_routes)
  rule_specs = [
    ("quality_eq_255", lambda row: row.quality == 255),
    ("state4_quality_eq_255", lambda row: row.state == 4 and row.quality == 255),
    ("state4_quality_ge_250", lambda row: row.state == 4 and row.quality >= 250),
    ("state4_quality_ge_240", lambda row: row.state == 4 and row.quality >= 240),
    ("state34_quality_eq_255", lambda row: row.state in (3, 4) and row.quality == 255),
    ("state4", lambda row: row.state == 4),
    ("quality_eq_255_lead0", lambda row: row.quality == 255 and row.lead0_qualified),
    ("quality_eq_255_any_lead", lambda row: row.quality == 255 and (row.lead0_qualified or row.lead1_qualified)),
    ("state4_quality_eq_255_lead0", lambda row: row.state == 4 and row.quality == 255 and row.lead0_qualified),
    ("state4_quality_eq_255_any_lead", lambda row: row.state == 4 and row.quality == 255 and
     (row.lead0_qualified or row.lead1_qualified)),
    ("quality_eq_255_any_lead_path_0_5", lambda row: row.quality == 255 and
     (row.lead0_qualified or row.lead1_qualified) and row.path_error is not None and row.path_error <= 0.5),
    ("quality_eq_255_any_lead_path_0_75", lambda row: row.quality == 255 and
     (row.lead0_qualified or row.lead1_qualified) and row.path_error is not None and row.path_error <= 0.75),
    ("quality_eq_255_any_lead_path_1_0", lambda row: row.quality == 255 and
     (row.lead0_qualified or row.lead1_qualified) and row.path_error is not None and row.path_error <= 1.0),
    ("quality_eq_255_any_lead_path_1_5", lambda row: row.quality == 255 and
     (row.lead0_qualified or row.lead1_qualified) and row.path_error is not None and row.path_error <= 1.5),
    ("quality_eq_255_any_lead_path_2_0", lambda row: row.quality == 255 and
     (row.lead0_qualified or row.lead1_qualified) and row.path_error is not None and row.path_error <= 2.0),
  ]
  qualification_rules = []
  for name, predicate in rule_specs:
    qualification_rules.append({
      "name": name,
      "training": evaluate_qualification_rule(qualification_rows, train_routes, name, predicate),
      "holdout": evaluate_qualification_rule(qualification_rows, holdout_routes, name, predicate),
    })
  def selected_rule(row: QualificationRow) -> bool:
    return row.quality == 255 and (row.lead0_qualified or row.lead1_qualified)
  selected_false_promotion_audit = {
    "training": false_promotion_audit(qualification_rows, train_routes, selected_rule),
    "holdout": false_promotion_audit(qualification_rows, holdout_routes, selected_rule),
  }
  route_reports = {}
  for route in routes:
    route_counts = counts[route]
    target_frames = route_counts["targetFrames"]
    no_target_frames = route_counts["noTargetFrames"]
    route_reports[route] = {
      "counts": dict(route_counts),
      "targetPrimaryAvailability": round(route_counts["targetPrimaryFresh"] / max(target_frames, 1), 6),
      "targetPrimaryMatchRecall": round(route_counts["targetPrimaryMatched"] / max(target_frames, 1), 6),
      "noTargetPrimaryPresence": round(route_counts["noTargetPrimaryFresh"] / max(no_target_frames, 1), 6),
      "distanceError": stats(distance_errors[route]),
      "velocityError": stats(velocity_errors[route]),
      "targetState": dict(target_states[route].most_common()),
      "noTargetState": dict(no_target_states[route].most_common()),
      "targetQuality": stats(target_quality[route]),
      "noTargetQuality": stats(no_target_quality[route]),
      "matchedState": dict(matched_states[route].most_common()),
      "rejectedState": dict(rejected_states[route].most_common()),
      "matchedQuality": {str(key): value for key, value in matched_quality[route].most_common()},
      "rejectedQuality": {str(key): value for key, value in rejected_quality[route].most_common()},
      "productionAssociation": {
        **dict(association_counts[route]),
        "precision": round(association_counts[route]["truePositives"] / max(association_counts[route]["predictions"], 1), 6),
        "recall": round(association_counts[route]["truePositives"] / max(association_counts[route]["positives"], 1), 6),
        "acceptedNis": stats(association_nis[route]),
      },
    }

  total = sum(counts.values(), Counter())
  target_frames = total["targetFrames"]
  no_target_frames = total["noTargetFrames"]
  return {
    "files": len(paths),
    "trainingRoutes": sorted(train_routes),
    "holdoutRoutes": sorted(holdout_routes),
    "qualificationRules": qualification_rules,
    "productionPrimaryContract": {
      "training": evaluate_production_primary_contract(qualification_rows, train_routes),
      "holdout": evaluate_production_primary_contract(qualification_rows, holdout_routes),
    },
    "velocityControlQualification": (
      velocity_policy_report(runtime_velocity_rows, routes, velocity_policy)
      if velocity_policy is not None else velocity_policy_sweep(runtime_velocity_rows, routes)
    ),
    "runtimeVelocityRows": len(runtime_velocity_rows),
    "selectedFalsePromotionAudit": selected_false_promotion_audit,
    "routes": route_reports,
    "all": {
      "counts": dict(total),
      "targetPrimaryAvailability": round(total["targetPrimaryFresh"] / max(target_frames, 1), 6),
      "targetPrimaryMatchRecall": round(total["targetPrimaryMatched"] / max(target_frames, 1), 6),
      "noTargetPrimaryPresence": round(total["noTargetPrimaryFresh"] / max(no_target_frames, 1), 6),
      "distanceError": stats(sum(distance_errors.values(), [])),
      "velocityError": stats(sum(velocity_errors.values(), [])),
      "targetState": dict(sum(target_states.values(), Counter()).most_common()),
      "noTargetState": dict(sum(no_target_states.values(), Counter()).most_common()),
      "targetQuality": stats(sum(target_quality.values(), [])),
      "noTargetQuality": stats(sum(no_target_quality.values(), [])),
      "matchedState": dict(sum(matched_states.values(), Counter()).most_common()),
      "rejectedState": dict(sum(rejected_states.values(), Counter()).most_common()),
      "matchedQuality": {str(key): value for key, value in sum(matched_quality.values(), Counter()).most_common()},
      "rejectedQuality": {str(key): value for key, value in sum(rejected_quality.values(), Counter()).most_common()},
      "productionAssociation": {
        **dict(sum(association_counts.values(), Counter())),
        "precision": round(
          sum(association_counts.values(), Counter())["truePositives"] /
          max(sum(association_counts.values(), Counter())["predictions"], 1), 6,
        ),
        "recall": round(
          sum(association_counts.values(), Counter())["truePositives"] /
          max(sum(association_counts.values(), Counter())["positives"], 1), 6,
        ),
        "acceptedNis": stats(sum(association_nis.values(), [])),
      },
    },
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("logs", nargs="+")
  parser.add_argument("--out", type=Path)
  parser.add_argument("--velocity-policy", type=json.loads,
                      help="Evaluate one JSON policy instead of sweeping the policy grid")
  parser.add_argument("--velocity-policy-selected", action="store_true",
                      help="Evaluate the current conservative selected policy only")
  parser.add_argument("--velocity-blend-weight", type=float, default=0.3,
                      help="Radar correction weight for --velocity-policy-selected")
  parser.add_argument("--velocity-candidate-strategy", choices=("conservative", "raw", "rate", "average", "median"),
                      default="conservative", help="Velocity estimate used by --velocity-policy-selected")
  parser.add_argument("--velocity-blend-mode", choices=("fixed", "closingFull"), default="fixed",
                      help="Use full radar authority only while both estimates are closing")
  parser.add_argument("--velocity-max-consensus", type=float, default=0.75,
                      help="Maximum radar/model velocity disagreement for the selected policy")
  parser.add_argument("--velocity-max-rate-residual", type=float, default=0.75,
                      help="Maximum raw velocity/distance-rate disagreement for the selected policy")
  parser.add_argument("--velocity-min-persistence", type=int, default=8,
                      help="Minimum continuous primary-track frames for the selected policy")
  args = parser.parse_args()
  selected_policy = {
    "candidateStrategy": args.velocity_candidate_strategy,
    "blendMode": args.velocity_blend_mode,
    "blendWeight": args.velocity_blend_weight,
    "maxAbsY": 0.75,
    "maxConsensus": args.velocity_max_consensus,
    "maxCorrection": args.velocity_max_consensus,
    "maxDistance": 40.0,
    "maxNis": 5.991,
    "maxPathError": 0.35,
    "maxRateResidual": args.velocity_max_rate_residual,
    "minPersistence": args.velocity_min_persistence,
  } if args.velocity_policy_selected else args.velocity_policy
  report = analyze(expand_paths(args.logs), selected_policy)
  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if args.out:
    args.out.write_text(output + "\n")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
