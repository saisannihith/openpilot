#!/usr/bin/env python3
"""Held-out acceptance test for Carnival speed-dependent torque calibration.

This intentionally mirrors torqued's total-least-squares fit and torque-bucket
balancing. It never changes vehicle parameters; its output is an actuation gate.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


CAR_FINGERPRINT = "KIA_CARNIVAL_4TH_GEN"
SPEED_BOUNDS = np.array([5.0, 8.0, 12.0, 18.0, 24.0, 29.0, 35.0])
SPEED_CENTERS = np.array([6.5, 10.0, 15.0, 21.0, 26.5, 32.0])
TORQUE_BOUNDS = np.array([-0.5, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.5])
FRICTION_FACTOR = 1.5
MIN_CONTINUOUS_CONTEXT_SECONDS = 2.0
MAX_SAMPLE_GAP_SECONDS = 0.035
MAX_POINTS_PER_ROUTE_TORQUE_BUCKET = 300
MIN_FIT_POINTS = 500

# A new control dimension needs a material held-out gain, stable tails, and no
# recent-route regressions. Be stricter than ordinary parameter-fit significance.
MIN_ROUTES = 20
MIN_USABLE_MINUTES = 300.0
MIN_WEIGHTED_MAE_IMPROVEMENT_PCT = 3.0
MIN_LOW_SPEED_MAE_IMPROVEMENT_PCT = 3.0
MIN_WEIGHTED_P95_IMPROVEMENT_PCT = 1.0
MAX_ROUTE_REGRESSION_PCT = 3.0
MAX_RECENT_ROUTE_REGRESSION_PCT = 2.0
MAX_ADJACENT_FACTOR_RATIO = 1.25


def _require_columns(frame: pd.DataFrame) -> None:
  required = {
    "route", "timestamp", "v_ego", "applied_torque", "lateral_delay",
    "pose_valid", "pose_lateral_accel_tp03", "pose_lateral_accel_tp06",
  }
  missing = sorted(required - set(frame.columns))
  if missing:
    raise ValueError(f"dataset is missing required columns: {', '.join(missing)}")


def prepare_samples(frame: pd.DataFrame) -> pd.DataFrame:
  """Pair applied torque with delayed physical yaw response and enforce context."""
  _require_columns(frame)
  data = frame.sort_values(["route", "timestamp"]).copy()
  delay_alpha = np.clip((data["lateral_delay"].to_numpy(float) - 0.3) / 0.3, 0.0, 1.0)
  data["response"] = (
    data["pose_lateral_accel_tp03"].to_numpy(float) * (1.0 - delay_alpha) +
    data["pose_lateral_accel_tp06"].to_numpy(float) * delay_alpha
  )
  # torqued stores the sign opposite to the normalized actuator command.
  data["steer"] = -data["applied_torque"].to_numpy(float)

  sample_gap = data.groupby("route", sort=False)["timestamp"].diff()
  run_break = sample_gap.isna() | (sample_gap > MAX_SAMPLE_GAP_SECONDS)
  data["clean_run"] = run_break.groupby(data["route"], sort=False).cumsum()
  run_start = data.groupby(["route", "clean_run"], sort=False)["timestamp"].transform("min")
  data["clean_context_age"] = data["timestamp"] - run_start

  valid = (
    data["pose_valid"].astype(bool) &
    np.isfinite(data["steer"]) & np.isfinite(data["response"]) &
    (data["clean_context_age"] >= MIN_CONTINUOUS_CONTEXT_SECONDS) &
    data["v_ego"].between(SPEED_BOUNDS[0], SPEED_BOUNDS[-1], inclusive="left") &
    data["steer"].abs().between(0.02, TORQUE_BOUNDS[-1], inclusive="both") &
    (data["response"].abs() <= 1.0)
  )
  return data.loc[valid].reset_index(drop=True)


def balanced_points(frame: pd.DataFrame) -> np.ndarray:
  """Deterministically balance routes and steer buckets before a TLS fit."""
  if frame.empty:
    return np.empty((0, 2))
  data = frame.sort_values(["route", "timestamp"]).copy()
  data["torque_bucket"] = np.digitize(data["steer"], TORQUE_BOUNDS[1:-1], right=False)
  groups = []
  for _, group in data.groupby(["route", "torque_bucket"], sort=True, observed=True):
    if len(group) > MAX_POINTS_PER_ROUTE_TORQUE_BUCKET:
      indexes = np.linspace(0, len(group) - 1, MAX_POINTS_PER_ROUTE_TORQUE_BUCKET, dtype=int)
      group = group.iloc[indexes]
    groups.append(group[["steer", "response"]].to_numpy(float))
  return np.concatenate(groups) if groups else np.empty((0, 2))


def fit_tls(frame: pd.DataFrame) -> dict[str, float | int]:
  points = balanced_points(frame)
  if len(points) < MIN_FIT_POINTS:
    raise ValueError(f"insufficient balanced points: {len(points)} < {MIN_FIT_POINTS}")
  design = np.column_stack((points[:, 0], np.ones(len(points)), points[:, 1]))
  _, _, vectors = np.linalg.svd(design, full_matrices=False)
  slope, offset = -vectors[-1, :2] / vectors[-1, 2]
  if not np.isfinite(slope) or not np.isfinite(offset) or slope <= 0.0:
    raise ValueError(f"invalid TLS fit: slope={slope}, offset={offset}")

  sine = np.sqrt(slope ** 2 / (slope ** 2 + 1.0))
  cosine = np.sqrt(1.0 / (slope ** 2 + 1.0))
  rotation = np.array([[cosine, -sine], [sine, cosine]])
  spread = (points @ rotation)[:, 1]
  return {
    "latAccelFactor": float(slope),
    "latAccelOffset": float(offset),
    "friction": float(np.std(spread) * FRICTION_FACTOR),
    "points": int(len(points)),
  }


def fit_speed_profile(frame: pd.DataFrame) -> list[dict[str, float | int]]:
  profile = []
  for lower, upper in zip(SPEED_BOUNDS[:-1], SPEED_BOUNDS[1:], strict=True):
    profile.append(fit_tls(frame[frame["v_ego"].between(lower, upper, inclusive="left")]))
  return profile


def _orthogonal_errors(frame: pd.DataFrame, factors: np.ndarray, offsets: np.ndarray) -> np.ndarray:
  residual = frame["response"].to_numpy(float) - (
    factors * frame["steer"].to_numpy(float) + offsets
  )
  return np.abs(residual) / np.sqrt(factors ** 2 + 1.0)


def score_holdout(frame: pd.DataFrame, global_fit: dict, speed_profile: list[dict]) -> dict[str, float | int]:
  global_factors = np.full(len(frame), float(global_fit["latAccelFactor"]))
  global_offsets = np.full(len(frame), float(global_fit["latAccelOffset"]))
  speed_factors = np.interp(
    frame["v_ego"], SPEED_CENTERS, [fit["latAccelFactor"] for fit in speed_profile],
  )
  speed_offsets = np.interp(
    frame["v_ego"], SPEED_CENTERS, [fit["latAccelOffset"] for fit in speed_profile],
  )
  baseline = _orthogonal_errors(frame, global_factors, global_offsets)
  candidate = _orthogonal_errors(frame, speed_factors, speed_offsets)

  def improvement(left: np.ndarray, right: np.ndarray) -> float:
    return float(100.0 * (left.mean() - right.mean()) / max(left.mean(), 1e-9))

  low_speed = frame["v_ego"].to_numpy(float) < 15.0
  return {
    "rows": int(len(frame)),
    "maeBaseline": float(baseline.mean()),
    "maeCandidate": float(candidate.mean()),
    "maeImprovementPct": improvement(baseline, candidate),
    "lowSpeedRows": int(low_speed.sum()),
    "lowSpeedMaeImprovementPct": improvement(baseline[low_speed], candidate[low_speed]) if low_speed.any() else 0.0,
    "p95Baseline": float(np.quantile(baseline, 0.95)),
    "p95Candidate": float(np.quantile(candidate, 0.95)),
    "p95ImprovementPct": float(100.0 * (np.quantile(baseline, 0.95) - np.quantile(candidate, 0.95)) /
                               max(np.quantile(baseline, 0.95), 1e-9)),
  }


def _weighted(reports: list[dict], key: str, weight_key: str = "rows") -> float:
  total = sum(int(report[weight_key]) for report in reports)
  return float(sum(int(report[weight_key]) * float(report[key]) for report in reports) / max(total, 1))


def evaluate(frame: pd.DataFrame, source_laf: float = 1.63) -> dict:
  samples = prepare_samples(frame)
  routes = sorted(samples["route"].unique())
  # Route logMonoTime resets between boots, so it cannot order separate routes.
  # Logger route identifiers are monotonic and zero-padded on this device.
  recent_routes = set(routes[-3:])
  holdouts = []

  for route in routes:
    train = samples[samples["route"] != route]
    heldout = samples[samples["route"] == route]
    try:
      # This is the fair baseline: current torqued learns one factor above 15 m/s.
      global_fit = fit_tls(train[train["v_ego"] > 15.0])
      speed_profile = fit_speed_profile(train)
    except ValueError as error:
      holdouts.append({"route": route, "rows": int(len(heldout)), "error": str(error)})
      continue
    report = score_holdout(heldout, global_fit, speed_profile)
    report["route"] = route
    report["recent"] = route in recent_routes
    holdouts.append(report)

  scored = [report for report in holdouts if "error" not in report]
  if not scored:
    raise ValueError("no route produced a valid held-out score")

  full_global = fit_tls(samples[samples["v_ego"] > 15.0])
  full_profile = fit_speed_profile(samples)
  factors = [float(fit["latAccelFactor"]) for fit in full_profile]
  adjacent_ratio = max(max(a, b) / min(a, b) for a, b in zip(factors[:-1], factors[1:], strict=True))
  usable_minutes = len(samples) / 100.0 / 60.0
  weighted_mae = _weighted(scored, "maeImprovementPct")
  low_reports = [report for report in scored if int(report["lowSpeedRows"]) > 0]
  weighted_low = _weighted(low_reports, "lowSpeedMaeImprovementPct", "lowSpeedRows") if low_reports else 0.0
  weighted_p95 = _weighted(scored, "p95ImprovementPct")
  worst_route = min(float(report["maeImprovementPct"]) for report in scored)
  worst_recent = min(float(report["maeImprovementPct"]) for report in scored if report["recent"])

  gates = {
    "routeCoverage": len(routes) >= MIN_ROUTES,
    "durationCoverage": usable_minutes >= MIN_USABLE_MINUTES,
    "weightedMae": weighted_mae >= MIN_WEIGHTED_MAE_IMPROVEMENT_PCT,
    "lowSpeedMae": weighted_low >= MIN_LOW_SPEED_MAE_IMPROVEMENT_PCT,
    "weightedP95": weighted_p95 >= MIN_WEIGHTED_P95_IMPROVEMENT_PCT,
    "worstRoute": worst_route >= -MAX_ROUTE_REGRESSION_PCT,
    "recentRoutes": worst_recent >= -MAX_RECENT_ROUTE_REGRESSION_PCT,
    "profileSmoothness": adjacent_ratio <= MAX_ADJACENT_FACTOR_RATIO,
  }
  accepted = all(gates.values())
  failed = [name for name, passed in gates.items() if not passed]
  return {
    "schemaVersion": 1,
    "carFingerprint": CAR_FINGERPRINT,
    "decision": "ACCEPT_ACTUATION" if accepted else "REJECT_ACTUATION",
    "reason": "all held-out gates passed" if accepted else f"failed gates: {', '.join(failed)}",
    "sourceLatAccelFactor": source_laf,
    "baseline": "route-held-out global torqued TLS fit using speeds above 15 m/s",
    "samples": {"rows": int(len(samples)), "minutesAt100Hz": round(usable_minutes, 2), "routes": len(routes)},
    "globalFit": full_global,
    "speedProfile": {
      "centersMps": SPEED_CENTERS.tolist(),
      "latAccelFactors": factors,
      "frictions": [float(fit["friction"]) for fit in full_profile],
      "adjacentFactorRatioMax": float(adjacent_ratio),
    },
    "heldout": {
      "weightedMaeImprovementPct": weighted_mae,
      "weightedLowSpeedMaeImprovementPct": weighted_low,
      "weightedP95ImprovementPct": weighted_p95,
      "worstRouteImprovementPct": worst_route,
      "worstRecentRouteImprovementPct": worst_recent,
      "recentRoutes": sorted(recent_routes),
      "routes": holdouts,
    },
    "gates": gates,
    "proofBoundary": "Offline route-held-out physical-response replay only; no engaged-drive proof.",
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("dataset", type=Path, help="clean_with_provenance.parquet")
  parser.add_argument("--output", type=Path)
  parser.add_argument("--source-laf", type=float, default=1.63)
  args = parser.parse_args()

  report = evaluate(pd.read_parquet(args.dataset), source_laf=args.source_laf)
  payload = json.dumps(report, indent=2) + "\n"
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(payload, encoding="utf-8")
  print(payload, end="")
  return 0 if report["decision"] == "ACCEPT_ACTUATION" else 2


if __name__ == "__main__":
  raise SystemExit(main())
