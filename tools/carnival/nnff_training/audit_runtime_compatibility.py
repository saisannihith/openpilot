#!/usr/bin/env python3
"""Read-only audit of a trained NNFF model against modern runtime inputs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd
from nnlc_tools.logreader import LogReader


T_IDXS = np.array([
  0.0, 0.009765625, 0.0390625, 0.087890625, 0.15625, 0.244140625,
  0.3515625, 0.478515625, 0.625, 0.791015625, 0.9765625,
  1.181640625, 1.40625, 1.650390625, 1.9140625, 2.197265625, 2.5,
  2.822265625, 3.1640625, 3.525390625, 3.90625, 4.306640625,
  4.7265625, 5.166015625, 5.625, 6.103515625, 6.6015625,
  7.119140625, 7.65625, 8.212890625, 8.7890625, 9.384765625, 10.0,
], dtype=np.float64)
FUTURE_TIMES = (0.3, 0.6, 1.0, 1.5)
LAT_PLAN_MIN_IDX = 5


def safe_attr(obj, name, default=None):
  try:
    return getattr(obj, name)
  except (AttributeError, TypeError):
    return default


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)$", path.parent.name)
  return int(match.group(1)) if match else 0


def find_rlogs(root: Path, routes: set[str]) -> list[Path]:
  found = set()
  for name in ("rlog", "rlog.zst", "rlog.bz2"):
    found.update(path for path in root.rglob(name) if route_name(path) in routes)
  return sorted(found, key=lambda path: (route_name(path), segment_number(path)))


def decode_param(value) -> str:
  if isinstance(value, bytes):
    return value.decode("utf-8", "replace")
  return str(value)


def read_route_model_metadata(rlogs: list[Path]) -> dict[str, dict[str, str]]:
  wanted = {
    "ModelName", "Model", "ModelVersion",
    "DrivingModelName", "DrivingModel", "DrivingModelVersion",
    "NNFF", "NNFFLite",
  }
  metadata = {}
  for path in rlogs:
    route = route_name(path)
    if route in metadata:
      continue
    values = {}
    for msg in LogReader(str(path), sort_by_time=False):
      if msg.which() != "initData":
        continue
      entries = safe_attr(safe_attr(msg.initData, "params"), "entries", [])
      for entry in entries:
        key = str(entry.key)
        if key in wanted:
          values[key] = decode_param(entry.value)
      break
    metadata[route] = {
      "name": values.get("ModelName", values.get("DrivingModelName", "unknown")),
      "id": values.get("Model", values.get("DrivingModel", "unknown")),
      "version": values.get("ModelVersion", values.get("DrivingModelVersion", "unknown")),
      "nnff": values.get("NNFF", "unknown"),
      "nnffLite": values.get("NNFFLite", "unknown"),
    }
  return metadata


def sign(value: float) -> float:
  return 1.0 if value > 0.0 else -1.0 if value < 0.0 else 0.0


def lookahead_value(values: list[float], current: float) -> float:
  if not values:
    return current
  if any(sign(value) != sign(current) for value in values):
    return 0.0
  return min(values + [current], key=abs)


class FluxModel:
  def __init__(self, path: Path):
    self.data = json.loads(path.read_text())
    self.mean = np.asarray(self.data["input_mean"], dtype=np.float64).reshape(-1)
    self.std = np.asarray(self.data["input_std"], dtype=np.float64).reshape(-1)
    self.layers = []
    for layer in self.data["layers"]:
      weight = np.asarray(layer[next(key for key in layer if key.endswith("_W"))], dtype=np.float64).T
      bias = np.asarray(layer[next(key for key in layer if key.endswith("_b"))], dtype=np.float64).reshape(-1)
      activation = layer["activation"].replace("σ", "sigmoid")
      self.layers.append((weight, bias, activation))

  def evaluate_many(self, inputs: np.ndarray) -> np.ndarray:
    values = (inputs - self.mean) / self.std
    for weight, bias, activation in self.layers:
      values = values @ weight + bias
      if activation == "sigmoid":
        values = 1.0 / (1.0 + np.exp(-np.clip(values, -60.0, 60.0)))
      elif activation != "identity":
        raise ValueError(f"unsupported activation {activation}")
    return values.reshape(-1)


def parse_segment(path: Path, clean_keys: set[tuple[int, int]]) -> list[dict]:
  state = {}
  rows = []
  steer_delay = 0.15
  route = route_name(path)
  segment = segment_number(path)
  for msg in LogReader(str(path), sort_by_time=True):
    try:
      which = msg.which()
    except Exception:
      continue
    if which == "carParams":
      steer_delay = float(safe_attr(msg.carParams, "steerActuatorDelay", steer_delay))
    elif which == "carState":
      state[which] = msg.carState
    elif which == "liveParameters":
      state[which] = msg.liveParameters
    elif which == "modelV2":
      model = msg.modelV2
      if len(model.acceleration.y) == len(T_IDXS) and len(model.orientation.x) == len(T_IDXS):
        state[which] = {
          "timestamp": msg.logMonoTime / 1e9,
          "acceleration_y": np.asarray(model.acceleration.y, dtype=np.float64),
          "orientation_x": np.asarray(model.orientation.x, dtype=np.float64),
        }
    elif which != "controlsState":
      continue

    if which != "controlsState" or "carState" not in state or "modelV2" not in state:
      continue
    timestamp = msg.logMonoTime / 1e9
    if (segment, round(timestamp * 1000)) not in clean_keys:
      continue
    lat_state = msg.controlsState.lateralControlState
    if lat_state.which() != "torqueState":
      continue
    torque = lat_state.torqueState
    car = state["carState"]
    live = state.get("liveParameters")
    model = state["modelV2"]
    rows.append({
      "route": route, "segment": segment, "timestamp": timestamp,
      "v_ego": float(car.vEgo), "a_ego": float(car.aEgo),
      "actual_lateral_accel": float(torque.actualLateralAccel),
      "desired_lateral_accel": float(torque.desiredLateralAccel),
      "torque_output": float(torque.output),
      "roll": float(safe_attr(live, "roll", math.nan)),
      "steer_delay": steer_delay, "model_age": timestamp - model["timestamp"],
      "model_acceleration_y": model["acceleration_y"],
      "model_orientation_x": model["orientation_x"],
    })
  return rows


def quantiles(values: np.ndarray) -> dict:
  finite = values[np.isfinite(values)]
  if not len(finite):
    return {"count": 0}
  return {
    "count": int(len(finite)), "mean": float(np.mean(finite)),
    "p50": float(np.quantile(finite, 0.50)), "p95": float(np.quantile(finite, 0.95)),
    "p99": float(np.quantile(finite, 0.99)), "max": float(np.max(finite)),
  }


def summarize_group(group: pd.DataFrame) -> dict:
  delta = np.abs(group.runtime_output - group.training_output).to_numpy()
  runtime_error = group.runtime_output.to_numpy() - group.torque_output.to_numpy()
  training_error = group.training_output.to_numpy() - group.torque_output.to_numpy()
  sign_flip = ((np.sign(group.runtime_output) != np.sign(group.training_output)) &
               (np.abs(group.training_output) > 0.03)).to_numpy()
  return {
    "rows": int(len(group)),
    "modelAgeSeconds": quantiles(np.abs(group.model_age.to_numpy())),
    "runtimeVsTrainingOutputAbsoluteDelta": quantiles(delta),
    "runtimeVsTrainingSignFlipRate": float(np.mean(sign_flip)),
    "runtimeOutputAbsolute": quantiles(np.abs(group.runtime_output.to_numpy())),
    "trainingOutputAbsolute": quantiles(np.abs(group.training_output.to_numpy())),
    "runtimeVsObservedTorque": {
      "mse": float(np.mean(runtime_error ** 2)),
      "mae": float(np.mean(np.abs(runtime_error))),
      "p95AbsoluteError": float(np.quantile(np.abs(runtime_error), 0.95)),
    },
    "trainingVsObservedTorque": {
      "mse": float(np.mean(training_error ** 2)),
      "mae": float(np.mean(np.abs(training_error))),
      "p95AbsoluteError": float(np.quantile(np.abs(training_error), 0.95)),
    },
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("rlog_root", type=Path)
  parser.add_argument("clean_dataset", type=Path)
  parser.add_argument("model", type=Path)
  parser.add_argument("output", type=Path)
  args = parser.parse_args()

  clean = pd.read_parquet(args.clean_dataset)
  routes = set(clean.route.astype(str))
  clean_keys_by_route = defaultdict(set)
  for row in clean[["route", "segment", "timestamp"]].itertuples(index=False):
    clean_keys_by_route[str(row.route)].add((int(row.segment), round(float(row.timestamp) * 1000)))

  rlogs = find_rlogs(args.rlog_root, routes)
  route_model_metadata = read_route_model_metadata(rlogs)
  rows = []
  for index, path in enumerate(rlogs, 1):
    print(f"[{index}/{len(rlogs)}] {path}", flush=True)
    rows.extend(parse_segment(path, clean_keys_by_route[route_name(path)]))
  if not rows:
    raise SystemExit("no clean controlsState/modelV2 pairs found")

  runtime = pd.DataFrame(rows)
  temporal_columns = [
    "actual_lateral_accel_tm03", "actual_lateral_accel_tm02", "actual_lateral_accel_tm01",
    "desired_lateral_accel_tm03", "desired_lateral_accel_tm02", "desired_lateral_accel_tm01",
    "actual_lateral_accel_tp03", "actual_lateral_accel_tp06", "actual_lateral_accel_tp10", "actual_lateral_accel_tp15",
    "roll_tm03", "roll_tm02", "roll_tm01", "roll_tp03", "roll_tp06", "roll_tp10", "roll_tp15",
  ]
  source = clean[["route", "segment", "timestamp", *temporal_columns]].copy()
  source["timestamp_ms"] = np.round(source.timestamp * 1000).astype(np.int64)
  runtime["timestamp_ms"] = np.round(runtime.timestamp * 1000).astype(np.int64)
  merged = runtime.merge(source.drop(columns="timestamp"), on=["route", "segment", "timestamp_ms"], how="inner")
  merged["driving_model"] = merged.route.map(
    lambda route: " / ".join(route_model_metadata.get(route, {}).get(key, "unknown") for key in ("name", "version", "id"))
  )

  runtime_inputs, training_inputs = [], []
  plan_errors = {horizon: [] for horizon in FUTURE_TIMES}
  for row in merged.itertuples(index=False):
    adjusted_times = np.asarray([
      horizon + row.steer_delay + 0.5 * row.a_ego * ((horizon + row.steer_delay) / max(row.v_ego, 1.0))
      for horizon in FUTURE_TIMES
    ])
    future_accels = np.interp(adjusted_times, T_IDXS, row.model_acceleration_y)
    future_rolls = np.interp(adjusted_times, T_IDXS, row.model_orientation_x) + row.roll
    predicted_jerk = np.diff(row.model_acceleration_y) / np.diff(T_IDXS)
    lookahead = float(np.interp(row.v_ego, [9.0, 30.0], [1.4, 2.0]))
    upper_idx = next((idx for idx, value in enumerate(T_IDXS) if value > lookahead), 16)
    delay = max(row.steer_delay, 0.01)
    desired_jerk = (float(np.interp(delay, T_IDXS, row.model_acceleration_y)) - row.desired_lateral_accel) / delay
    lookahead_jerk = lookahead_value(predicted_jerk[LAT_PLAN_MIN_IDX:upper_idx].tolist(), desired_jerk)
    # Both current SunnyPilot and StarPilot permanently switch this factor to
    # 1.0 after the first rejected lookahead. Every audited segment reaches
    # that state during startup, so 1.0 matches steady-state runtime behavior.
    friction_input = (row.desired_lateral_accel - row.actual_lateral_accel) + 0.4 * lookahead_jerk
    runtime_inputs.append([
      row.v_ego, row.desired_lateral_accel, friction_input, row.roll,
      row.desired_lateral_accel_tm03, row.desired_lateral_accel_tm02, row.desired_lateral_accel_tm01,
      *future_accels, row.roll_tm03, row.roll_tm02, row.roll_tm01, *future_rolls,
    ])
    training_inputs.append([
      row.v_ego, row.actual_lateral_accel,
      (row.actual_lateral_accel_tp03 - row.actual_lateral_accel) / 0.3, row.roll,
      row.actual_lateral_accel_tm03, row.actual_lateral_accel_tm02, row.actual_lateral_accel_tm01,
      row.actual_lateral_accel_tp03, row.actual_lateral_accel_tp06,
      row.actual_lateral_accel_tp10, row.actual_lateral_accel_tp15,
      row.roll_tm03, row.roll_tm02, row.roll_tm01,
      row.roll_tp03, row.roll_tp06, row.roll_tp10, row.roll_tp15,
    ])
    achieved = [row.actual_lateral_accel_tp03, row.actual_lateral_accel_tp06,
                row.actual_lateral_accel_tp10, row.actual_lateral_accel_tp15]
    for horizon, planned, actual in zip(FUTURE_TIMES, future_accels, achieved, strict=True):
      plan_errors[horizon].append(planned - actual)

  model = FluxModel(args.model)
  runtime_array = np.asarray(runtime_inputs, dtype=np.float64)
  training_array = np.asarray(training_inputs, dtype=np.float64)
  merged["runtime_output"] = model.evaluate_many(runtime_array)
  merged["training_output"] = model.evaluate_many(training_array)
  runtime_z = np.abs((runtime_array - model.mean) / model.std)
  names = model.data.get("input_vars", [f"input_{index}" for index in range(runtime_array.shape[1])])
  ood = {name: {
    "over3SigmaRate": float(np.mean(runtime_z[:, index] > 3.0)),
    "over5SigmaRate": float(np.mean(runtime_z[:, index] > 5.0)),
    "p99AbsoluteZ": float(np.quantile(runtime_z[:, index], 0.99)),
    "maximumAbsoluteZ": float(np.max(runtime_z[:, index])),
  } for index, name in enumerate(names)}

  sign_flip = ((np.sign(merged.runtime_output) != np.sign(merged.training_output)) &
               (np.abs(merged.training_output) > 0.03))
  runtime_contract_gates = {
    "modelAgeP99Below100ms": bool(np.quantile(np.abs(merged.model_age), 0.99) < 0.1),
    "allRuntimeFeaturesP99Below5Sigma": bool(all(item["p99AbsoluteZ"] < 5.0 for item in ood.values())),
  }
  report = {
    "model": str(args.model), "modelSha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
    "rlogsFound": len(rlogs), "rows": len(merged),
    "runtimeContract": {
      "thirdInput": "lateral-acceleration error plus model-planned lookahead jerk",
      "futureInputs": "modelV2 plan at actuator-delay-adjusted horizons",
    },
    "trainingContract": {
      "thirdInput": "future achieved lateral-acceleration change over 0.3 seconds",
      "futureInputs": "future achieved lateral acceleration and observed roll",
    },
    "overall": summarize_group(merged),
    "perRoute": {route: summarize_group(group) for route, group in merged.groupby("route")},
    "routeModelMetadata": route_model_metadata,
    "perDrivingModel": {model_name: summarize_group(group) for model_name, group in merged.groupby("driving_model")},
    "drivingModelsObserved": sorted(merged.driving_model.unique().tolist()),
    "crossModelCompatibilityEstablished": False,
    "plannedVsAchievedLateralAccelerationAbsoluteError": {
      str(horizon): quantiles(np.abs(np.asarray(errors))) for horizon, errors in plan_errors.items()
    },
    "runtimeInputOutOfDistribution": ood,
    "inverseDynamicsSubstitutionDiagnostics": {
      "runtimeVsTrainingOutputDeltaP95": float(np.quantile(np.abs(merged.runtime_output - merged.training_output), 0.95)),
      "runtimeVsTrainingSignFlipRate": float(np.mean(sign_flip)),
      "interpretation": (
        "Diagnostic only: inverse-dynamics models are trained on achieved motion and queried with desired/planned motion. "
        "These differences are not compatibility gates by themselves."
      ),
    },
    "runtimeContractGates": runtime_contract_gates,
    "readyForOfflineRuntimeEvaluation": bool(all(runtime_contract_gates.values())),
    "compatibleForShadowTesting": False,
    "compatibleForActuation": False,
    "note": (
      "Offline replay cannot establish cross-model compatibility, closed-loop stability, or road safety. "
      "Each intended driving-model family needs its own logged runtime-contract audit and shadow validation."
    ),
  }
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(report, indent=2) + "\n")
  print(json.dumps(report, indent=2))
  return 0 if all(runtime_contract_gates.values()) else 3


if __name__ == "__main__":
  raise SystemExit(main())
