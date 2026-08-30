#!/usr/bin/env python3
"""Validate Carnival NNFF JSON models against route-held-out datasets."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import pandas as pd


EXPECTED_INPUTS = [
  "v_ego", "actual_lateral_accel", "lateral_jerk", "roll",
  "actual_lateral_accel_tm03", "actual_lateral_accel_tm02", "actual_lateral_accel_tm01",
  "actual_lateral_accel_tp03", "actual_lateral_accel_tp06", "actual_lateral_accel_tp10",
  "actual_lateral_accel_tp15", "roll_tm03", "roll_tm02", "roll_tm01",
  "roll_tp03", "roll_tp06", "roll_tp10", "roll_tp15",
]
SPEED_BANDS = ((0.3, 5.0), (5.0, 10.0), (10.0, 15.0), (15.0, 22.0), (22.0, 35.0), (35.0, 45.0))


class JsonModel:
  def __init__(self, path: Path):
    self.path = path
    self.data = json.loads(path.read_text())
    self.input_size = int(self.data["input_size"])
    self.input_vars = list(self.data.get("input_vars", []))
    self.mean = np.asarray(self.data["input_mean"], dtype=np.float32).reshape(-1)
    self.std = np.asarray(self.data["input_std"], dtype=np.float32).reshape(-1)
    self.layers = []
    for layer in self.data["layers"]:
      weight_key = next(key for key in layer if key.endswith("_W"))
      bias_key = next(key for key in layer if key.endswith("_b"))
      activation = str(layer["activation"]).replace("σ", "sigmoid")
      self.layers.append((
        np.asarray(layer[weight_key], dtype=np.float32),
        np.asarray(layer[bias_key], dtype=np.float32).reshape(-1),
        activation,
      ))

  def evaluate(self, values: np.ndarray) -> np.ndarray:
    output = (values.astype(np.float32) - self.mean) / self.std
    for weights, bias, activation in self.layers:
      output = output @ weights.T + bias
      if activation == "sigmoid":
        output = 1.0 / (1.0 + np.exp(-np.clip(output, -40.0, 40.0)))
      elif activation not in ("identity", "identity_fast"):
        raise ValueError(f"Unsupported activation {activation!r} in {self.path}")
    return output.reshape(-1)


def canonical_frame(frame: pd.DataFrame) -> pd.DataFrame:
  result = frame.copy()
  result["lateral_jerk"] = (
    result["actual_lateral_accel_tp03"] - result["actual_lateral_accel"]
  ) / 0.3
  return result


def metrics(target: np.ndarray, prediction: np.ndarray) -> dict:
  error = prediction - target
  return {
    "rows": int(len(target)),
    "mse": float(np.mean(error ** 2)),
    "mae": float(np.mean(np.abs(error))),
    "p95AbsoluteError": float(np.percentile(np.abs(error), 95)),
    "maximumAbsoluteOutput": float(np.max(np.abs(prediction))),
    "signAgreement": float(np.mean((np.sign(prediction) == np.sign(target)) | (np.abs(target) < 0.03))),
  }


def heldout_metrics(model: JsonModel, frame: pd.DataFrame) -> dict:
  frame = canonical_frame(frame)
  values = frame[EXPECTED_INPUTS].to_numpy(dtype=np.float32)
  target = frame["torque_output"].to_numpy(dtype=np.float32)
  prediction = model.evaluate(values)
  report = metrics(target, prediction)
  report["speedBands"] = {}
  speeds = frame["v_ego"].to_numpy(dtype=float)
  for low, high in SPEED_BANDS:
    mask = (speeds >= low) & (speeds < high)
    if mask.sum() >= 100:
      report["speedBands"][f"{low:g}-{high:g}"] = metrics(target[mask], prediction[mask])
  return report


def physical_contract_metrics(model: JsonModel) -> dict:
  speeds = np.asarray((2.0, 5.0, 10.0, 15.0, 22.0, 30.0, 38.0), dtype=np.float32)
  lat_accels = np.linspace(-3.0, 3.0, 121, dtype=np.float32)
  samples = []
  for speed in speeds:
    for lat_accel in lat_accels:
      row = np.zeros(18, dtype=np.float32)
      row[0] = speed
      row[1] = lat_accel
      row[4:11] = lat_accel
      samples.append(row)
  samples = np.asarray(samples)
  outputs = model.evaluate(samples).reshape(len(speeds), len(lat_accels))

  # Training targets controlsState.torqueState.output. LatControlNNFF negates
  # this output at the actuator boundary, so valid inverse-dynamics models are
  # monotonically decreasing as requested lateral acceleration increases.
  monotonic_violations = int((np.diff(outputs, axis=1) > 1e-4).sum())
  monotonic_comparisons = int(np.diff(outputs, axis=1).size)
  mirrored = np.flip(outputs, axis=1)
  symmetry_error = np.abs(outputs + mirrored)
  center_index = len(lat_accels) // 2
  return {
    "finite": bool(np.isfinite(outputs).all()),
    "maximumAbsoluteOutput": float(np.max(np.abs(outputs))),
    "maximumAbsoluteOriginOutput": float(np.max(np.abs(outputs[:, center_index]))),
    "p95OddSymmetryError": float(np.percentile(symmetry_error, 95)),
    "monotonicDirection": "decreasing (controller output sign convention)",
    "monotonicViolationRate": monotonic_violations / max(monotonic_comparisons, 1),
  }


def model_name_from_directory(path: Path) -> str:
  return path.parent.name


def heldout_identifier(model_name: str) -> str | None:
  marker = "_without_"
  return model_name.split(marker, 1)[1] if marker in model_name else None


def validate_model_contract(model: JsonModel) -> list[str]:
  failures = []
  if model.input_size != 18:
    failures.append(f"input_size={model.input_size}, expected 18")
  if model.input_vars != EXPECTED_INPUTS:
    failures.append("input_vars do not match StarPilot's positional NNFF contract")
  if len(model.mean) != 18 or len(model.std) != 18:
    failures.append("normalization vectors are not length 18")
  if not np.isfinite(model.mean).all() or not np.isfinite(model.std).all():
    failures.append("normalization vectors contain non-finite values")
  if (model.std <= 1e-6).any():
    failures.append("normalization vector contains a near-zero standard deviation")
  return failures


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("run_dir", type=Path)
  args = parser.parse_args()

  manifest = json.loads((args.run_dir / "dataset_manifest.json").read_text())
  model_paths = sorted((args.run_dir / "training_inputs" / "training_results").glob("*/*.json"))
  if not model_paths:
    raise SystemExit("No trained JSON models found")

  reports = []
  for model_path in model_paths:
    model = JsonModel(model_path)
    name = model_name_from_directory(model_path)
    failures = validate_model_contract(model)
    physical = physical_contract_metrics(model) if not failures else {}
    identifier = heldout_identifier(name)
    holdout = None
    if identifier:
      heldout_path = args.run_dir / "heldout" / f"{identifier}.parquet"
      if not heldout_path.exists():
        failures.append(f"held-out dataset is missing: {heldout_path.name}")
      else:
        holdout = heldout_metrics(model, pd.read_parquet(heldout_path))
        if holdout["mse"] > 0.04:
          failures.append(f"held-out MSE {holdout['mse']:.5f} exceeds 0.04")
        if holdout["signAgreement"] < 0.85:
          failures.append(f"held-out sign agreement {holdout['signAgreement']:.3f} is below 0.85")

    if physical:
      if not physical["finite"]:
        failures.append("physical grid produced non-finite outputs")
      if physical["maximumAbsoluteOutput"] > 1.25:
        failures.append("physical grid exceeds normalized output magnitude 1.25")
      if physical["maximumAbsoluteOriginOutput"] > 0.08:
        failures.append("zero-demand output exceeds 0.08")
      if physical["p95OddSymmetryError"] > 0.12:
        failures.append("odd-symmetry error exceeds 0.12")
      if physical["monotonicViolationRate"] > 0.01:
        failures.append("monotonic violation rate exceeds 1%")

    reports.append({
      "name": name,
      "path": str(model_path),
      "internalTestLoss": float(model.data.get("model_test_loss", math.nan)),
      "physicalContract": physical,
      "heldout": holdout,
      "passed": not failures,
      "failures": failures,
    })

  full = next((item for item in reports if item["name"] == "KIA_CARNIVAL_4TH_GEN"), None)
  folds = [item for item in reports if "_without_" in item["name"]]
  all_folds_pass = bool(folds) and all(item["passed"] for item in folds)
  ready_for_runtime_audit = bool(full and full["passed"] and all_folds_pass and manifest["prototypeReady"])
  report = {
    "dataset": manifest,
    "models": reports,
    "allHeldoutFoldsPassed": all_folds_pass,
    "readyForOfflineRuntimeAudit": ready_for_runtime_audit,
    "readyForShadowTesting": False,
    "readyForActuationTesting": False,
    "note": (
      "Static training and held-out validation cannot establish compatibility with the live NNFF input contract "
      "or a driving-model family. Run audit_runtime_compatibility.py first; actuation additionally requires "
      "reviewed model-specific shadow telemetry and an explicit deployment decision."
    ),
  }
  output = args.run_dir / "validation_report.json"
  output.write_text(json.dumps(report, indent=2) + "\n")
  print(json.dumps(report, indent=2))
  return 0 if ready_for_runtime_audit else 3


if __name__ == "__main__":
  raise SystemExit(main())
