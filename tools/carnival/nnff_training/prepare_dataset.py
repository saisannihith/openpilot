#!/usr/bin/env python3
"""Build provenance-preserving, segment-safe Carnival NNFF datasets."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd

from nnlc_tools.logreader import LogReader


PAST_TIMES = (-0.3, -0.2, -0.1)
FUTURE_TIMES = (0.3, 0.6, 1.0, 1.5)
TEMPORAL_TIMES = PAST_TIMES + FUTURE_TIMES
CAR_FINGERPRINT = "KIA_CARNIVAL_4TH_GEN"
MODEL_NAME = f"{CAR_FINGERPRINT}.csv"
MAX_DRIVER_TORQUE = 100.0
MAX_TEMPORAL_SKEW = 0.025
ACCELERATION_DUE_TO_GRAVITY = 9.80665

BASE_COLUMNS = [
  "timestamp", "v_ego", "a_ego", "steering_angle_deg", "steering_rate_deg",
  "steering_torque", "steering_pressed", "steer_fault_temporary", "standstill",
  "lat_active", "actual_lateral_accel", "desired_lateral_accel", "torque_output",
  "applied_torque", "car_output_age", "lateral_delay", "saturated", "roll", "lane_change_state",
  "pose_lateral_accel", "pose_age", "pose_valid",
  "live_torque_use_params", "live_torque_valid", "live_torque_cal_perc",
  "live_lat_accel_factor_raw", "live_lat_accel_factor_filtered",
  "live_friction_raw", "live_friction_filtered",
]


def safe_attr(obj, name, default=None):
  try:
    return getattr(obj, name)
  except (AttributeError, TypeError):
    return default


def lane_change_code(value) -> int:
  """Normalize pycapnp enum wrappers without depending on schema ordinals."""
  try:
    return int(value)
  except (TypeError, ValueError):
    name = str(value).rsplit(".", 1)[-1].lower()
    return 0 if name in ("off", "none", "") else 1


def rotation_from_euler(rpy: np.ndarray) -> np.ndarray:
  roll, pitch, yaw = rpy
  sr, cr = math.sin(roll), math.cos(roll)
  sp, cp = math.sin(pitch), math.cos(pitch)
  sy, cy = math.sin(yaw), math.cos(yaw)
  return np.array([
    [cy * cp, (cy * sp * sr) - (sy * cr), (cy * sp * cr) + (sy * sr)],
    [sy * cp, (sy * sp * sr) + (cy * cr), (sy * sp * cr) - (cy * sr)],
    [-sp, cp * sr, cp * cr],
  ])


def find_rlogs(root: Path) -> list[Path]:
  paths: set[Path] = set()
  for name in ("rlog", "rlog.zst", "rlog.bz2"):
    paths.update(root.rglob(name))
  return sorted(paths)


def route_name(path: Path) -> str:
  match = re.match(r"(.+--[0-9a-f]+)--\d+$", path.parent.name)
  return match.group(1) if match else path.parent.name


def segment_number(path: Path) -> int:
  match = re.search(r"--(\d+)$", path.parent.name)
  return int(match.group(1)) if match else 0


def extract_segment(path: Path) -> tuple[pd.DataFrame, dict]:
  state = {}
  calib_from_device = np.eye(3)
  calib_valid = False
  rows = []
  metadata = {
    "route": route_name(path),
    "segment": segment_number(path),
    "source": str(path),
    "git_commit": "",
    "git_branch": "",
    "git_dirty": None,
    "car_fingerprint": "",
    "skipped_messages": 0,
  }

  try:
    messages = LogReader(str(path), sort_by_time=True)
  except Exception as error:
    metadata["error"] = f"open failed: {error}"
    return pd.DataFrame(columns=BASE_COLUMNS), metadata

  try:
    for msg in messages:
      try:
        which = msg.which()
      except Exception:
        metadata["skipped_messages"] += 1
        continue
      if which == "initData":
        init = msg.initData
        metadata["git_commit"] = str(safe_attr(init, "gitCommit", ""))
        metadata["git_branch"] = str(safe_attr(init, "gitBranch", ""))
        metadata["git_dirty"] = bool(safe_attr(init, "dirty", False))
      elif which == "carParams":
        metadata["car_fingerprint"] = str(safe_attr(msg.carParams, "carFingerprint", ""))
      elif which == "carState":
        state[which] = msg.carState
      elif which == "controlsState":
        state[which] = msg.controlsState
      elif which == "carControl":
        state[which] = msg.carControl
      elif which == "carOutput":
        state[which] = (msg.logMonoTime / 1e9, msg.carOutput)
      elif which == "selfdriveState":
        state[which] = msg.selfdriveState
      elif which == "liveParameters":
        state[which] = msg.liveParameters
      elif which == "liveDelay":
        state[which] = msg.liveDelay
      elif which == "liveTorqueParameters":
        state[which] = msg.liveTorqueParameters
      elif which == "liveCalibration":
        rpy_calib = np.asarray(safe_attr(msg.liveCalibration, "rpyCalib", ()), dtype=float)
        if rpy_calib.shape == (3,):
          calib_from_device = rotation_from_euler(rpy_calib).T
          status = str(safe_attr(msg.liveCalibration, "calStatus", "")).rsplit(".", 1)[-1].lower()
          calib_valid = status == "calibrated"
      elif which == "livePose" and "carState" in state:
        pose = msg.livePose
        orientation = safe_attr(pose, "orientationNED")
        angular_velocity = safe_attr(pose, "angularVelocityDevice")
        pose_valid = bool(
          calib_valid and safe_attr(orientation, "valid", False) and
          safe_attr(angular_velocity, "valid", False) and safe_attr(pose, "inputsOK", False)
        )
        orientation_xyz = np.array([
          safe_attr(orientation, "x", math.nan), safe_attr(orientation, "y", math.nan), safe_attr(orientation, "z", math.nan),
        ], dtype=float)
        angular_velocity_xyz = np.array([
          safe_attr(angular_velocity, "x", math.nan), safe_attr(angular_velocity, "y", math.nan),
          safe_attr(angular_velocity, "z", math.nan),
        ], dtype=float)
        calibrated_angular_velocity = calib_from_device @ angular_velocity_xyz
        ned_from_calibrated = rotation_from_euler(orientation_xyz) @ calib_from_device.T
        calibrated_roll = math.atan2(ned_from_calibrated[2, 1], ned_from_calibrated[2, 2])
        lateral_accel = (
          float(state["carState"].vEgo) * calibrated_angular_velocity[2] -
          math.sin(calibrated_roll) * ACCELERATION_DUE_TO_GRAVITY
        )
        state["poseLateralAccel"] = (msg.logMonoTime / 1e9, lateral_accel, pose_valid)
      elif which == "modelV2":
        state[which] = msg.modelV2

      if which != "controlsState" or "carState" not in state:
        continue

      cs = state["carState"]
      controls = state["controlsState"]
      lat_state = controls.lateralControlState
      try:
        lat_type = lat_state.which()
      except Exception:
        metadata["skipped_messages"] += 1
        continue
      if lat_type != "torqueState":
        continue
      torque_state = lat_state.torqueState

      selfdrive = state.get("selfdriveState")
      car_control = state.get("carControl")
      lat_active = bool(safe_attr(
        car_control, "latActive", safe_attr(selfdrive, "active", safe_attr(controls, "activeDEPRECATED", False)),
      ))
      lane_change_state = 0
      model = state.get("modelV2")
      if model is not None:
        lane_change_state = lane_change_code(
          safe_attr(safe_attr(model, "meta"), "laneChangeState", 0),
        )

      live_params = state.get("liveParameters")
      car_output_time, car_output = state.get("carOutput", (math.nan, None))
      applied_torque = float(safe_attr(safe_attr(car_output, "actuatorsOutput"), "torque", math.nan))
      lateral_delay = float(safe_attr(state.get("liveDelay"), "lateralDelay", math.nan))
      pose_time, pose_lateral_accel, pose_valid = state.get("poseLateralAccel", (math.nan, math.nan, False))
      live_torque = state.get("liveTorqueParameters")
      rows.append([
        msg.logMonoTime / 1e9,
        float(cs.vEgo),
        float(cs.aEgo),
        float(cs.steeringAngleDeg),
        float(cs.steeringRateDeg),
        float(cs.steeringTorque),
        bool(cs.steeringPressed),
        bool(safe_attr(cs, "steerFaultTemporary", False)),
        bool(cs.standstill),
        lat_active,
        float(torque_state.actualLateralAccel),
        float(torque_state.desiredLateralAccel),
        float(torque_state.output),
        applied_torque,
        (msg.logMonoTime / 1e9) - car_output_time,
        lateral_delay,
        bool(torque_state.saturated),
        float(safe_attr(live_params, "roll", math.nan)),
        lane_change_state,
        pose_lateral_accel,
        (msg.logMonoTime / 1e9) - pose_time,
        pose_valid,
        bool(safe_attr(live_torque, "useParams", False)),
        bool(safe_attr(live_torque, "liveValid", False)),
        float(safe_attr(live_torque, "calPerc", math.nan)),
        float(safe_attr(live_torque, "latAccelFactorRaw", math.nan)),
        float(safe_attr(live_torque, "latAccelFactorFiltered", math.nan)),
        float(safe_attr(live_torque, "frictionCoefficientRaw", math.nan)),
        float(safe_attr(live_torque, "frictionCoefficientFiltered", math.nan)),
      ])
  except Exception as error:
    metadata["error"] = f"parse failed: {error}"

  return pd.DataFrame(rows, columns=BASE_COLUMNS), metadata


def temporal_suffix(offset: float) -> str:
  return f"_t{offset:+.1f}".replace(".", "").replace("+", "p").replace("-", "m")


def nearest_temporal(values: np.ndarray, times: np.ndarray, offset: float) -> np.ndarray:
  targets = times + offset
  right = np.searchsorted(times, targets, side="left")
  right = np.clip(right, 0, len(times) - 1)
  left = np.clip(right - 1, 0, len(times) - 1)
  choose_left = np.abs(times[left] - targets) <= np.abs(times[right] - targets)
  index = np.where(choose_left, left, right)
  valid = (
    (targets >= times[0]) &
    (targets <= times[-1]) &
    (np.abs(times[index] - targets) <= MAX_TEMPORAL_SKEW)
  )
  return np.where(valid, values[index], np.nan)


def context_is_clean(df: pd.DataFrame) -> np.ndarray:
  times = df["timestamp"].to_numpy(dtype=float)
  bad = (
    ~df["lat_active"].to_numpy(dtype=bool) |
    df["steering_pressed"].to_numpy(dtype=bool) |
    df["steer_fault_temporary"].to_numpy(dtype=bool) |
    df["saturated"].to_numpy(dtype=bool) |
    df["standstill"].to_numpy(dtype=bool) |
    (df["lane_change_state"].to_numpy(dtype=int) != 0) |
    (np.abs(df["steering_torque"].to_numpy(dtype=float)) > MAX_DRIVER_TORQUE) |
    (np.abs(df["torque_output"].to_numpy(dtype=float)) >= 0.98) |
    (df["v_ego"].to_numpy(dtype=float) <= 0.3)
  )
  prefix = np.concatenate(([0], np.cumsum(bad.astype(np.int64))))
  left = np.searchsorted(times, times + min(TEMPORAL_TIMES), side="left")
  right = np.searchsorted(times, times + max(TEMPORAL_TIMES), side="right")
  return (prefix[right] - prefix[left]) == 0


def add_temporal_features(df: pd.DataFrame) -> pd.DataFrame:
  if df.empty:
    return df
  df = df.sort_values("timestamp").drop_duplicates("timestamp").reset_index(drop=True)
  times = df["timestamp"].to_numpy(dtype=float)
  for offset in TEMPORAL_TIMES:
    suffix = temporal_suffix(offset)
    for column in ("actual_lateral_accel", "desired_lateral_accel", "roll", "pose_lateral_accel"):
      df[f"{column}{suffix}"] = nearest_temporal(df[column].to_numpy(dtype=float), times, offset)
  df["clean_context"] = context_is_clean(df)
  return df


def route_software_metadata(segment_metadata: list[dict]) -> dict[str, dict]:
  result: dict[str, dict] = {}
  for item in segment_metadata:
    route = item["route"]
    current = result.setdefault(route, {})
    for key in ("git_commit", "git_branch", "git_dirty", "car_fingerprint"):
      if item.get(key) not in (None, ""):
        current[key] = item[key]
  return result


def training_columns() -> list[str]:
  temporal_lat = [f"actual_lateral_accel{temporal_suffix(offset)}" for offset in TEMPORAL_TIMES]
  temporal_roll = [f"roll{temporal_suffix(offset)}" for offset in TEMPORAL_TIMES]
  return ["v_ego", "actual_lateral_accel", "roll", "torque_output", *temporal_lat, *temporal_roll]


def summarize(clean: pd.DataFrame, metadata: list[dict]) -> dict:
  speed_edges = (0.3, 5.0, 10.0, 15.0, 22.0, 35.0, 45.0)
  speed_counts = {}
  for low, high in zip(speed_edges[:-1], speed_edges[1:], strict=True):
    speed_counts[f"{low:g}-{high:g}"] = int(((clean.v_ego >= low) & (clean.v_ego < high)).sum())

  per_route = {}
  for route, group in clean.groupby("route"):
    per_route[str(route)] = {
      "rows": int(len(group)),
      "clean_minutes": round(len(group) / 100.0 / 60.0, 2),
      "min_speed": round(float(group.v_ego.min()), 3),
      "max_speed": round(float(group.v_ego.max()), 3),
      "left_turn_rows": int((group.actual_lateral_accel < -0.15).sum()),
      "right_turn_rows": int((group.actual_lateral_accel > 0.15).sum()),
      "commits": sorted(set(group.git_commit.astype(str)) - {""}),
    }

  clean_minutes = len(clean) / 100.0 / 60.0
  routes = len(per_route)
  populated_speed_bands = sum(count >= 1000 for count in speed_counts.values())
  prototype_ready = routes >= 3 and clean_minutes >= 30 and populated_speed_bands >= 4
  production_data_ready = routes >= 20 and clean_minutes >= 300 and populated_speed_bands >= 5
  return {
    "carFingerprint": CAR_FINGERPRINT,
    "sourceSegments": len(metadata),
    "cleanRows": int(len(clean)),
    "cleanMinutes": round(clean_minutes, 2),
    "routes": routes,
    "speedBandRows": speed_counts,
    "perRoute": per_route,
    "prototypeReady": prototype_ready,
    "productionDataReady": production_data_ready,
    "productionDataRequirements": {
      "minimumRoutes": 20,
      "minimumCleanMinutes": 300,
      "minimumPopulatedSpeedBands": 5,
    },
  }


def safe_route_id(route: str) -> str:
  return re.sub(r"[^A-Za-z0-9_.-]+", "_", route)


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("input", type=Path)
  parser.add_argument("output", type=Path)
  parser.add_argument(
    "--parquet-only",
    action="store_true",
    help="Write the provenance parquet and manifest without NNFF CSV/heldout exports.",
  )
  args = parser.parse_args()

  rlogs = find_rlogs(args.input)
  if not rlogs:
    raise SystemExit(f"No rlogs found under {args.input}")

  segment_frames = []
  segment_metadata = []
  for index, path in enumerate(rlogs, 1):
    print(f"[{index}/{len(rlogs)}] {path}", flush=True)
    frame, metadata = extract_segment(path)
    segment_metadata.append(metadata)
    if not frame.empty:
      frame["route"] = metadata["route"]
      frame["segment"] = metadata["segment"]
      frame = add_temporal_features(frame)
      segment_frames.append(frame)

  if not segment_frames:
    raise SystemExit("No torque-controller samples were extracted")

  route_metadata = route_software_metadata(segment_metadata)
  combined = pd.concat(segment_frames, ignore_index=True)
  for key in ("git_commit", "git_branch", "git_dirty", "car_fingerprint"):
    combined[key] = combined["route"].map(lambda route: route_metadata.get(route, {}).get(key, ""))

  required = training_columns()
  finite = np.isfinite(combined[required].to_numpy(dtype=float)).all(axis=1)
  fingerprint_ok = combined.car_fingerprint.isin(("", CAR_FINGERPRINT))
  clean = combined[combined.clean_context & finite & fingerprint_ok].copy()
  clean = clean.sort_values(["route", "segment", "timestamp"]).reset_index(drop=True)
  if clean.empty:
    raise SystemExit("All extracted samples were rejected by safety/quality filters")

  args.output.mkdir(parents=True, exist_ok=True)
  clean.to_parquet(args.output / "clean_with_provenance.parquet", index=False)
  if not args.parquet_only:
    (args.output / "training_inputs").mkdir(exist_ok=True)
    (args.output / "heldout").mkdir(exist_ok=True)
    clean[required].to_csv(args.output / "training_inputs" / MODEL_NAME, index=False)

    routes = sorted(clean.route.unique())
    if len(routes) >= 3:
      for route in routes:
        identifier = safe_route_id(str(route))
        train = clean[clean.route != route]
        heldout = clean[clean.route == route]
        train[required].to_csv(
          args.output / "training_inputs" / f"{CAR_FINGERPRINT}_without_{identifier}.csv", index=False,
        )
        heldout.to_parquet(args.output / "heldout" / f"{identifier}.parquet", index=False)

  summary = summarize(clean, segment_metadata)
  summary["parquetOnly"] = args.parquet_only
  summary["sourceErrors"] = [item for item in segment_metadata if item.get("error")]
  (args.output / "dataset_manifest.json").write_text(json.dumps(summary, indent=2) + "\n")
  print(json.dumps(summary, indent=2))
  return 0 if summary["prototypeReady"] else 2


if __name__ == "__main__":
  raise SystemExit(main())
