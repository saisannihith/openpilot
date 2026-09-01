import numpy as np
import pytest

pd = pytest.importorskip("pandas")

from analysis.analyze_carnival_speed_dependent_torque import balanced_points, fit_tls, prepare_samples


def test_tls_recovers_physical_slope_and_offset():
  steer = np.linspace(-0.45, 0.45, 2000)
  frame = pd.DataFrame({
    "route": np.repeat(["route-a", "route-b"], 1000),
    "timestamp": np.arange(2000) * 0.01,
    "steer": steer,
    "response": 1.72 * steer - 0.015,
  })
  fit = fit_tls(frame)
  assert fit["latAccelFactor"] == pytest.approx(1.72, abs=1e-3)
  assert fit["latAccelOffset"] == pytest.approx(-0.015, abs=1e-3)


def test_balancing_caps_each_route_torque_bucket():
  frame = pd.DataFrame({
    "route": ["route-a"] * 1000,
    "timestamp": np.arange(1000) * 0.01,
    "steer": [0.15] * 1000,
    "response": [0.2] * 1000,
  })
  assert len(balanced_points(frame)) == 300


def test_prepare_samples_requires_continuous_clean_context():
  timestamp = np.r_[np.arange(0.0, 2.51, 0.01), np.arange(3.0, 5.51, 0.01)]
  frame = pd.DataFrame({
    "route": ["route-a"] * len(timestamp),
    "timestamp": timestamp,
    "v_ego": [20.0] * len(timestamp),
    "applied_torque": [0.1] * len(timestamp),
    "lateral_delay": [0.36] * len(timestamp),
    "pose_valid": [True] * len(timestamp),
    "pose_lateral_accel_tp03": [-0.16] * len(timestamp),
    "pose_lateral_accel_tp06": [-0.16] * len(timestamp),
  })
  prepared = prepare_samples(frame)
  assert prepared["timestamp"].min() >= 2.0
  assert not prepared["timestamp"].between(3.0, 4.999).any()
