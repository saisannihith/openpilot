import random
import time
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import messaging, log, car
from openpilot.selfdrive.locationd.lagd import LateralLagEstimator, LongitudinalLagEstimator, retrieve_initial_lag, \
                                               retrieve_initial_longitudinal_lag, masked_normalized_cross_correlation, \
                                               longitudinal_delay_window_estimate, \
                                               BLOCK_NUM_NEEDED, BLOCK_SIZE, MIN_OKAY_WINDOW_SEC, MAX_LAG
from openpilot.selfdrive.test.process_replay.migration import migrate, migrate_carParams
from openpilot.selfdrive.locationd.test.test_locationd_scenarios import TEST_ROUTE
from openpilot.common.params import Params
from openpilot.tools.lib.logreader import LogReader
from openpilot.system.hardware import PC

MAX_ERR_FRAMES = 1
DT = 0.05
LAGD_MAX_LAG_FRAMES = int(round(MAX_LAG / DT))


def process_messages(estimator, lag_frames, n_frames, vego=20.0, rejection_threshold=0.0):
  for i in range(n_frames):
    t = i * estimator.dt
    desired_la = np.cos(10 * t) * 0.3
    actual_la = np.cos(10 * (t - lag_frames * estimator.dt)) * 0.3

    # if sample is masked out, set it to desired value (no lag)
    rejected = random.uniform(0, 1) < rejection_threshold
    if rejected:
      actual_la = desired_la

    desired_cuvature = float(desired_la / (vego ** 2))
    actual_yr = float(actual_la / vego)
    msgs = [
      (t, "carControl", car.CarControl(latActive=not rejected)),
      (t, "carState", car.CarState(vEgo=vego, steeringPressed=False)),
      (t, "controlsState", log.ControlsState(desiredCurvature=desired_cuvature)),
      (t, "livePose", log.LivePose(angularVelocityDevice=log.LivePose.XYZMeasurement(z=actual_yr, valid=True),
                                   posenetOK=True, inputsOK=True)),
      (t, "liveCalibration", log.LiveCalibrationData(rpyCalib=[0, 0, 0], calStatus=log.LiveCalibrationData.Status.calibrated)),
    ]
    for t, w, m in msgs:
      estimator.handle_log(t, w, m)
    estimator.update_points()
    estimator.update_estimate()


def process_longitudinal_messages(estimator, lag_frames, n_frames, vego=20.0, pedal_override=False):
  levels = np.array([0.7, -0.45, 0.3, -0.7, 0.55, -0.25, 0.8, -0.5, 0.35,
                     -0.65, 0.45, -0.3, 0.75, -0.4, 0.25, -0.75, 0.5, -0.3, 0.65])

  def command_at(t):
    return float(levels[int(max(t, 0.0) / 0.65) % len(levels)] + 0.08 * np.sin(2.7 * t))

  for i in range(n_frames):
    t = i * estimator.dt
    command = command_at(t)
    actual = command_at(t - lag_frames * estimator.dt)
    cc = car.CarControl(longActive=True)
    cc.actuators.accel = command
    cs = car.CarState(vEgo=vego, aEgo=actual, steeringAngleDeg=0.0,
                      gasPressed=pedal_override, brakePressed=False, standstill=False)
    estimator.handle_log(t, "carControl", cc)
    estimator.handle_log(t, "carState", cs)
    estimator.update_points()
    estimator.update_estimate()


class TestLagd:
  def test_manual_delay_uses_exact_configured_value(self):
    mocked_CP = car.CarParams(steerActuatorDelay=0.11)
    estimator = LateralLagEstimator(mocked_CP, DT)
    estimator.starpilot_toggles = SimpleNamespace(
      use_custom_steerActuatorDelay=True,
      steerActuatorDelay=0.30,
    )

    msg = estimator.get_msg(True)

    assert msg.liveDelay.lateralDelay == pytest.approx(0.30)

  def test_read_saved_params(self, tmp_path):
    params = Params(str(tmp_path))

    lr = migrate(LogReader(TEST_ROUTE), [migrate_carParams])
    CP = next(m for m in lr if m.which() == "carParams").carParams

    msg = messaging.new_message('liveDelay')
    msg.liveDelay.lateralDelayEstimate = random.random()
    msg.liveDelay.validBlocks = random.randint(1, 10)
    params.put("LiveDelay", msg.to_bytes())
    params.put("CarParamsPrevRoute", CP.as_builder().to_bytes())

    saved_lag_params = retrieve_initial_lag(params, CP)
    assert saved_lag_params is not None

    lag, valid_blocks = saved_lag_params
    assert lag == msg.liveDelay.lateralDelayEstimate
    assert valid_blocks == msg.liveDelay.validBlocks

  def test_ncc(self):
    lag_frames = random.randint(1, 19)

    desired_sig = np.sin(np.arange(0.0, 10.0, 0.1))
    actual_sig = np.sin(np.arange(0.0, 10.0, 0.1) - lag_frames * 0.1)
    mask = np.ones(len(desired_sig), dtype=bool)

    corr = masked_normalized_cross_correlation(desired_sig, actual_sig, mask, 200)[len(desired_sig) - 1:len(desired_sig) + 20]
    assert np.argmax(corr) == lag_frames

    # add some noise
    desired_sig += np.random.normal(0, 0.05, len(desired_sig))
    actual_sig += np.random.normal(0, 0.05, len(actual_sig))
    corr = masked_normalized_cross_correlation(desired_sig, actual_sig, mask, 200)[len(desired_sig) - 1:len(desired_sig) + 20]
    assert np.argmax(corr)  in range(lag_frames - MAX_ERR_FRAMES, lag_frames + MAX_ERR_FRAMES + 1)

    # mask out 40% of the values, and make them noise
    mask = np.random.choice([True, False], size=len(desired_sig), p=[0.6, 0.4])
    desired_sig[~mask] = np.random.normal(0, 1, size=np.sum(~mask))
    actual_sig[~mask] = np.random.normal(0, 1, size=np.sum(~mask))
    corr = masked_normalized_cross_correlation(desired_sig, actual_sig, mask, 200)[len(desired_sig) - 1:len(desired_sig) + 20]
    assert np.argmax(corr) in range(lag_frames - MAX_ERR_FRAMES, lag_frames + MAX_ERR_FRAMES + 1)

  def test_empty_estimator(self):
    mocked_CP = car.CarParams(steerActuatorDelay=0.5)
    estimator = LateralLagEstimator(mocked_CP, DT)
    estimator.starpilot_toggles = SimpleNamespace(use_custom_steerActuatorDelay=False)
    msg = estimator.get_msg(True)
    assert msg.liveDelay.status == 'unestimated'
    assert np.allclose(msg.liveDelay.lateralDelay, estimator.initial_lag)
    assert np.allclose(msg.liveDelay.lateralDelayEstimate, estimator.initial_lag)
    assert msg.liveDelay.validBlocks == 0
    assert msg.liveDelay.calPerc == 0

  def test_longitudinal_estimator_and_pedal_rejection(self):
    mocked_CP = car.CarParams(
      carFingerprint="KIA_CARNIVAL_4TH_GEN", openpilotLongitudinalControl=True,
      longitudinalActuatorDelay=0.5,
    )
    lag_frames = 7
    estimator = LongitudinalLagEstimator(
      mocked_CP, DT, min_recovery_buffer_sec=0.0, min_valid_block_count=1,
      block_size=1, okay_window_sec=8.0, min_ncc=0.6,
    )
    process_longitudinal_messages(estimator, lag_frames, int(10.0 / DT) + 20)
    msg = messaging.new_message('liveDelay')
    estimator.apply_to_msg(msg.liveDelay)
    assert msg.liveDelay.longitudinalStatus == 'estimated'
    assert msg.liveDelay.longitudinalDelay == pytest.approx(lag_frames * DT, abs=0.03)

    rejected = LongitudinalLagEstimator(
      mocked_CP, DT, min_recovery_buffer_sec=0.0, min_valid_block_count=1,
      block_size=2, okay_window_sec=2.0,
    )
    process_longitudinal_messages(rejected, lag_frames, int(5.0 / DT), pedal_override=True)
    rejected_msg = messaging.new_message('liveDelay')
    rejected.apply_to_msg(rejected_msg.liveDelay)
    assert rejected_msg.liveDelay.longitudinalStatus == 'unestimated'
    assert rejected_msg.liveDelay.longitudinalDelay == pytest.approx(0.5)

  def test_longitudinal_estimator_rejects_broad_periodic_peak(self):
    t = np.arange(0.0, 12.0, DT)
    lag = 7 * DT
    command = 0.7 * np.sin(0.8 * t) + 0.25 * np.sin(2.1 * t)
    actual = 0.7 * np.sin(0.8 * (t - lag)) + 0.25 * np.sin(2.1 * (t - lag))
    assert longitudinal_delay_window_estimate(command, actual, DT) is None

  def test_read_saved_longitudinal_params(self, tmp_path):
    params = Params(str(tmp_path))
    CP = car.CarParams(carFingerprint="KIA_CARNIVAL_4TH_GEN", longitudinalActuatorDelay=0.5)
    msg = messaging.new_message('liveDelay')
    msg.liveDelay.longitudinalDelayEstimate = 0.42
    msg.liveDelay.longitudinalValidBlocks = 3
    msg.liveDelay.longitudinalStatus = 'estimated'
    msg.liveDelay.longitudinalEstimatorVersion = 2
    params.put("LiveDelay", msg.to_bytes())
    params.put("CarParamsPrevRoute", CP.to_bytes())
    assert retrieve_initial_longitudinal_lag(params, CP) == pytest.approx((0.42, 3))

  def test_reject_saved_longitudinal_params_from_old_estimator(self, tmp_path):
    params = Params(str(tmp_path))
    CP = car.CarParams(carFingerprint="KIA_CARNIVAL_4TH_GEN", longitudinalActuatorDelay=0.5)
    msg = messaging.new_message('liveDelay')
    msg.liveDelay.longitudinalDelayEstimate = 0.42
    msg.liveDelay.longitudinalValidBlocks = 3
    msg.liveDelay.longitudinalStatus = 'estimated'
    params.put("LiveDelay", msg.to_bytes())
    params.put("CarParamsPrevRoute", CP.to_bytes())
    assert retrieve_initial_longitudinal_lag(params, CP) is None

  def test_estimator_basics(self, subtests):
    for lag_frames in range(LAGD_MAX_LAG_FRAMES - 1):
      with subtests.test(msg=f"lag_frames={lag_frames}"):
        mocked_CP = car.CarParams(steerActuatorDelay=0.5)
        estimator = LateralLagEstimator(mocked_CP, DT, min_recovery_buffer_sec=0.0, min_yr=0.0)
        estimator.starpilot_toggles = SimpleNamespace(use_custom_steerActuatorDelay=False)
        process_messages(estimator, lag_frames, int(MIN_OKAY_WINDOW_SEC / DT) + BLOCK_NUM_NEEDED * BLOCK_SIZE)
        msg = estimator.get_msg(True)
        assert msg.liveDelay.status == 'estimated'
        assert np.allclose(msg.liveDelay.lateralDelay, lag_frames * DT, atol=0.01)
        assert np.allclose(msg.liveDelay.lateralDelayEstimate, lag_frames * DT, atol=0.01)
        assert np.allclose(msg.liveDelay.lateralDelayEstimateStd, 0.0, atol=0.01)
        assert msg.liveDelay.validBlocks == BLOCK_NUM_NEEDED
        assert msg.liveDelay.calPerc == 100

  def test_estimator_masking(self):
    mocked_CP = car.CarParams(steerActuatorDelay=0.5)
    lag_frames = random.randint(1, LAGD_MAX_LAG_FRAMES - 1)
    estimator = LateralLagEstimator(mocked_CP, DT, min_recovery_buffer_sec=0.0, min_yr=0.0, min_valid_block_count=1)
    estimator.starpilot_toggles = SimpleNamespace(use_custom_steerActuatorDelay=False)
    process_messages(estimator, lag_frames, (int(MIN_OKAY_WINDOW_SEC / DT) + BLOCK_SIZE) * 2, rejection_threshold=0.4)
    msg = estimator.get_msg(True)
    assert np.allclose(msg.liveDelay.lateralDelayEstimate, lag_frames * DT, atol=0.01)
    assert np.allclose(msg.liveDelay.lateralDelayEstimateStd, 0.0, atol=0.01)
    assert msg.liveDelay.calPerc == 100

  @pytest.mark.skipif(PC, reason="only on device")
  @pytest.mark.timeout(60)
  def test_estimator_performance(self):
    mocked_CP = car.CarParams(steerActuatorDelay=0.5)
    estimator = LateralLagEstimator(mocked_CP, DT)

    ds = []
    for _ in range(1000):
      st = time.perf_counter()
      estimator.update_points()
      estimator.update_estimate()
      d = time.perf_counter() - st
      ds.append(d)

    assert np.mean(ds) < DT
