from opendbc.car.hyundai.carcontroller import (
  CARNIVAL_4TH_GEN_EPS_GUARD_RELEASE_FRAMES,
  CARNIVAL_4TH_GEN_EPS_GUARD_TORQUE_FRACTION,
  CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES,
  CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_RELEASE_FRAMES,
  apply_carnival_4th_gen_eps_fault_guard,
  apply_carnival_4th_gen_eps_predictive_taper,
  apply_carnival_4th_gen_manual_turn_torque_guard,
)
from opendbc.car.hyundai.values import CAR


STEER_MAX = 409


def test_carnival_manual_turn_guard_yields_on_low_speed_driver_touch():
  apply_torque, active, guard_frames = apply_carnival_4th_gen_manual_turn_torque_guard(
    CAR.KIA_CARNIVAL_4TH_GEN,
    apply_torque=300,
    steer_max=STEER_MAX,
    v_ego=9.0,
    steering_angle_deg=40.0,
    steering_torque=5.0,
    steering_pressed=True,
    apply_torque_last=300,
  )

  assert active
  assert guard_frames > 0
  assert apply_torque == 300 - CARNIVAL_4TH_GEN_MANUAL_TURN_GUARD_RELEASE_FRAMES


def test_carnival_manual_turn_guard_ignores_no_driver_touch():
  apply_torque, active, guard_frames = apply_carnival_4th_gen_manual_turn_torque_guard(
    CAR.KIA_CARNIVAL_4TH_GEN,
    apply_torque=300,
    steer_max=STEER_MAX,
    v_ego=9.0,
    steering_angle_deg=40.0,
    steering_torque=5.0,
    steering_pressed=False,
    apply_torque_last=300,
  )

  assert not active
  assert guard_frames == 0
  assert apply_torque == 300


def test_carnival_eps_guard_caps_even_when_steering_pressed():
  high_torque_frames = 0
  guard_frames = 0
  apply_torque = 390

  for _ in range(CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES):
    apply_torque, high_torque_frames, guard_frames, active, near_limit = apply_carnival_4th_gen_eps_fault_guard(
      CAR.KIA_CARNIVAL_4TH_GEN,
      apply_torque=390,
      steer_max=STEER_MAX,
      v_ego=15.0,
      steering_angle_deg=12.0,
      lat_active=True,
      steering_pressed=True,
      apply_torque_last=390,
      high_torque_frames=high_torque_frames,
      guard_frames=guard_frames,
    )

  assert near_limit
  assert active
  assert high_torque_frames == CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES
  assert guard_frames > 0
  assert apply_torque == 390 - CARNIVAL_4TH_GEN_EPS_GUARD_RELEASE_FRAMES
  assert abs(apply_torque) / STEER_MAX < CARNIVAL_4TH_GEN_EPS_GUARD_TORQUE_FRACTION


def test_carnival_eps_guard_resets_when_lateral_inactive():
  apply_torque, high_torque_frames, guard_frames, active, near_limit = apply_carnival_4th_gen_eps_fault_guard(
    CAR.KIA_CARNIVAL_4TH_GEN, 390, STEER_MAX, 15.0, 12.0, False, True, 390, 12, 40,
  )
  assert apply_torque == 390
  assert high_torque_frames == 0
  assert guard_frames == 39
  assert not active
  assert not near_limit


def test_carnival_eps_predictive_taper_is_bounded_and_preemptive():
  apply_torque, risk = apply_carnival_4th_gen_eps_predictive_taper(
    CAR.KIA_CARNIVAL_4TH_GEN, 409, STEER_MAX, 24.0, 28.0, True,
    CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES - 2,
  )
  assert risk > 0.58
  assert round(STEER_MAX * 0.88) <= apply_torque < STEER_MAX


def test_carnival_eps_predictive_taper_does_not_touch_normal_torque():
  apply_torque, risk = apply_carnival_4th_gen_eps_predictive_taper(
    CAR.KIA_CARNIVAL_4TH_GEN, 240, STEER_MAX, 24.0, 8.0, True, 0,
  )
  assert risk < 0.58
  assert apply_torque == 240
