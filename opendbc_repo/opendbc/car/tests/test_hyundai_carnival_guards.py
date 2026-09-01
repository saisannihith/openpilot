from types import SimpleNamespace

from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags


def _torque_params(v_ego: float) -> CarControllerParams:
  CP = SimpleNamespace(
    flags=HyundaiFlags.CANFD,
    carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN,
  )
  return CarControllerParams(CP, v_ego)


def test_carnival_4th_gen_uses_low_speed_dynamic_torque_rates():
  params = _torque_params(14.99)

  assert params.STEER_MAX == 409
  assert params.STEER_THRESHOLD == 100
  assert params.STEER_DELTA_UP == 10
  assert params.STEER_DELTA_DOWN == 8
  assert params.STEER_DRIVER_DELTA_DOWN == 10


def test_carnival_4th_gen_uses_high_speed_dynamic_torque_rates():
  params = _torque_params(15.0)

  assert params.STEER_MAX == 409
  assert params.STEER_THRESHOLD == 100
  assert params.STEER_DELTA_UP == 2
  assert params.STEER_DELTA_DOWN == 3
  assert params.STEER_DRIVER_DELTA_DOWN == 10


def test_carnival_driver_conflict_uses_safety_retreat_rate_only_while_limited():
  params = _torque_params(30.0)

  assert apply_driver_steer_torque_limits(-409, -258, 400, params) == -248
  assert apply_driver_steer_torque_limits(0, -258, 0, params) == -255
