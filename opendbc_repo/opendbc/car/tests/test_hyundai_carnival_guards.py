from types import SimpleNamespace

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


def test_carnival_4th_gen_uses_high_speed_dynamic_torque_rates():
  params = _torque_params(15.0)

  assert params.STEER_MAX == 409
  assert params.STEER_THRESHOLD == 100
  assert params.STEER_DELTA_UP == 2
  assert params.STEER_DELTA_DOWN == 3
