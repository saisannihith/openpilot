from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.hyundai.carcontroller import CARNIVAL_DRIVER_CONFLICT_HOLD_FRAMES, update_carnival_driver_conflict_hold
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


def test_carnival_driver_conflict_hold_reaches_neutral_without_exceeding_safety_rate():
  torque = 260
  hold_frames = 0

  outputs = []
  for _ in range(30):
    torque, hold_frames = update_carnival_driver_conflict_hold(
      CAR.KIA_CARNIVAL_4TH_GEN, torque, torque, -350, True, hold_frames,
    )
    outputs.append(torque)

  assert outputs[:3] == [250, 240, 230]
  assert outputs[-1] == 0
  assert hold_frames == CARNIVAL_DRIVER_CONFLICT_HOLD_FRAMES
  assert max(abs(current - previous) for previous, current in zip([260] + outputs[:-1], outputs, strict=True)) == 10


def test_carnival_driver_conflict_hold_covers_delayed_eps_fault_and_resumes_normally():
  torque, hold_frames = update_carnival_driver_conflict_hold(
    CAR.KIA_CARNIVAL_4TH_GEN, 110, 120, -350, True, 0,
  )
  assert (torque, hold_frames) == (110, CARNIVAL_DRIVER_CONFLICT_HOLD_FRAMES)

  for remaining in range(CARNIVAL_DRIVER_CONFLICT_HOLD_FRAMES - 1, 0, -1):
    torque, hold_frames = update_carnival_driver_conflict_hold(
      CAR.KIA_CARNIVAL_4TH_GEN, 2, torque, 0, True, hold_frames,
    )
    assert hold_frames == remaining
    assert torque == max(0, 110 - 10 * (CARNIVAL_DRIVER_CONFLICT_HOLD_FRAMES - remaining))

  torque, hold_frames = update_carnival_driver_conflict_hold(
    CAR.KIA_CARNIVAL_4TH_GEN, 2, torque, 0, True, hold_frames,
  )
  assert (torque, hold_frames) == (2, 0)


def test_carnival_driver_conflict_hold_does_not_touch_normal_or_other_car_commands():
  assert update_carnival_driver_conflict_hold(
    CAR.KIA_CARNIVAL_4TH_GEN, 409, 407, 0, True, 0,
  ) == (409, 0)
  assert update_carnival_driver_conflict_hold(
    CAR.KIA_CARNIVAL_4TH_GEN, 409, 407, 350, True, 0,
  ) == (409, 0)
  assert update_carnival_driver_conflict_hold(
    CAR.HYUNDAI_IONIQ_5, 409, 407, -350, True, 5,
  ) == (409, 0)


def test_canfd_mdps_status_fields_use_the_official_two_bit_positions():
  packer = CANPacker("hyundai_canfd_generated")
  _, data, _ = packer.make_can_msg("MDPS", 0, {
    "MDPS_WARNING_LAMP": 5,
    "MDPS_LKA_PLUGIN": 1,
    "MDPS_LKA_TOI_ACTIVE": 2,
    "MDPS_LKA_TOI_UNAVAILABLE": 3,
    "MDPS_LKA_TOI_FAULT": 1,
    "MDPS_LKA_FAIL": 2,
  })
  raw = int.from_bytes(data, byteorder="little")

  assert (raw >> 24) & 0x7 == 5
  assert (raw >> 46) & 0x3 == 1
  assert (raw >> 48) & 0x3 == 2
  assert (raw >> 50) & 0x3 == 3
  assert (raw >> 52) & 0x3 == 1
  assert (raw >> 54) & 0x3 == 2
