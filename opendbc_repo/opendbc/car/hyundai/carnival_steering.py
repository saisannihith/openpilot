"""Driver-conflict unwind for the 2022-24 Carnival torque steering system."""

from opendbc.car.hyundai.values import CAR


DRIVER_CONFLICT_TORQUE = 300
DRIVER_CONFLICT_MIN_COMMAND = 40
DRIVER_CONFLICT_HOLD_FRAMES = 20
CANFD_SAFETY_RATE_DOWN = 3


def conflict_request(fingerprint, requested: int, previous: int, driver_torque: float,
                     lat_active: bool, hold_frames: int, steer_fault_temporary: bool) -> tuple[int, int, bool]:
  if fingerprint != CAR.KIA_CARNIVAL_4TH_GEN or not lat_active:
    return requested, 0, False

  strong_override = abs(driver_torque) >= DRIVER_CONFLICT_TORQUE
  opposing = abs(previous) >= DRIVER_CONFLICT_MIN_COMMAND and previous * driver_torque < 0
  if strong_override and (hold_frames > 0 or opposing):
    hold_frames = DRIVER_CONFLICT_HOLD_FRAMES
  elif steer_fault_temporary and hold_frames > 0:
    hold_frames = DRIVER_CONFLICT_HOLD_FRAMES
  elif not strong_override:
    hold_frames = 0
  elif hold_frames > 0:
    hold_frames -= 1

  return (0 if hold_frames > 0 else requested), hold_frames, hold_frames > 0


def safety_rate_limited_unwind(applied: int, previous: int, active: bool) -> int:
  if not active:
    return applied
  return min(max(applied, previous - CANFD_SAFETY_RATE_DOWN), previous + CANFD_SAFETY_RATE_DOWN)
