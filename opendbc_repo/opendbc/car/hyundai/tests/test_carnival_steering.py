from opendbc.car.hyundai.carnival_steering import conflict_request, safety_rate_limited_unwind
from opendbc.car.hyundai.values import CAR


def test_opposing_driver_unwinds_within_panda_rate():
  previous = 180
  frames = 0
  for _ in range(20):
    requested, frames, active = conflict_request(CAR.KIA_CARNIVAL_4TH_GEN, 200, previous, -350, True, frames, False)
    assert requested == 0 and active
    applied = safety_rate_limited_unwind(0, previous, active)
    assert previous - applied <= 3
    previous = applied


def test_no_conflict_or_other_car_is_unchanged():
  assert conflict_request(CAR.KIA_CARNIVAL_4TH_GEN, 200, 180, 350, True, 0, False) == (200, 0, False)
  assert conflict_request(CAR.KIA_CARNIVAL_2025, 200, 180, -350, True, 0, False) == (200, 0, False)
  assert safety_rate_limited_unwind(100, 180, False) == 100


def test_conflict_clears_when_driver_releases():
  assert conflict_request(CAR.KIA_CARNIVAL_4TH_GEN, 200, 180, 0, True, 20, False) == (200, 0, False)
