import time

import pytest

from opendbc.car import structs
from opendbc.car.hyundai.radar_interface import (
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_BASE,
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE,
  RadarInterface,
  carnival_radar_object_valid,
  decode_carnival_confirmation_velocity,
  decode_carnival_radar_object,
)


def encode_velocity(value: float, bit_offset: int = 0) -> bytes:
  raw = round((value - 2.4) / 0.05)
  if raw < 0:
    raw += 1 << 11
  packed = (raw & ((1 << 11) - 1)) << (bit_offset + 91)
  return packed.to_bytes(32, "little")


def encode_object(track_id: int, heartbeat: int, distance: float, lateral: float,
                  velocity: float, bit_offset: int = 0) -> bytes:
  values = (
    (track_id, bit_offset + 42, 8),
    (heartbeat, bit_offset + 124, 4),
    (round(distance / 0.05), bit_offset + 64, 13),
    (round(lateral / 0.05), bit_offset + 78, 11),
    (round((velocity - 2.4) / 0.05), bit_offset + 91, 11),
  )
  packed = 0
  for value, start, size in values:
    packed |= (value & ((1 << size) - 1)) << start
  return packed.to_bytes(32, "little")


def make_probe() -> RadarInterface:
  probe = RadarInterface.__new__(RadarInterface)
  probe.carnival_object_probe_prev = {}
  probe.carnival_object_probe_last_log = time.monotonic()
  probe.carnival_object_probe_seen = 0
  probe.carnival_object_probe_valid = 0
  probe.carnival_confirmation_tracks = {}
  probe.carnival_confirmation_prev = {}
  probe.carnival_confirmation_persist = {}
  return probe


def test_carnival_r0100_relative_velocity_decode() -> None:
  for value in (-15.0, -5.0, 0.0, 2.4, 8.7):
    dat = encode_velocity(value)
    assert decode_carnival_confirmation_velocity(dat, 0, 4, 14) == pytest.approx(value)


def test_carnival_r0100_second_slot_relative_velocity_decode() -> None:
  dat = encode_velocity(-1.25, 128)
  assert decode_carnival_confirmation_velocity(dat, 128, 3, 8) == pytest.approx(-1.25)


def test_carnival_r0100_full_object_decode_both_slots() -> None:
  first = int.from_bytes(encode_object(17, 9, 42.5, -1.25, -3.4), "little")
  second = int.from_bytes(encode_object(91, 4, 88.0, 2.15, 7.3, 128), "little")
  dat = (first | second).to_bytes(32, "little")

  obj1 = decode_carnival_radar_object(dat, 0)
  obj2 = decode_carnival_radar_object(dat, 128)

  assert (obj1.raw_track_id, obj1.heartbeat) == (17, 9)
  assert (obj2.raw_track_id, obj2.heartbeat) == (91, 4)
  assert (obj1.d_rel, obj1.y_rel, obj1.v_rel) == pytest.approx((42.5, -1.25, -3.4))
  assert (obj2.d_rel, obj2.y_rel, obj2.v_rel) == pytest.approx((88.0, 2.15, 7.3))
  assert carnival_radar_object_valid(obj1)
  assert carnival_radar_object_valid(obj2)


@pytest.mark.parametrize(("track_id", "heartbeat"), ((0, 3), (12, 0)))
def test_carnival_radar_rejects_invalid_identity(track_id: int, heartbeat: int) -> None:
  obj = decode_carnival_radar_object(encode_object(track_id, heartbeat, 20.0, 0.0, 0.0), 0)
  assert not carnival_radar_object_valid(obj)


def test_carnival_radar_requires_persistence_and_publishes_stable_id() -> None:
  probe = make_probe()
  frame = encode_object(37, 6, 31.0, -0.45, -2.0)
  can_strings = [(0, [(0x180, frame, 1)])]

  probe._update_carnival_object_probe(can_strings)
  probe._update_carnival_object_probe(can_strings)
  assert probe.carnival_confirmation_tracks == {}
  probe._update_carnival_object_probe(can_strings)

  rr = structs.RadarData()
  probe._add_carnival_confirmation_tracks(rr)
  assert len(rr.points) == 1
  assert rr.points[0].trackId == CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_ID_BASE + 37
  assert (rr.points[0].dRel, rr.points[0].yRel, rr.points[0].vRel) == pytest.approx((31.0, -0.45, -2.0))


def test_carnival_radar_expires_stale_tracks() -> None:
  probe = make_probe()
  now = time.monotonic()
  probe.carnival_confirmation_tracks[12] = (
    now - CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE - 0.01,
    20.0,
    0.0,
    0.0,
  )
  probe.carnival_confirmation_prev[12] = (now, 20.0, 0.0, 0.0)
  probe.carnival_confirmation_persist[12] = 4

  probe._expire_carnival_confirmation_tracks(now)

  assert probe.carnival_confirmation_tracks == {}
  assert probe.carnival_confirmation_prev == {}
  assert probe.carnival_confirmation_persist == {}
