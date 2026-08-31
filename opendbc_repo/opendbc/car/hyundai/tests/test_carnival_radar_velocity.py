import math
import time

import pytest

from opendbc.can.dbc import DBC as DBCReader
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai.radar_interface import (
  CARNIVAL_4TH_GEN_CONFIRMATION_TRACK_MAX_AGE,
  CARNIVAL_4TH_GEN_TRACK_ID_BASE,
  RadarInterface,
  carnival_radar_frame_checksum,
  carnival_radar_frame_valid,
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
                  velocity: float, bit_offset: int = 0, *, valid_count: int = 1,
                  state_alt: int = 0, state: int = 1, lateral_velocity: float = 0.0,
                  relative_acceleration: float = 0.0) -> bytes:
  values = (
    (valid_count, bit_offset + 32, 8),
    (track_id, bit_offset + 42, 8),
    (state_alt, bit_offset + 51, 4),
    (state, bit_offset + 55, 3),
    (heartbeat, bit_offset + 124, 4),
    (round(distance / 0.05), bit_offset + 64, 13),
    (round(lateral / 0.05), bit_offset + 78, 11),
    (round((velocity - 2.4) / 0.05), bit_offset + 91, 11),
    (round((lateral_velocity - 0.6) / 0.05), bit_offset + 104, 9),
    (round(relative_acceleration / 0.1), bit_offset + 115, 9),
  )
  packed = 0
  for value, start, size in values:
    packed |= (value & ((1 << size) - 1)) << start
  return packed.to_bytes(32, "little")


def with_frame_crc(dat: bytes, address: int = 0x180) -> bytes:
  result = bytearray(dat)
  result[:2] = carnival_radar_frame_checksum(address, result).to_bytes(2, "little")
  return bytes(result)


def make_probe() -> RadarInterface:
  probe = RadarInterface.__new__(RadarInterface)
  probe.carnival_object_probe_prev = {}
  probe.carnival_object_probe_last_log = time.monotonic()
  probe.carnival_object_probe_seen = 0
  probe.carnival_object_probe_valid = 0
  probe.carnival_object_probe_crc_invalid = 0
  probe.carnival_confirmation_tracks = {}
  probe.carnival_confirmation_prev = {}
  probe.carnival_confirmation_persist = {}
  return probe


def test_carnival_r0100_relative_velocity_decode() -> None:
  for value in (-15.0, -5.0, 0.0, 2.4, 8.7):
    dat = encode_velocity(value)
    assert decode_carnival_confirmation_velocity(dat, 0) == pytest.approx(value)


def test_carnival_r0100_second_slot_relative_velocity_decode() -> None:
  dat = encode_velocity(-1.25, 128)
  assert decode_carnival_confirmation_velocity(dat, 128) == pytest.approx(-1.25)


def test_carnival_r0100_full_object_decode_both_slots() -> None:
  first = int.from_bytes(encode_object(17, 9, 42.5, -1.25, -3.4,
                                       valid_count=73, state_alt=12, state=4,
                                       lateral_velocity=-1.4, relative_acceleration=-2.3), "little")
  second = int.from_bytes(encode_object(91, 4, 88.0, 2.15, 7.3, 128,
                                        valid_count=46, state_alt=6, state=3,
                                        lateral_velocity=2.6, relative_acceleration=1.7), "little")
  dat = (first | second).to_bytes(32, "little")

  obj1 = decode_carnival_radar_object(dat, 0)
  obj2 = decode_carnival_radar_object(dat, 128)

  assert (obj1.raw_track_id, obj1.heartbeat) == (17, 9)
  assert (obj2.raw_track_id, obj2.heartbeat) == (91, 4)
  assert obj1.valid_count == 73
  assert obj2.valid_count == 46
  assert (obj1.d_rel, obj1.y_rel, obj1.v_rel) == pytest.approx((42.5, -1.25, -3.4))
  assert (obj2.d_rel, obj2.y_rel, obj2.v_rel) == pytest.approx((88.0, 2.15, 7.3))
  assert (obj1.yv_rel, obj1.a_rel) == pytest.approx((-1.4, -2.3))
  assert (obj2.yv_rel, obj2.a_rel) == pytest.approx((2.6, 1.7))
  assert carnival_radar_object_valid(obj1)
  assert carnival_radar_object_valid(obj2)


def test_carnival_r0100_decoder_matches_documented_dbc() -> None:
  first = int.from_bytes(encode_object(17, 9, 42.5, -1.25, -3.4,
                                       valid_count=73, lateral_velocity=-1.4,
                                       relative_acceleration=-2.3), "little")
  second = int.from_bytes(encode_object(91, 4, 88.0, 2.15, 7.3, 128,
                                        valid_count=46, lateral_velocity=2.6,
                                        relative_acceleration=1.7), "little")
  dat = with_frame_crc((first | second).to_bytes(32, "little"))
  dbc = DBCReader("hyundai_r0100_radar_generated")
  signals = dbc.addr_to_msg[0x180].sigs

  def physical(name: str) -> float:
    signal = signals[name]
    raw = get_raw_value(dat, signal)
    if signal.is_signed:
      raw -= ((raw >> (signal.size - 1)) & 1) * (1 << signal.size)
    return raw * signal.factor + signal.offset

  obj1 = decode_carnival_radar_object(dat, 0)
  obj2 = decode_carnival_radar_object(dat, 128)
  assert physical("1_VALID_CNT") == obj1.valid_count
  assert physical("2_VALID_CNT") == obj2.valid_count
  assert physical("1_TRACK_ID") == obj1.raw_track_id
  assert physical("2_TRACK_ID") == obj2.raw_track_id
  assert (physical("1_LONG_DIST"), physical("1_LAT_DIST"), physical("1_REL_SPEED")) == pytest.approx(
    (obj1.d_rel, obj1.y_rel, obj1.v_rel)
  )
  assert (physical("2_LONG_DIST"), physical("2_LAT_DIST"), physical("2_REL_SPEED")) == pytest.approx(
    (obj2.d_rel, obj2.y_rel, obj2.v_rel)
  )


def test_carnival_r0100_frame_checksum_rejects_corruption() -> None:
  frame = with_frame_crc(encode_object(17, 9, 42.5, -1.25, -3.4))
  assert carnival_radar_frame_valid(0x180, frame)
  corrupted = bytearray(frame)
  corrupted[12] ^= 0x01
  assert not carnival_radar_frame_valid(0x180, corrupted)
  assert not carnival_radar_frame_valid(0x181, frame)


def test_carnival_r0100_probe_drops_bad_checksum_before_decode() -> None:
  probe = make_probe()
  frame = bytearray(with_frame_crc(encode_object(17, 9, 42.5, -1.25, -3.4)))
  frame[12] ^= 0x01
  for _ in range(3):
    probe._update_carnival_object_probe([(0, [(0x180, bytes(frame), 1)])])
  assert probe.carnival_confirmation_tracks == {}
  assert probe.carnival_object_probe_seen == 3
  assert probe.carnival_object_probe_valid == 0
  assert probe.carnival_object_probe_crc_invalid == 3


def test_carnival_radar_rejects_zero_track_id() -> None:
  obj = decode_carnival_radar_object(encode_object(0, 3, 20.0, 0.0, 0.0), 0)
  assert not carnival_radar_object_valid(obj)


def test_carnival_radar_rejects_zero_valid_count() -> None:
  obj = decode_carnival_radar_object(
    encode_object(12, 3, 20.0, 0.0, 0.0, valid_count=0), 0,
  )
  assert not carnival_radar_object_valid(obj)


@pytest.mark.parametrize("state", range(8))
def test_carnival_radar_does_not_gate_unverified_state_bits(state: int) -> None:
  obj = decode_carnival_radar_object(encode_object(12, 3, 20.0, 0.0, 0.0, state=state), 0)
  assert carnival_radar_object_valid(obj)


def test_carnival_radar_accepts_complete_rolling_counter_cycle() -> None:
  probe = make_probe()
  for heartbeat in range(16):
    distance = 20.0 + 0.05 * heartbeat
    frame = [(0, [(0x180, with_frame_crc(encode_object(12, heartbeat, distance, 0.0, -0.5)), 1)])]
    probe._update_carnival_object_probe(frame)

  rr = structs.RadarData()
  probe._add_carnival_confirmation_tracks(rr)
  assert len(rr.points) == 1
  assert rr.points[0].trackId == CARNIVAL_4TH_GEN_TRACK_ID_BASE + 12
  assert rr.points[0].dRel == pytest.approx(20.75)


def test_carnival_radar_requires_persistence_and_publishes_stable_id() -> None:
  probe = make_probe()
  frame = with_frame_crc(encode_object(37, 6, 31.0, -0.45, -2.0))
  can_strings = [(0, [(0x180, frame, 1)])]

  probe._update_carnival_object_probe(can_strings)
  probe._update_carnival_object_probe(can_strings)
  assert probe.carnival_confirmation_tracks == {}
  probe._update_carnival_object_probe(can_strings)

  rr = structs.RadarData()
  probe._add_carnival_confirmation_tracks(rr)
  assert len(rr.points) == 1
  assert rr.points[0].trackId == CARNIVAL_4TH_GEN_TRACK_ID_BASE + 37
  assert (rr.points[0].dRel, rr.points[0].yRel, rr.points[0].vRel) == pytest.approx((31.0, -0.45, -2.0))
  assert math.isnan(rr.points[0].yvRel)
  assert math.isnan(rr.points[0].aRel)


def test_carnival_radar_recycled_id_must_requalify() -> None:
  probe = make_probe()
  first = [(0, [(0x180, with_frame_crc(encode_object(37, 6, 31.0, -0.45, -2.0)), 1)])]
  replacement = [(0, [(0x180, with_frame_crc(encode_object(37, 7, 55.0, 3.0, 4.0)), 1)])]

  for _ in range(3):
    probe._update_carnival_object_probe(first)
  assert 37 in probe.carnival_confirmation_tracks

  probe._update_carnival_object_probe(replacement)
  assert 37 not in probe.carnival_confirmation_tracks
  probe._update_carnival_object_probe(replacement)
  assert 37 not in probe.carnival_confirmation_tracks
  probe._update_carnival_object_probe(replacement)
  assert probe.carnival_confirmation_tracks[37][1:4] == pytest.approx((55.0, 3.0, 4.0))


def test_carnival_valid_count_saturation_does_not_reclassify_track() -> None:
  probe = make_probe()
  before_saturation = [(0, [(0x180, with_frame_crc(encode_object(37, 6, 31.0, -0.45, -2.0, valid_count=254)), 1)])]
  saturated = [(0, [(0x180, with_frame_crc(encode_object(37, 7, 30.95, -0.45, -2.0, valid_count=255)), 1)])]

  for _ in range(3):
    probe._update_carnival_object_probe(before_saturation)
  rr = structs.RadarData()
  probe._add_carnival_confirmation_tracks(rr)
  assert rr.points[0].trackId == CARNIVAL_4TH_GEN_TRACK_ID_BASE + 37

  probe._update_carnival_object_probe(saturated)
  assert 37 in probe.carnival_confirmation_tracks
  rr = structs.RadarData()
  probe._add_carnival_confirmation_tracks(rr)
  assert rr.points[0].trackId == CARNIVAL_4TH_GEN_TRACK_ID_BASE + 37


def test_carnival_both_slots_use_the_same_track_contract() -> None:
  probe = make_probe()
  first = int.from_bytes(encode_object(21, 5, 22.0, 0.0, -1.0, valid_count=20), "little")
  second = int.from_bytes(encode_object(22, 5, 24.0, 0.2, -1.0, 128, valid_count=255), "little")
  frame = [(0, [(0x180, with_frame_crc((first | second).to_bytes(32, "little")), 1)])]

  for _ in range(3):
    probe._update_carnival_object_probe(frame)
  rr = structs.RadarData()
  probe._add_carnival_confirmation_tracks(rr)
  ids = {point.trackId for point in rr.points}
  assert CARNIVAL_4TH_GEN_TRACK_ID_BASE + 21 in ids
  assert CARNIVAL_4TH_GEN_TRACK_ID_BASE + 22 in ids


def test_carnival_radar_conflicting_id_is_withdrawn() -> None:
  probe = make_probe()
  first = [(0, [(0x180, with_frame_crc(encode_object(37, 6, 31.0, -0.45, -2.0)), 1)])]
  for _ in range(3):
    probe._update_carnival_object_probe(first)
  assert 37 in probe.carnival_confirmation_tracks

  near = int.from_bytes(encode_object(37, 7, 30.8, -0.4, -2.0), "little")
  far = int.from_bytes(encode_object(37, 8, 70.0, 4.0, 5.0, 128), "little")
  probe._update_carnival_object_probe([(0, [(0x180, with_frame_crc((near | far).to_bytes(32, "little")), 1)])])
  assert 37 not in probe.carnival_confirmation_tracks


def test_carnival_radar_publishes_complete_ten_object_bank() -> None:
  probe = make_probe()
  frames = []
  expected = {}
  for address_index in range(5):
    first_id = 20 + address_index * 2
    second_id = first_id + 1
    first_values = (15.0 + address_index, -0.5 + address_index * 0.1, -2.0 + address_index * 0.2)
    second_values = (35.0 + address_index, 0.6 - address_index * 0.1, 1.0 + address_index * 0.2)
    first = int.from_bytes(encode_object(first_id, 5, *first_values), "little")
    second = int.from_bytes(encode_object(second_id, 7, *second_values, 128), "little")
    address = 0x180 + address_index
    frames.append((address, with_frame_crc((first | second).to_bytes(32, "little"), address), 1))
    expected[first_id] = first_values
    expected[second_id] = second_values

  can_strings = [(0, frames)]
  for _ in range(3):
    probe._update_carnival_object_probe(can_strings)

  rr = structs.RadarData()
  probe._add_carnival_confirmation_tracks(rr)
  actual = {
    point.trackId - CARNIVAL_4TH_GEN_TRACK_ID_BASE: (point.dRel, point.yRel, point.vRel)
    for point in rr.points
  }
  assert actual.keys() == expected.keys()
  for track_id, values in expected.items():
    assert actual[track_id] == pytest.approx(values)


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
