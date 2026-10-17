"""Verified receive-only R0100 object bank on the 2022-24 Carnival."""

from dataclasses import dataclass
import time

from opendbc.car import structs
from opendbc.car.crc import CRC16_XMODEM


START_ADDR = 0x180
END_ADDR = 0x184
BUS = 1
FRAME_LEN = 32
TRACK_ID_BASE = 0xC4100
MAX_AGE_S = 0.25
MAX_GAP_S = 0.15
MIN_PERSIST = 3
CHECKSUM_XOR = 0x9F5B


def unsigned(dat: bytes, start: int, size: int) -> int:
  return (int.from_bytes(dat, "little") >> start) & ((1 << size) - 1)


def signed(dat: bytes, start: int, size: int) -> int:
  value = unsigned(dat, start, size)
  return value - (1 << size) if value & (1 << (size - 1)) else value


def frame_checksum(address: int, dat: bytes) -> int:
  crc = 0
  for value in (*dat[2:], address & 0xFF, (address >> 8) & 0xFF):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ value]) & 0xFFFF
  return crc ^ CHECKSUM_XOR


def frame_valid(address: int, dat: bytes) -> bool:
  return len(dat) == FRAME_LEN and int.from_bytes(dat[:2], "little") == frame_checksum(address, dat)


@dataclass(frozen=True)
class Object:
  raw_track_id: int
  valid_count: int
  d_rel: float
  y_rel: float
  v_rel: float


def decode_object(dat: bytes, bit_offset: int) -> Object:
  return Object(
    raw_track_id=unsigned(dat, bit_offset + 42, 8),
    valid_count=unsigned(dat, bit_offset + 32, 8),
    d_rel=unsigned(dat, bit_offset + 64, 13) * 0.05,
    y_rel=signed(dat, bit_offset + 78, 11) * 0.05,
    v_rel=signed(dat, bit_offset + 91, 11) * 0.05 + 2.4,
  )


def object_valid(obj: Object) -> bool:
  return (obj.valid_count != 0 and obj.raw_track_id != 0 and
          0.5 <= obj.d_rel <= 220.0 and abs(obj.y_rel) <= 50.0 and abs(obj.v_rel) <= 60.0)


class CarnivalR0100:
  def __init__(self):
    self.previous: dict[int, tuple[float, float, float, float]] = {}
    self.persistence: dict[int, int] = {}
    self.tracks: dict[int, tuple[float, float, float, float]] = {}

  def update(self, can_packets, now: float | None = None) -> None:
    now = time.monotonic() if now is None else now
    batch: dict[int, Object] = {}
    conflicting: set[int] = set()
    for _, frames in can_packets:
      for address, dat, bus in frames:
        if bus != BUS or not START_ADDR <= address <= END_ADDR or not frame_valid(address, dat):
          continue
        for bit_offset in (0, 128):
          obj = decode_object(dat, bit_offset)
          if not object_valid(obj):
            continue
          existing = batch.get(obj.raw_track_id)
          if existing is not None and existing != obj:
            conflicting.add(obj.raw_track_id)
          else:
            batch[obj.raw_track_id] = obj

    for raw_id in conflicting:
      batch.pop(raw_id, None)
      self.previous.pop(raw_id, None)
      self.persistence.pop(raw_id, None)
      self.tracks.pop(raw_id, None)

    for raw_id, obj in batch.items():
      previous = self.previous.get(raw_id)
      continuous = False
      if previous is not None:
        previous_time, previous_d, previous_y, previous_v = previous
        dt = now - previous_time
        continuous = (0.0 <= dt <= MAX_GAP_S and
                      abs(obj.d_rel - previous_d) <= max(1.5, 60.0 * dt) and
                      abs(obj.y_rel - previous_y) <= max(1.0, 20.0 * dt) and
                      abs(obj.v_rel - previous_v) <= 8.0)
      count = self.persistence.get(raw_id, 0) + 1 if continuous else 1
      self.previous[raw_id] = (now, obj.d_rel, obj.y_rel, obj.v_rel)
      self.persistence[raw_id] = count
      if count >= MIN_PERSIST:
        self.tracks[raw_id] = (now, obj.d_rel, obj.y_rel, obj.v_rel)
      else:
        self.tracks.pop(raw_id, None)

    for raw_id, (sample_time, *_) in tuple(self.previous.items()):
      if now - sample_time > MAX_AGE_S:
        self.previous.pop(raw_id, None)
        self.persistence.pop(raw_id, None)
        self.tracks.pop(raw_id, None)

  def add_points(self, radar_data: structs.RadarData, now: float | None = None) -> None:
    now = time.monotonic() if now is None else now
    points = list(radar_data.points)
    for raw_id, (sample_time, distance, lateral, velocity) in sorted(self.tracks.items()):
      if now - sample_time > MAX_AGE_S:
        continue
      point = structs.RadarData.RadarPoint()
      point.trackId = TRACK_ID_BASE + raw_id
      point.dRel = distance
      point.yRel = lateral
      point.vRel = velocity
      points.append(point)
    radar_data.points = points
