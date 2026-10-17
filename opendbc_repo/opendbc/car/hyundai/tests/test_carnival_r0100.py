from opendbc.car import structs
from opendbc.car.hyundai.carnival_r0100 import CarnivalR0100, decode_object, frame_checksum, frame_valid
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import CAR


def set_bits(data: bytearray, start: int, size: int, value: int) -> None:
  mask = ((1 << size) - 1) << start
  bits = (int.from_bytes(data, "little") & ~mask) | ((value & ((1 << size) - 1)) << start)
  data[:] = bits.to_bytes(len(data), "little")


def frame(raw_id: int = 7, distance: float = 20.0, lateral: float = -0.25, velocity: float = -1.0) -> bytes:
  data = bytearray(32)
  set_bits(data, 32, 8, 3)
  set_bits(data, 42, 8, raw_id)
  set_bits(data, 64, 13, round(distance / 0.05))
  set_bits(data, 78, 11, round(lateral / 0.05))
  set_bits(data, 91, 11, round((velocity - 2.4) / 0.05))
  data[:2] = frame_checksum(0x180, data).to_bytes(2, "little")
  return bytes(data)


def packet(data: bytes, bus: int = 1):
  return [(0, [(0x180, data, bus)])]


def test_crc_decode_and_persistence():
  data = frame()
  assert frame_valid(0x180, data)
  assert decode_object(data, 0).raw_track_id == 7
  assert not frame_valid(0x181, data)
  corrupt = bytearray(data)
  corrupt[12] ^= 1
  assert not frame_valid(0x180, corrupt)

  tracker = CarnivalR0100()
  for now in (1.0, 1.05, 1.10):
    tracker.update(packet(data), now)
  result = structs.RadarData()
  tracker.add_points(result, 1.10)
  assert len(result.points) == 1
  point = result.points[0]
  assert (point.trackId, point.dRel, point.yRel, point.vRel) == (0xC4107, 20.0, -0.25, -1.0)
  tracker.update([], 1.36)
  result = structs.RadarData()
  tracker.add_points(result, 1.36)
  assert len(result.points) == 0


def test_wrong_bus_crc_or_conflicting_identity_never_publish():
  tracker = CarnivalR0100()
  data = frame()
  for now in (1.0, 1.05, 1.10):
    tracker.update(packet(data, bus=0), now)
  assert tracker.tracks == {}
  for now in (1.15, 1.20, 1.25):
    tracker.update(packet(data), now)
  assert tracker.tracks
  conflict = [(0, [(0x180, data, 1), (0x180, frame(distance=21.0), 1)])]
  tracker.update(conflict, 1.30)
  assert tracker.tracks == {}


def test_carnival_interface_publishes_receive_only_tracks():
  cp = structs.CarParams()
  cp.carFingerprint = CAR.KIA_CARNIVAL_4TH_GEN
  cp.radarUnavailable = True
  interface = RadarInterface(cp)
  data = frame()
  result = None
  for _ in range(5):
    result = interface.update(packet(data))
  assert result is not None
  assert len(result.points) == 1
  assert result.points[0].trackId == 0xC4107
