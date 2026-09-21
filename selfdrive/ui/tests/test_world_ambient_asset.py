import struct
from pathlib import Path


ASSET = Path(__file__).parents[2] / 'assets/world/carnival_aurora_ambient_v2.png'


def test_ambient_asset_is_a_bounded_wide_png():
  raw = ASSET.read_bytes()
  assert raw[:8] == b'\x89PNG\r\n\x1a\n'
  length, kind, width, height = struct.unpack('>I4sII', raw[8:24])
  assert kind == b'IHDR' and length == 13
  assert 1.8 <= width / height <= 2.1
  assert width <= 2160 and height <= 1080
  assert len(raw) <= 2_500_000
