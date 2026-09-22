#!/usr/bin/env python3
"""Bake the authored asphalt PNG into a compact, device-safe NumPy asset."""
import argparse
import struct
import zlib
from pathlib import Path

import numpy as np


PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def read_png_rgb(path: Path) -> np.ndarray:
  """Read an 8-bit, non-interlaced RGB/RGBA PNG without runtime dependencies."""
  data = path.read_bytes()
  if not data.startswith(PNG_SIGNATURE):
    raise ValueError("Expected a PNG asset")
  cursor, width, height, bit_depth, color_type, interlace = len(PNG_SIGNATURE), 0, 0, 0, 0, 0
  chunks = []
  while cursor < len(data):
    length = struct.unpack_from(">I", data, cursor)[0]
    kind = data[cursor + 4:cursor + 8]
    payload = data[cursor + 8:cursor + 8 + length]
    cursor += 12 + length
    if kind == b"IHDR":
      width, height, bit_depth, color_type, _, _, interlace = struct.unpack(">IIBBBBB", payload)
    elif kind == b"IDAT":
      chunks.append(payload)
    elif kind == b"IEND":
      break
  if not width or bit_depth != 8 or color_type not in (2, 6) or interlace:
    raise ValueError("Expected non-interlaced 8-bit RGB or RGBA PNG")
  channels = 4 if color_type == 6 else 3
  stride = width * channels
  packed = zlib.decompress(b"".join(chunks))
  if len(packed) != height * (stride + 1):
    raise ValueError("Unexpected PNG scanline length")
  rows = np.empty((height, stride), dtype=np.uint8)
  previous = np.zeros(stride, dtype=np.uint8)
  offset = 0
  for y in range(height):
    filter_type = packed[offset]
    source = np.frombuffer(packed[offset + 1:offset + stride + 1], dtype=np.uint8)
    row = source.copy()
    for x in range(stride):
      left = int(row[x - channels]) if x >= channels else 0
      above = int(previous[x])
      upper_left = int(previous[x - channels]) if x >= channels else 0
      if filter_type == 1:
        row[x] = (int(row[x]) + left) & 0xFF
      elif filter_type == 2:
        row[x] = (int(row[x]) + above) & 0xFF
      elif filter_type == 3:
        row[x] = (int(row[x]) + ((left + above) >> 1)) & 0xFF
      elif filter_type == 4:
        p, pa, pb, pc = left + above - upper_left, 0, 0, 0
        pa, pb, pc = abs(p - left), abs(p - above), abs(p - upper_left)
        predictor = left if pa <= pb and pa <= pc else (above if pb <= pc else upper_left)
        row[x] = (int(row[x]) + predictor) & 0xFF
      elif filter_type != 0:
        raise ValueError("Unsupported PNG filter")
    rows[y] = row
    previous = row
    offset += stride + 1
  return rows.reshape(height, width, channels)[..., :3]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("input", type=Path)
  parser.add_argument("output", type=Path)
  args = parser.parse_args()
  pixels = read_png_rgb(args.input)
  if pixels.shape[0] < 64 or pixels.shape[1] < 64:
    raise ValueError("Asphalt asset is unexpectedly small")
  args.output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(args.output, pixels=np.ascontiguousarray(pixels, dtype=np.uint8))
  print(f"{args.output}: {pixels.shape[1]}x{pixels.shape[0]} RGB")


if __name__ == "__main__":
  main()
