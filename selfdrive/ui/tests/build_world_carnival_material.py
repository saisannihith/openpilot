"""Bake the CC-BY Kia Carnival GLB into a full-fidelity display-only ego mesh.

This is an offline developer tool. The runtime receives only a compact NPZ;
there is no GLB parser, texture loading, or model processing on the comma.
"""
import argparse
import json
import struct
from pathlib import Path

import numpy as np


CARNIVAL_LENGTH_M = 5.155
SOURCE_URL = 'https://sketchfab.com/3d-models/kia-carnival-85cc817cc9984fafb879760df5af3ae8'
DTYPES = {5121: np.dtype('<u1'), 5123: np.dtype('<u2'), 5125: np.dtype('<u4'), 5126: np.dtype('<f4')}
WIDTHS = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}
STYLE_PAINT = 1
STYLE_LAMP = 2
STYLE_GLASS = 3
STYLE_METAL = 4
STYLE_TRIM = 5


def read_glb(path):
  raw = path.read_bytes()
  magic, version, length = struct.unpack_from('<4sII', raw)
  assert magic == b'glTF' and version == 2 and length == len(raw)
  json_length, json_type = struct.unpack_from('<II', raw, 12)
  assert json_type == 0x4E4F534A
  document = json.loads(raw[20:20 + json_length])
  binary_offset = 20 + json_length
  binary_length, binary_type = struct.unpack_from('<II', raw, binary_offset)
  assert binary_type == 0x004E4942 and binary_offset + 8 + binary_length == len(raw)
  return document, memoryview(raw)[binary_offset + 8:binary_offset + 8 + binary_length]


def load_accessor(document, binary, index):
  accessor = document['accessors'][index]
  assert 'bufferView' in accessor and not accessor.get('sparse')
  view = document['bufferViews'][accessor['bufferView']]
  dtype = DTYPES[accessor['componentType']]
  width = WIDTHS[accessor['type']]
  item_bytes = dtype.itemsize * width
  stride = view.get('byteStride', item_bytes)
  assert stride >= item_bytes
  offset = view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
  return np.ndarray((accessor['count'], width), dtype=dtype, buffer=binary,
                    offset=offset, strides=(stride, dtype.itemsize)).copy()


def local_matrix(node):
  if 'matrix' in node:
    return np.asarray(node['matrix'], dtype=np.float64).reshape(4, 4).T
  tx, ty, tz = node.get('translation', (0., 0., 0.))
  x, y, z, w = node.get('rotation', (0., 0., 0., 1.))
  scale = np.asarray(node.get('scale', (1., 1., 1.)), dtype=np.float64)
  rotation = np.array(((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
                       (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
                       (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y))), dtype=np.float64)
  result = np.eye(4, dtype=np.float64)
  result[:3, :3] = rotation @ np.diag(scale)
  result[:3, 3] = (tx, ty, tz)
  return result


def nodes(document):
  scene = document['scenes'][document.get('scene', 0)]
  stack = [(index, np.eye(4, dtype=np.float64)) for index in reversed(scene['nodes'])]
  while stack:
    index, parent = stack.pop()
    node = document['nodes'][index]
    world = parent @ local_matrix(node)
    yield node, world
    stack.extend((child, world) for child in reversed(node.get('children', ())))


def primitive_color(document, primitive):
  if 'material' not in primitive:
    return np.array((245, 247, 250), dtype=np.uint8), STYLE_PAINT
  material = document['materials'][primitive['material']]
  name = material.get('name', '')
  # Preserve the exterior geometry while presenting it in the intentionally
  # high-contrast OLED world palette. These labels are from this CC-BY source;
  # generic traffic never uses this source-specific styling.
  if name == '2022_Kia_Carnival_Panthera_Metal':
    return np.array((245, 247, 250), dtype=np.uint8), STYLE_PAINT
  if name in ('2022_Kia_Carnival_BrakeLightsMain', '2022_Kia_Carnival_Reflector'):
    return np.array((205, 28, 34), dtype=np.uint8), STYLE_LAMP
  if any(token in name for token in ('Windows', 'HeadLightsGlass', 'BrakeLightsGlass', 'ReflectorGlass')):
    return np.array((20, 34, 48), dtype=np.uint8), STYLE_GLASS
  if any(token in name for token in ('HeadLights', 'DRLs', 'Chrome', 'Mirrors', 'Rims', 'BrakeRotors', 'Badges')):
    return np.array((190, 199, 209), dtype=np.uint8), STYLE_METAL
  if any(token in name for token in ('Interior', 'Tires', 'UnderCarrier', 'Grille', 'Black', 'Calipers')):
    return np.array((24, 29, 35), dtype=np.uint8), STYLE_TRIM
  factor = material.get('pbrMetallicRoughness', {}).get('baseColorFactor', (1., 1., 1., 1.))
  return np.rint(np.clip(factor[:3], 0., 1.) * 255).astype(np.uint8), STYLE_METAL


def load_triangles(document, binary):
  vertices, normals, colors, styles = [], [], [], []
  for node, world in nodes(document):
    if 'mesh' not in node:
      continue
    for primitive in document['meshes'][node['mesh']]['primitives']:
      assert primitive.get('mode', 4) == 4 and {'POSITION', 'NORMAL'} <= primitive['attributes'].keys()
      positions = load_accessor(document, binary, primitive['attributes']['POSITION']).astype(np.float64)
      source_normals = load_accessor(document, binary, primitive['attributes']['NORMAL']).astype(np.float64)
      indices = load_accessor(document, binary, primitive['indices']).reshape(-1) if 'indices' in primitive else np.arange(len(positions))
      assert len(indices) % 3 == 0 and (not len(indices) or indices.max() < len(positions))
      transformed = np.column_stack((positions[indices], np.ones(len(indices)))) @ world.T
      normal_matrix = np.linalg.inv(world[:3, :3]).T
      transformed_normals = source_normals[indices] @ normal_matrix.T
      transformed_normals /= np.maximum(1e-12, np.linalg.norm(transformed_normals, axis=1))[:, None]
      vertices.append(transformed[:, :3])
      normals.append(transformed_normals)
      color, style = primitive_color(document, primitive)
      colors.append(np.tile(color, (len(indices), 1)))
      styles.append(np.full(len(indices), style, dtype=np.uint8))
  assert vertices
  return np.concatenate(vertices), np.concatenate(normals), np.concatenate(colors), np.concatenate(styles)


def orient_and_scale(vertices, normals, colors, styles):
  spans = np.ptp(vertices, axis=0)
  longitudinal = 0 if spans[0] >= spans[2] else 2
  lateral = 2 if longitudinal == 0 else 0
  center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
  red = (colors[:, 0] > 95) & (colors[:, 0] > colors[:, 1] * 1.45) & (colors[:, 0] > colors[:, 2] * 1.45)
  assert red.any(), 'Could not identify a rear-lamp material'
  rear_sign = 1. if vertices[red, longitudinal].mean() >= center[longitudinal] else -1.
  oriented = np.column_stack((vertices[:, lateral] - center[lateral],
                               vertices[:, 1] - vertices[:, 1].min(),
                               rear_sign * (vertices[:, longitudinal] - center[longitudinal])))
  oriented_normals = np.column_stack((normals[:, lateral], normals[:, 1], rear_sign * normals[:, longitudinal]))
  oriented_normals /= np.maximum(1e-12, np.linalg.norm(oriented_normals, axis=1))[:, None]
  oriented *= CARNIVAL_LENGTH_M / np.ptp(oriented[:, 2])
  return oriented.astype(np.float32), oriented_normals.astype(np.float32), colors, styles, longitudinal, rear_sign


def material_colors(colors, styles):
  # Alpha transports the source material category to the display-only shader.
  # That shader outputs opaque geometry and separately applies actor fading.
  return np.column_stack((colors, styles)).astype(np.uint8)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('source', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  document, binary = read_glb(args.source)
  vertices, normals, colors, styles = load_triangles(document, binary)
  vertices, normals, colors, styles, axis, rear_sign = orient_and_scale(vertices, normals, colors, styles)
  rgba = material_colors(colors, styles)
  assert len(vertices) % 3 == 0 and 0 < len(vertices) <= 1_500_000 and np.isfinite(vertices).all()
  args.output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(args.output, vertices=vertices, colors=rgba, normals=normals)
  print({'source': str(args.source), 'source_url': SOURCE_URL, 'triangles': len(vertices) // 3,
         'full_fidelity': True, 'longitudinal_axis': int(axis), 'rear_sign': rear_sign,
         'material_styles': {int(style): int((styles == style).sum()) for style in np.unique(styles)},
         'bounds_m': (vertices.min(axis=0).round(3).tolist(), vertices.max(axis=0).round(3).tolist())})


if __name__ == '__main__':
  main()
