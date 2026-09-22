"""Create compact, non-classified traffic proxies from the approved ego mesh.

The scene has no reliable vehicle make/class message. These meshes therefore
remain *generic traffic proxies*. Voxel clustering merely preserves the smooth
passenger-vehicle silhouette of the already approved display asset at a bounded
triangle budget; it never changes a detection, its pose, or its lifecycle.
"""

import argparse
from pathlib import Path

import numpy as np


def material_groups(colors):
  rgb = colors[:, :3].astype(np.int16)
  brightness = rgb.mean(axis=1) / 255.0
  red = (rgb[:, 0] > rgb[:, 1] * 3 // 2) & (rgb[:, 0] > rgb[:, 2] * 3 // 2)
  glass = ~red & (rgb[:, 2] > rgb[:, 0] + 10) & (rgb[:, 2] >= rgb[:, 1]) & (brightness < .60)
  dark = ~red & ~glass & (brightness < .23)
  return np.select((red, glass, dark), (1, 2, 3), default=0).astype(np.int8)


def cluster_mesh(vertices, colors, cell_size):
  """Quantize a triangle soup, average clustered vertices, and deduplicate faces."""
  lower = vertices.min(axis=0)
  # Never average geometry across a paint/glass/lamp/trim boundary. That was
  # the source of the soft, muddy silhouette in the first decimated preview.
  spatial = np.floor((vertices - lower) / cell_size + .5).astype(np.int32)
  keys = np.column_stack((spatial, material_groups(colors)))
  unique, inverse = np.unique(keys, axis=0, return_inverse=True)
  count = np.bincount(inverse, minlength=len(unique)).astype(np.float32)
  compact_vertices = np.column_stack(tuple(np.bincount(inverse, weights=vertices[:, axis],
                                                       minlength=len(unique)) / count
                                        for axis in range(3))).astype(np.float32)
  compact_colors = np.column_stack(tuple(np.bincount(inverse, weights=colors[:, axis],
                                                     minlength=len(unique)) / count
                                      for axis in range(4))).clip(0, 255).astype(np.uint8)

  faces = inverse.reshape(-1, 3)
  non_degenerate = np.fromiter((len(set(face)) == 3 for face in faces), dtype=bool, count=len(faces))
  faces = faces[non_degenerate]
  # Deduplicate geometry while retaining the source winding of the first face.
  ordered = np.sort(faces, axis=1)
  _, first = np.unique(ordered, axis=0, return_index=True)
  faces = faces[np.sort(first)]
  return compact_vertices[faces].reshape(-1, 3), compact_colors[faces].reshape(-1, 4)


def recolor_generic(colors):
  """Remove model-specific paint while retaining neutral material contrast."""
  rgb = colors[:, :3].astype(np.int16)
  brightness = rgb.mean(axis=1) / 255.0
  groups = material_groups(colors)
  red, glass, dark = groups == 1, groups == 2, groups == 3
  output = colors.copy()
  palettes = np.tile(np.array((222, 230, 240), dtype=np.float32), (len(colors), 1))
  palettes[glass] = (17, 40, 66)
  palettes[dark] = (18, 23, 30)
  palettes[red] = (238, 52, 58)
  # Preserve the source's baked form lighting while stripping paint identity.
  factor = (.66 + brightness[:, None] * .38).clip(.66, 1.04)
  output[:, :3] = np.rint(palettes * factor).clip(0, 255).astype(np.uint8)
  return output


def bake(source, output, cell_size):
  with np.load(source, allow_pickle=False) as asset:
    vertices, colors = asset["vertices"], asset["colors"]
  compact_vertices, compact_colors = cluster_mesh(vertices, recolor_generic(colors), cell_size)
  assert compact_vertices.dtype == np.float32 and compact_colors.dtype == np.uint8
  assert compact_vertices.shape == (len(compact_colors), 3) and len(compact_vertices) % 3 == 0
  assert np.isfinite(compact_vertices).all()
  output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(output, vertices=compact_vertices, colors=compact_colors)
  return {"triangles": len(compact_vertices) // 3, "vertices": len(compact_vertices),
          "bytes": output.stat().st_size, "bounds": [compact_vertices.min(axis=0).round(3).tolist(),
                                                         compact_vertices.max(axis=0).round(3).tolist()]}


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("source", type=Path)
  parser.add_argument("full", type=Path)
  parser.add_argument("lod", type=Path)
  parser.add_argument("--full-cell", type=float, default=.055)
  parser.add_argument("--lod-cell", type=float, default=.115)
  args = parser.parse_args()
  print({"full": bake(args.source, args.full, args.full_cell),
         "lod": bake(args.source, args.lod, args.lod_cell)})


if __name__ == "__main__":
  main()
