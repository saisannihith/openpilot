"""Add smooth vertex normals to the approved Carnival display mesh.

This is an offline staging utility. It preserves every source vertex and color;
the only added data is a normal used by the preview material shader.
"""
from pathlib import Path
import argparse

import numpy as np


def smooth_normals(vertices: np.ndarray, quantum: float = 1e-5) -> np.ndarray:
  if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) % 3:
    raise ValueError('expected a triangle-list position array')

  faces = vertices.reshape(-1, 3, 3)
  face_normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
  lengths = np.linalg.norm(face_normals, axis=1, keepdims=True)
  face_normals /= np.maximum(lengths, 1e-8)
  vertex_normals = np.repeat(face_normals, 3, axis=0)

  # The source is a triangle soup, so accumulate normals at matching spatial
  # coordinates rather than preserving visibly faceted per-face normals.
  keys = np.rint(vertices / quantum).astype(np.int32)
  _, inverse = np.unique(keys, axis=0, return_inverse=True)
  accumulated = np.zeros((inverse.max() + 1, 3), dtype=np.float64)
  np.add.at(accumulated, inverse, vertex_normals)
  normals = accumulated[inverse]
  normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-8)
  return normals.astype(np.float32)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument('source', type=Path)
  parser.add_argument('destination', type=Path)
  args = parser.parse_args()

  with np.load(args.source, allow_pickle=False) as source:
    vertices = source['vertices'].astype(np.float32, copy=False)
    colors = source['colors'].astype(np.uint8, copy=False)
  normals = smooth_normals(vertices)
  args.destination.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(args.destination, vertices=vertices, colors=colors, normals=normals)
  print(f'vertices={len(vertices)} triangles={len(vertices)//3} normals={len(normals)}')


if __name__ == '__main__':
  main()
