"""Offline vertex clustering of the licensed sedan; never run onroad."""
import argparse
from pathlib import Path
import numpy as np


def simplify(vertices, colors, grid):
  original = vertices.reshape(-1,3,3)
  snapped = (np.rint(original/grid)*grid).astype(np.float32)
  before = np.cross(original[:,1]-original[:,0],original[:,2]-original[:,0])
  after = np.cross(snapped[:,1]-snapped[:,0],snapped[:,2]-snapped[:,0])
  keep = (np.linalg.norm(after,axis=1) > 1e-6) & (np.sum(before*after,axis=1) > 0)
  return snapped[keep].reshape(-1,3), colors.reshape(-1,3,4)[keep].reshape(-1,4)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('source',type=Path)
  parser.add_argument('output',type=Path)
  parser.add_argument('--grid',type=float,default=.16)
  args = parser.parse_args()
  assert 0 < args.grid <= .2
  with np.load(args.source,allow_pickle=False) as asset:
    v,c = asset['vertices'],asset['colors']
  low,colors = simplify(v,c,args.grid)
  assert 0 < len(low) < len(v)*.75
  assert np.isfinite(low).all()
  np.savez_compressed(args.output,vertices=low,colors=colors)
  print(dict(original_vertices=len(v),lod_vertices=len(low),grid=args.grid))
