"""Offline asset bake. Imports CC0 OBJ through Raylib, not handwritten parsing."""
import argparse
from pathlib import Path

import numpy as np
import pyray as rl


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('source',type=Path)
  parser.add_argument('output',type=Path)
  args = parser.parse_args()
  rl.init_window(32,32,'Offline vehicle bake')
  model = rl.load_model(str(args.source))
  vertices, normals, colors = [], [], []
  light_centers = {}
  try:
    for i in range(model.meshCount):
      mesh = model.meshes[i]
      xyz = np.frombuffer(rl.ffi.buffer(mesh.vertices,mesh.vertexCount*12),np.float32).reshape(-1,3).copy()
      norm = np.frombuffer(rl.ffi.buffer(mesh.normals,mesh.vertexCount*12),np.float32).reshape(-1,3).copy()
      if mesh.indices != rl.ffi.NULL:
        indices = np.frombuffer(rl.ffi.buffer(mesh.indices,mesh.triangleCount*6),np.uint16)
        xyz,norm = xyz[indices],norm[indices]
      c = model.materials[model.meshMaterial[i]].maps[rl.MaterialMapIndex.MATERIAL_MAP_ALBEDO].color
      if c.b > c.r*1.2:
        color = (245,247,250)
      elif c.r > c.g*2 and c.g > c.b*1.5:
        color = (230,238,248)
        light_centers['front'] = xyz.mean(axis=0)
      elif c.r > c.g*2:
        color = (160,30,36)
        light_centers['rear'] = xyz.mean(axis=0)
      elif max(c.r,c.g,c.b) < 5:
        color = (19,21,23)
      elif max(c.r,c.g,c.b) < 20:
        color = (36,47,59)
      else:
        color = (145,151,160)
      print(i,mesh.vertexCount,(c.r,c.g,c.b),color,flush=True)
      vertices.append(xyz)
      normals.append(norm)
      colors.append(np.tile(color,(len(xyz),1)))
    xyz,norm,rgb = np.concatenate(vertices),np.concatenate(normals),np.concatenate(colors)
    assert len(xyz) <= 20000 and len(xyz)%3 == 0
    assert set(light_centers) == {'front','rear'}
    if light_centers['front'][2] > light_centers['rear'][2]:
      xyz[:,[0,2]] *= -1
      norm[:,[0,2]] *= -1
    lower,upper = xyz.min(axis=0),xyz.max(axis=0)
    xyz -= [(lower[0]+upper[0])/2,lower[1],(lower[2]+upper[2])/2]
    xyz *= 4.8/(upper[2]-lower[2])
    lighting = .62 + .27*np.clip(norm[:,1],0,1) + .09*np.clip(norm[:,2],0,1) + .02*np.clip(-norm[:,0],0,1)
    rgba = np.column_stack((np.clip(rgb*lighting[:,None],0,255),np.full(len(xyz),255))).astype(np.uint8)
    assert np.isfinite(xyz).all() and np.isfinite(norm).all()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(args.output,vertices=xyz.astype(np.float32),colors=rgba)
    print('BAKED',len(xyz)//3,'triangles; bounds',xyz.min(axis=0),xyz.max(axis=0))
  finally:
    rl.unload_model(model)
    rl.close_window()


if __name__ == '__main__':
  main()
