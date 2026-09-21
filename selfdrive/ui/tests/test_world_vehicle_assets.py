from pathlib import Path

import numpy as np


ASSET = Path(__file__).parents[2] / 'assets/world/carnival.npz'


def test_carnival_ego_asset_is_bounded_and_vehicle_sized():
  with np.load(ASSET, allow_pickle=False) as mesh:
    vertices, colors = mesh['vertices'], mesh['colors']
  assert vertices.dtype == np.float32 and colors.dtype == np.uint8
  assert vertices.shape == (len(colors), 3) and colors.shape == (len(vertices), 4)
  # This is the source's complete 450,828-triangle mesh, not a decimated LOD.
  assert len(vertices) // 3 == 450_828 and np.isfinite(vertices).all()
  span = np.ptp(vertices, axis=0)
  assert np.isclose(span[2], 5.155, atol=.001)
  # The exterior mesh includes mirrors, which are wider than the 1.995 m body.
  assert 1.8 <= span[0] <= 2.4 and 1.4 <= span[1] <= 2.1
  assert (colors[:, :3].min(axis=1) > 150).any()
  assert (colors[:, :3].max(axis=1) < 75).any()
  assert ((colors[:, 0] > 100) & (colors[:, 0] > colors[:, 1] * 1.6)).any()
