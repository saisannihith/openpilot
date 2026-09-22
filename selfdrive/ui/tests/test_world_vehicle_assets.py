from pathlib import Path

import numpy as np


WORLD = Path(__file__).parents[2] / 'assets/world'
ASSET = WORLD / 'carnival.npz'
MATERIAL_ASSET = WORLD / 'carnival_pbr.npz'
LEAD_ASSET = WORLD / 'traffic_vehicle_hq.npz'
LEAD_LOD_ASSET = WORLD / 'traffic_vehicle_lod_hq.npz'
ASPHALT_ASSET = WORLD / 'asphalt_aurora_dark.npz'


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


def test_material_ego_asset_keeps_authored_normals_and_categories():
  with np.load(MATERIAL_ASSET, allow_pickle=False) as mesh:
    vertices, colors, normals = mesh['vertices'], mesh['colors'], mesh['normals']
  assert vertices.shape == normals.shape == (len(colors), 3)
  assert vertices.dtype == normals.dtype == np.float32 and colors.dtype == np.uint8
  assert len(vertices) // 3 == 450_828 and np.isfinite(vertices).all() and np.isfinite(normals).all()
  normal_length = np.linalg.norm(normals, axis=1)
  assert np.allclose(normal_length, 1., atol=1e-3)
  # Alpha is a display-only source-material category. It must never reach control code.
  assert set(np.unique(colors[:, 3])).issubset({1, 2, 3, 4, 5})


def test_neutral_lead_assets_are_bounded_and_lit():
  for asset, maximum_vertices in ((LEAD_ASSET, 250_000), (LEAD_LOD_ASSET, 100_000)):
    with np.load(asset, allow_pickle=False) as mesh:
      vertices, colors, normals = mesh['vertices'], mesh['colors'], mesh['normals']
    assert vertices.shape == normals.shape == (len(colors), 3)
    assert vertices.dtype == normals.dtype == np.float32 and colors.dtype == np.uint8
    assert 0 < len(vertices) <= maximum_vertices and len(vertices) % 3 == 0
    assert np.isfinite(vertices).all() and np.isfinite(normals).all()


def test_baked_asphalt_asset_is_compact_rgb_material_only():
  with np.load(ASPHALT_ASSET, allow_pickle=False) as asset:
    pixels = asset['pixels']
  assert pixels.dtype == np.uint8 and pixels.ndim == 3 and pixels.shape[2] == 3
  assert 64 <= pixels.shape[0] <= 1024 and 64 <= pixels.shape[1] <= 1024
  # The texture is dark road material with non-flat authored detail.
  assert np.ptp(pixels.astype(np.int16), axis=(0, 1)).max() > 10
