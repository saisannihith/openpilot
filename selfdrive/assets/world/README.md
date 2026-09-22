# Tesla Road Vehicle Assets

## Generic traffic

`sedan.npz` is `NormalCar1` from Quaternius's Realistic Car Pack (November
2018), baked as 2,874 vertex-colored triangles. `sedan_lod.npz` is its
1,000-triangle distant LOD. They remain as legacy compatibility assets; the
Tesla Road renderer now uses the neutral proxies described below.

Source: https://quaternius.itch.io/lowpoly-cars

License: CC0 1.0, https://creativecommons.org/publicdomain/zero/1.0/

These remain generic proxies. Model/radar inputs do not provide a vehicle make
or body class, so generic traffic must not be represented as a detected sedan,
truck, or a particular OEM vehicle. Static lamp colors are decoration, not a
braking-state measurement.

## Neutral traffic proxies

`traffic_vehicle.npz` and `traffic_vehicle_lod.npz` are 18,883- and
4,580-triangle neutral silhouettes for ordinary model-associated traffic.
`traffic_vehicle_hq.npz` and `traffic_vehicle_lod_hq.npz` are the 80,580- and
23,052-triangle near/distant versions reserved for the single active
`leadOne` or `leadTwo` object. All four include offline vertex normals for the
display-only automotive material shader.

The proxy is not a claim that a tracked car is a Kia Carnival or that its class
has been identified. It is a stable, high-quality generic silhouette. A
radar-only object remains a dot or radar avatar; no mesh selection changes a
detection, its pose, lifecycle, or control authority.

Rebuild the proxy offline:

```sh
python selfdrive/ui/tests/build_world_lead_vehicle.py \
  selfdrive/assets/world/carnival.npz \
  /tmp/traffic_vehicle_hq_raw.npz \
  /tmp/traffic_vehicle_lod_hq_raw.npz \
  --full-cell .035 --lod-cell .085
python selfdrive/ui/tests/build_world_lit_asset.py \
  /tmp/traffic_vehicle_hq_raw.npz selfdrive/assets/world/traffic_vehicle_hq.npz
python selfdrive/ui/tests/build_world_lit_asset.py \
  /tmp/traffic_vehicle_lod_hq_raw.npz selfdrive/assets/world/traffic_vehicle_lod_hq.npz
```

## 2024 Kia Carnival ego vehicle

`carnival.npz` is the display-only mesh for the known ego vehicle. It is never
used for control or perception.

`carnival_pbr.npz` retains the same complete 450,828-triangle source mesh,
its authored normals, and a compact source-material category used only by the
OLED material shader. It is the current Tesla Road ego asset; it does not add
any runtime GLB parsing, texture loading, or vehicle-control behavior.

Source: [Kia Carnival](https://sketchfab.com/3d-models/kia-carnival-85cc817cc9984fafb879760df5af3ae8)
by [Nieve5677](https://sketchfab.com/niev), licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The committed asset
preserves every source triangle and authored normal; it applies only the
coordinate transform required by the road renderer and OLED material-color
baking. The source GLB is intentionally not shipped; the committed NPZ is the
full-fidelity derivative used by the renderer.

Rebuild with a downloaded, licensed source GLB:

```sh
python selfdrive/ui/tests/build_world_carnival.py \
  /path/to/kia_carnival.glb selfdrive/assets/world/carnival.npz
python selfdrive/ui/tests/build_world_carnival_material.py \
  /path/to/kia_carnival.glb selfdrive/assets/world/carnival_pbr.npz
```

`carnival_aurora_ambient_v2.png` is an original, display-only edge-framed
night landscape. It remains behind measured road, paths, lane lines, traffic,
and safety labels; it never represents camera, map, model, radar, or
navigation data. The asphalt, lane lighting, and red shoulder glow are drawn
only from fresh paired model road-edge geometry.

## Asphalt road material

`asphalt_aurora_dark.npz` is a compact baked RGB color field derived from the
original `asphalt_aurora_dark.png` material. The renderer samples it only
inside fresh paired model-road edges, then applies a bounded, shader-only
aggregate grain. It never changes lane geometry, creates road topology, or
participates in perception or control. The NumPy asset avoids relying on a
GLSL texture sampler, which is unavailable on the comma 3X driver.

Rebuild it with the dependency-free build helper:

```sh
python selfdrive/ui/tests/build_world_asphalt_asset.py \
  /path/to/asphalt_aurora_dark.png selfdrive/assets/world/asphalt_aurora_dark.npz
```
