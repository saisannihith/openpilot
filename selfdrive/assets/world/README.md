# Tesla Road Vehicle Assets

## Generic traffic

`sedan.npz` is `NormalCar1` from Quaternius's Realistic Car Pack (November
2018), baked as 2,874 vertex-colored triangles. `sedan_lod.npz` is its
1,000-triangle distant LOD. The original OBJ, MTL, and CC0 license are kept
alongside the bake.

Source: https://quaternius.itch.io/lowpoly-cars

License: CC0 1.0, https://creativecommons.org/publicdomain/zero/1.0/

These remain generic proxies. Model/radar inputs do not provide a vehicle make
or body class, so generic traffic must not be represented as a detected sedan,
truck, or a particular OEM vehicle. Static lamp colors are decoration, not a
braking-state measurement.

## Model-associated lead proxy

`lead_vehicle.npz` and `lead_vehicle_lod.npz` are display-only, neutralized
derivatives of the approved Carnival mesh. They preserve separate body, glass,
trim, and lamp material families during offline voxel clustering, yielding
18,883 and 4,580 triangles respectively. They are used only for a nearby
model-associated `leadOne` or `leadTwo` object, with a 30/34 m hysteresis
boundary. Other model-associated traffic continues to use the generic CC0
asset, while unqualified radar tracks continue to render as dots or radar
avatars.

The proxy is not a claim that a tracked car is a Kia Carnival or that its class
has been identified. Its purpose is a stable, high-quality visual silhouette
for the one lead whose distance and speed are already shown by openpilot.

Rebuild the proxy offline:

```sh
python selfdrive/ui/tests/build_world_lead_vehicle.py \
  selfdrive/assets/world/carnival.npz \
  selfdrive/assets/world/lead_vehicle.npz \
  selfdrive/assets/world/lead_vehicle_lod.npz \
  --full-cell .10 --lod-cell .22
```

## 2024 Kia Carnival ego vehicle

`carnival.npz` is the display-only mesh for the known ego vehicle. It is never
used for control or perception.

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
```

`carnival_aurora_ambient_v2.png` is an original, display-only edge-framed
night landscape. It remains behind measured road, paths, lane lines, traffic,
and safety labels; it never represents camera, map, model, radar, or
navigation data. The asphalt, lane lighting, and red shoulder glow are drawn
only from fresh paired model road-edge geometry.
