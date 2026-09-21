# World Vehicle Asset

Generic sedan: `NormalCar1` from Quaternius's Realistic Car Pack (November 2018).
Author/source: https://quaternius.itch.io/lowpoly-cars
License: CC0 1.0, https://creativecommons.org/publicdomain/zero/1.0/
The author's original license, OBJ and MTL are included alongside the bake.

`sedan.npz` contains 2,874 triangles with baked white paint and vertex lighting.
Wheels, arches, mirrors, pillars and glazing are authored mesh geometry. No
runtime textures, external downloads or OBJ parser are needed. Generic cars
represent model-associated vehicles; the UI does not infer sedan/truck class.
Tail lamps are static decoration, not a claimed braking measurement.

`sedan_lod.npz` is a 1,000-triangle distant derivative of the same CC0 asset.
Offline 0.16 m vertex clustering removes degenerate/reversed triangles and
preserves baked colors. The renderer switches beyond 60 m, returning to the
full mesh below 50 m; the ego and nearby cars always retain the original mesh.

## 2024 Kia Carnival ego vehicle

`carnival.npz` is the display-only mesh for the known ego vehicle. It is never
used for sensor-tracked traffic, because model and radar messages do not report
vehicle make or model.

Source: [Kia Carnival](https://sketchfab.com/3d-models/kia-carnival-85cc817cc9984fafb879760df5af3ae8)
by [Nieve5677](https://sketchfab.com/niev), licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The committed asset
preserves every source triangle and authored normal; it applies only the
coordinate transform required by the road renderer and OLED material-color
baking. The source GLB is intentionally not shipped; the committed NPZ is the
full-fidelity derivative used by the renderer.

`carnival_aurora_ambient.png` is an original, display-only low-luminance
panorama used only when `Ambient Landscape` is enabled in Tesla Road. It is
behind the measured road, paths, lane lines, traffic, and safety labels; it
does not represent camera, map, model, radar, or navigation data.

Rebuild with a downloaded, licensed source GLB:

```sh
python selfdrive/ui/tests/build_world_carnival.py \
  /path/to/kia_carnival.glb selfdrive/assets/world/carnival.npz
```

```sh
python selfdrive/ui/tests/bake_world_lod.py \
  selfdrive/assets/world/sedan.npz selfdrive/assets/world/sedan_lod.npz
```

Rebuild offline with a Raylib-capable Python environment:

```sh
xvfb-run -a python selfdrive/ui/tests/build_world_vehicle.py \
  selfdrive/assets/world/NormalCar1.obj selfdrive/assets/world/sedan.npz
```
