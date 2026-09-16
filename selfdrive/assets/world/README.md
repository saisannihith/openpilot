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

Rebuild offline with a Raylib-capable Python environment:

```sh
xvfb-run -a python selfdrive/ui/tests/build_world_vehicle.py \
  selfdrive/assets/world/NormalCar1.obj selfdrive/assets/world/sedan.npz
```
