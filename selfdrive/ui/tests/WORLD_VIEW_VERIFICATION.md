# Native World View Verification

## Scope

CameraView=5 (Tesla Road) uses a real perspective camera, reusable shaded 3D
vehicle meshes, depth testing, and model-coordinate road geometry. The dark
palette preserves contrast with the existing light onroad instruments.
It is a display change only. Model inference, cameras, radar fusion, actuation,
TOI/EPS protection, and vehicle parameters are unchanged.

The model path is shown honestly, including disagreement with lane lines. It
is not visually snapped to the lane center. Radar yRel is converted from
left-positive to model right-positive exactly once. Model-associated leads
use a generic vehicle mesh; unclassified radar returns use small wire markers.
No truck/sedan classification, brake-light state, rear coverage, or road
topology is invented. Vehicle dimensions and shadows are illustrative.

## Runtime Bounds

- At most 33 samples per line, 128 incoming radar candidates, and 16 objects.
- One reusable color/depth target capped at 1440 x 810 pixels.
- Fixed 4095-vertex road buffer; uploaded only when model geometry changes.
- Two cached vehicle meshes; no new model, worker, queue, or background process.
- New-message-only object smoothing; stale data expires after 0.35 seconds.
- Offroad/view changes release native meshes and textures. Close is idempotent.
- Rendering failures fall back to the camera for the remainder of that drive.
- Parent framebuffer, transforms, and scissor are restored before HUD/alerts.

## Verified 2026-09-16

- 47 focused scene, layer-order, fallback, cleanup, and camera lifecycle tests
  passed using the comma's native Python libraries. The temporary pytest runner
  emitted three irrelevant missing-plugin configuration warnings.
- Three archived one-minute drive segments: 14,375 relevant messages processed;
  24 real-route geometry snapshots passed the GPU buffer bound checks.
- Desktop Raylib 5.5: 2,000 real graphics frames; landscape and narrow scaled
  parent-framebuffer pixel checks passed.
- Comma 3x, Qualcomm Adreno 630, OpenGL ES 3.2, Raylib 6.0: 6,000 headless GPU
  frames and the same scaled-parent pixel checks passed.
- A final 600-frame regression on both backends additionally verified closing
  the view mid-frame preserves the parent's framebuffer for subsequent HUD
  drawing. This cleanup hardening followed the extended benchmark.
- Device render time, including a benchmark-only pixel readback: median
  12.22 ms, p99 16.60 ms, maximum 23.37 ms. CPU median 11.24 ms, p99 14.58 ms.
- Device scene-update time: median 0.94 ms, p99 1.39 ms, maximum 1.67 ms.
- 12,000 changing-ID updates retained 18,888 Python bytes, peak 400,616 bytes.
  Process high-water RSS stayed below 56 MiB. Post-cleanup RSS settled near
  44 MiB over repeated cycles; screenshot export adds temporary allocation.
- Source lint and Python compilation passed. No control-code diff.

These are isolated-renderer, replay, and offroad proofs, not a live-drive
thermal/performance guarantee or proof of complete object perception. The
real onroad uiDebug timing and device thermals remain useful acceptance data.

## Reproduce

From a configured repository, with a Raylib-capable Python environment:

```sh
python selfdrive/ui/tests/verify_world_view.py --out /tmp/world-ui-check --frames 6000
# Add --route /absolute/path/rlog.zst for each archived segment.
```

For desktop CI without a display, prefix with `xvfb-run -a`.

Qualcomm's EGL default display cannot initialize this headless backend without
a GBM native display. This TEST-ONLY adapter uses the render node and does not
take DRM screen ownership:

```sh
gcc -shared -fPIC -O2 selfdrive/ui/tests/egl_gbm_headless.c -o /tmp/egl_gbm_headless.so -lgbm -ldl
LD_PRELOAD=/tmp/egl_gbm_headless.so RAYLIB_BACKEND=headless \
  /usr/local/venv/bin/python3 selfdrive/ui/tests/verify_world_view.py \
  --out /tmp/world-ui-check --frames 6000
```

Never preload that adapter into the real UI. The benchmark refuses to run
onroad and checks again throughout the run. Pixel readback and all stress
traffic exist only in the test harness, not in production.

The test runner uses Raylib's own graphics entry points. Direct cross-library
ctypes GL queries were incompatible with this device's driver and are not
used. Screen composition must always be tested inside a scaled parent render
texture: ordinary unscaled screenshots alone miss framebuffer/transform bugs.
