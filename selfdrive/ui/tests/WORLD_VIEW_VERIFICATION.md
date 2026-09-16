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
use a generic rounded vehicle mesh; unclassified radar returns use small dots.
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

## OLED / Lead Overlay Revision, 2026-09-16

- RGB 0/0/0 background and ground, shaded white vehicles, white lane lines,
  red road edges. The blue predicted-path ribbon is retained. This reduces
  lit background pixels; it does not guarantee prevention of OLED burn-in.
- Reusable swept rounded body/cabin meshes replace the coarse body. Radar
  returns without model vehicle evidence remain neutral dots, not invented
  cars or trucks. Static tail lamps do not claim detected braking.
- Ego avatar heading follows the first six meters of the predicted path,
  with a receive-time-based 0.15 s filter and a 35 degree visual bound. It
  pivots at the path origin, resets when geometry expires, and does not
  rotate early for far-away bends. This is illustrative planned heading,
  NOT measured body yaw, steering-wheel angle or a steering command.
- Restored the shared ModelRenderer lead icon and metrics in world mode.
  Anchors come from the same 3D scene object. Distance is radarState dRel,
  speed is absolute vLead (not vRel), and existing LeadInfo/HideLeadMarker,
  metric/imperial/SI formatting and desired-gap logic are retained. Missing,
  stale, invalid or deduplicated leads do not get an unrelated label.
- 60 targeted tests pass on the comma's native libraries. Coverage includes
  shared metric formatting, freshness, hidden icons, invalid speed, coordinate
  sign, heading filter, fallback, cleanup and alert layering.
- Three archived segments: 14,375 relevant messages and 24 GPU geometry
  snapshots; desktop 600-frame resource/resize regression passed.
- Comma GPU: 2,000 frames, median 12.71 ms, p99 17.44 ms, maximum 22.64 ms,
  including benchmark readback. 12,000 changing-ID updates retained 17,880
  Python bytes. Repeated resource resets passed; no runaway growth observed.
- Actual GPU pixels verified black background, red edges, white cars/lanes,
  and both narrow/landscape parent-target restoration.
- Separate native-font captures verify straight/left/right scenes with real
  production lead text and an exact lead-icon color sample. This caught a
  reversed triangle winding before deployment; that regression is now tested.

The optional full-overlay headless capture needs the venv CFFI backend first
in LD_PRELOAD, before the GBM test adapter, to avoid an incompatible system
libffi callback ABI. This is test-only and never applied to the driving UI:

```sh
PYTHONPATH=/data/openpilot \
LD_PRELOAD=/usr/local/venv/lib/python3.12/site-packages/_cffi_backend.cpython-312-aarch64-linux-gnu.so:/tmp/egl_gbm_headless.so \
RAYLIB_BACKEND=headless /usr/local/venv/bin/python3 \
  selfdrive/ui/tests/verify_world_lead_frame.py --out /tmp/world-lead-check
```

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
