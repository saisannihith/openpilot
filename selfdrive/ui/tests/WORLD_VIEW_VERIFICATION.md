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
use a generic authored sedan mesh; unclassified radar returns use small dots.
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

## Authored Cars and Shared Overlays, 2026-09-16

This revision supersedes the procedural vehicle body and prior overlay behavior
above. Earlier benchmark numbers describe their respective earlier revisions.

- Added Quaternius's CC0 NormalCar1 sedan, with wheels, arches, mirrors, glazing,
  and pillars. Original OBJ/MTL/license and a reproducible offline bake are included.
  Runtime reads one cached, validated 23 KiB NPZ; 2,874 triangles per car.
- Only radarState.leadOne can receive a lead chevron or distance/speed/gap labels
  in world view. There is no secondary-lead fallback or display-side lead selection.
  Other associated vehicles can remain visible without lead annotations.
- Shared StarPilot stopping-point octagon/metrics now use fresh planner distance
  projected into the world. It is a planned-stop indicator, not physical sign
  classification. ShowStoppingPoint and ShowStoppingPointMetrics retain authority.
- The existing path width, edge, color, rainbow, acceleration, dynamic-width,
  adjacent-lane and blindspot display policies now draw through world projection.
  White lane lines and red road edges intentionally retain the requested palette.
  The current branch's Tesla Road selector is CameraView=5; no separate Tesla-path
  toggle was found. No new driving policy or parameter writes were introduced.
- Speed, cruise, steering/experimental button, driver monitor and alerts remain
  in the existing HUD layers. Full-alert suppression and camera fallback remain.
- STOP bounds are reserved before primary lead metrics are laid out. Collision
  handling tries a bounded left/right placement and otherwise omits crowded text.
- Batched projection matches Raylib's native projection within 0.001 pixels at
  tested points. It avoids hundreds of Python/CFFI projection crossings per frame.
- Actual Cap'n Proto model messages exposed unsupported list slicing during audit;
  bounded islice/fromiter conversion fixes that issue, with a real-schema regression.

Verification against this revision on the offroad comma 3x:

- 72 focused tests passed; three warnings concern unavailable optional pytest plugins.
- Three archived segments: 57,333 relevant messages, 3,599 model frames, 359 sampled
  complete GPU render frames using actual messages and all three path color modes.
- Full native-font scene captures: primary-only lead marker, STOP/lead non-overlap,
  straight/left/right turns, exact lead-icon pixels, and projection equivalence.
- 600-frame complete-scene timing including benchmark-only readback: median 22.18 ms,
  p99 31.99 ms, maximum 120.92 ms (includes cold initialization). This is not an
  onroad frame-time guarantee. Full HUD/live camera-process load was not benchmarked.
- 1,000-frame mesh/resource run: median 12.70 ms, p99 17.99 ms, max 20.75 ms after
  warmup. 12,000 changing-ID scene updates retained 17,880 Python bytes. RSS after
  two cleanup cycles was 60,204/60,488 KiB; this is bounded-test evidence, not proof
  against every possible leak.
- Both standalone and shared-road-callback modes passed landscape/narrow scaled
  parent framebuffer tests, including drawing HUD pixels after view cleanup.
- New modules pass lint; changed modules introduce no new lint findings versus
  HEAD (eight pre-existing upstream findings remain). Compile and diff-scope
  checks pass and exclude control-code changes.

The device's view selection is preserved. Select Tesla Road to use this scene.
The screenshot fixtures intentionally exercise both a STOP marker and primary
lead metrics simultaneously; they are not a reconstructed complete road scene.

## Vehicle Orientation and Identity Revision, 2026-09-16

- Fixed surrounding meshes always rendering at zero yaw. A bounded local tangent
  from the two confident lane boundaries surrounding each detection now supplies
  illustrative body orientation. Boundary slopes interpolate across a crossing;
  detected position is never snapped or biased toward a lane center.
- The mesh center offset rotates with the body about its detected rear reference.
  Lead icons project the same rotated center, rather than a separate straight-ahead
  offset. Mesh length remains illustrative (4.8 m), not measured vehicle dimensions.
- This is NOT measured target yaw, lane intent or direction classification. Missing
  lane support uses neutral orientation; the display cannot infer arbitrary oncoming
  headings or accurately reconstruct every object from these messages.
- Radar track identity prevents duplicate representations and prevents smoothing
  across different targets reusing leadOne/leadTwo. Identity continuity is retained
  when a track changes lead slots. Vision-only targets use bounded geometric
  continuity because no persistent vision object ID is exposed here.
- Smoothing runs only with advancing source timestamps. Other-topic updates no
  longer jump a smoothed position back to the raw coordinate. A new drive resets
  continuity. Missing/status-false/stale/error targets are not artificially retained.
- Corrected the synthetic curve fixture: adjacent vehicle coordinates now follow
  its curved lane geometry. This fixture correction does not alter real detections.
- 95 native tests pass, including left/right curves across five lane/crossing
  positions, rear-reference preservation, identity swaps/deduplication, disappearance,
  model-only refresh, asynchronous raw radar refresh, and new-drive resets.
- Three actual segments replayed: 57,333 messages / 359 sampled GPU frames. Segment
  44 contained 23 oriented vehicle-frames (>1 degree); the other two contained none.
  All three passed duplicate-identity and primary-only annotation checks.
- Full-scene 600-frame device benchmark: median 22.14 ms, p99 27.04 ms, maximum
  99.24 ms including cold initialization. Scaled parent/HUD composition still passes.
- Final resource run: 1,000 GPU frames and 12,000 changing-ID updates; 20,104
  retained Python bytes, 403,368 peak bytes, scene-update median/p99 1.57/2.45 ms.
  Renderer-only warm median/p99 13.14/17.35 ms. Cleanup RSS 60,832/61,076 KiB.
  These bounded offroad tests do not establish unlimited leak-free/live-drive behavior.
- No changes to fusion, planners, controllers, safety limits, or device settings.

## Compact Lead Label Revision, 2026-09-16

- World-only lead text is now 24 px rather than 36 px, arranged in two lines:
  distance and absolute speed; time gap and desired distance (when applicable).
- The label follows the primary car's projected anchor and remains horizontally
  centered on it. STOP collisions stack the label upward, never off to the side.
  Labels without enough viewport space are omitted rather than clipped/detached.
- Camera-mode label size/format and planner/control behavior are unchanged.
- The synthetic renderer preview now defaults to no STOP state. Pass --with-stop
  explicitly for its stopping-point/collision test. Real display still requires
  ShowStoppingPoint plus a valid redLight planner state and fresh world geometry.
- 96 native unit tests pass. Native-font straight/left/right captures and scaled
  framebuffer checks pass both with and without the synthetic STOP state.

## Live Tesla Road Toggle, 2026-09-16

- Tesla Road is now an explicit toggle under Appearance > Model & Path
  Visualization, available even when the Model UI parent is off. It writes only
  the existing CameraView integer (5); TeslaRoad is a UI row ID, not a new native
  parameter. No params library rebuild or data migration is required.
- The old Camera View picker contains only camera/blank modes. Turning Tesla Road
  off restores the prior selection in this UI session; after a UI restart it
  returns to Standard if no earlier selection is known. Choosing a camera also
  turns the Tesla Road toggle off because they share the same stored value.
- Settings writes invalidate the shared parameter cache immediately. Renderer
  transitions release/recreate graphics and reset camera clients without restarting
  the manager, model, controls, device or car.
- A render failure still falls back to the camera without repeatedly retrying
  each frame. Choosing another view or returning from settings clears the failure
  latch, permitting a user-requested retry during the same drive.
- 108 native tests passed: settings placement and real callbacks, cached readback,
  20 toggle cycles for each of five camera modes, 30 renderer-switch cycles,
  persisted-on disable, and injected-failure recovery, plus existing UI regressions.
- Native GPU checks passed 20 hot resource recreation cycles and scaled HUD
  framebuffer restoration. Actual path pixels changed with RainbowPath on and
  matched the starting colors exactly when switched off, in the same process.
  Rainbow uses the existing shared gradient and takes priority over acceleration
  colors. The pixel test clears its framebuffer exactly as GuiApplication does.
- Physical live-camera stream reacquisition while driving was not tested offroad.

## Full-Screen Framing And Radar Avatars, 2026-09-16

- The approved 2160x1080 composition uses a closer camera and viewport clipping
  rather than dropping offscreen ribbon endpoints. Existing near-field lane
  tangents continue behind the ego avatar for presentation only. No detected
  path, lane, road-edge, or vehicle position is moved or written back to controls.
- Display-only radar history uses at most 128 input IDs and the existing 16-object
  scene cap. Measured, continuous targets moving within confident lane geometry
  can receive a muted generic car avatar after at least 0.3 seconds / four samples.
  This is a visualization heuristic, not an OEM vehicle classification or a new
  control-qualified lead. Uncertain/static returns stay dots; a previously moving
  target may remain an avatar when it stops while observations remain continuous.
- Identity jumps, missing IDs, error flags, stale input and new drives reset the
  visual history. Raw radar avatars are centered at the radar reflection location;
  their body dimensions are illustrative. No lane snapping or unseen rear traffic.
  Estimated traffic direction uses consistent ego-plus-relative longitudinal
  velocity and road tangent, not measured vehicle yaw or divided-road classification.
- 130 native tests passed, including lifecycle, identity reuse, duplicates,
  stationary clutter, opposing motion, 128-ID bounds, projection and existing UI.
  Three unrelated pytest configuration warnings reflect absent optional plugins.
- Six unique recorded segments passed UI replay: 114,674 relevant messages and
  719 sampled GPU frames. Recorded lane counts span 0-4; radar avatars appeared
  in multiple segments, with up to three simultaneous radar-only avatars.
  Replay advances scene state for every relevant message before sampling renders.
- Two exact camera/model frame matches from directly identified device recordings
  were inspected: a vehicle in the left lane and two vehicles ahead on an
  undivided road. These are spot checks, not comprehensive object classification
  ground truth. An older copied segment did not match the initially selected
  current-route video by hash; that mismatched video was excluded from validation.
- No sampled real segment contained a qualified oncoming avatar. That branch has
  synthetic coverage only. No 360-degree tracking or unseen vehicles are implied.
- Native GPU checks include 2160x1080 bottom-edge pixels, straight/left/right
  geometry, label/icon alignment, rainbow on/off and 20 hot resource recreations.
  No changes to control/radar qualification, steering safety, parameters or schemas.
- The initial 2,200-update / 128-input stress run retained 1,096 Python bytes;
  its repeated tangent work prompted an equivalent lane-membership optimization.
  The optimized 800-update test retained 2,888 bytes after collection, with
  18.63 ms median / 21.88 ms p99 whole-scene update time at the 128-input cap.
  Both runs stayed at <=128 history entries and <=16 displayed objects. These
  finite offroad tests do not establish unlimited leak-free or onroad performance.

## Five Display Improvements, 2026-09-16

- Added a bounded presentation layer for up to 16 visible vehicle identities.
  Cars and their lead labels share the same interpolated pose. Radar/model
  handoffs can carry asynchronously timestamped observations without changing
  identity or spawning a duplicate. The underlying measured scene is untouched.
- Observed motion is interpolated over 50 ms, including lateral cut-ins and
  forward-range passing. There is no extrapolation, rear-traffic invention, lane
  snapping, or resurrection of missing targets. Large jumps, stale data, new
  drives and recycled IDs reset presentation history. Low-rate source data and
  uncertain vehicle heading still limit visual fidelity.
- Adjacent surfaces stay OLED black; model geometry does not prove traffic
  direction, lane availability or a highway divider. Removed the world-only
  red/green adjacent-lane fills; stock camera rendering is unchanged. Existing
  HUD alerts and blind-spot indicators retain their own rendering paths.
- Compact primary-lead metrics follow the rendered vehicle. Invalid, zero or
  stale desired distances are hidden, as are time gaps when ego speed is stale
  or below 1 m/s. Infinite desired distance no longer crashes label formatting.
- Tesla Road falls back to Standard camera when model data is stale or its path
  unusable. It requires 0.75 s of continuously fresh geometry before returning.
  Unmarked roads do not trigger fallback merely because lane lines are absent.
  Render exceptions retain the existing camera fallback latch and explicit retry.
  No control state, Params value or driver alert is changed by these transitions.
- 157 native tests passed, including actual overlay/fallback integration,
  asynchronous handoffs, stale/recycled identities and real Cap'n Proto inputs.
  Three existing optional pytest-plugin configuration warnings remain.
- Three identified recorded segments (route 27 segments 5/12; route 28 segment
  20) passed: 57,341 relevant messages, 3,600 model frames and 360 GPU samples.
  Presentation is advanced at model cadence, not just screenshot cadence.
  Pixel/lifecycle checks covered native 2160x1080, narrower/scaled parent targets,
  rainbow toggling, STOP overlays and 20 resource recreations. Captures were
  visually inspected; no real oncoming-avatar example exists in these segments.
- The 800-update scene-plus-presentation stress test at 128 inputs retained
  2,664 bytes after GC, with 19.23 ms median and 23.44 ms p99 update time.
  This is finite offroad evidence, not an unlimited leak or onroad FPS guarantee.
- Road-camera reconnection while moving is not verified by these offroad tests.
  Camera fallback depends on a functioning camera stream. This display cannot
  compensate for a missing perception model or make driving decisions.

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
