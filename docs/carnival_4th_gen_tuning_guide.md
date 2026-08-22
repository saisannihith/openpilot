# 2024 Kia Carnival 4th Gen Tuning Guide

Status: SNITHPilot branch guidance as of 2026-08-22.

Vehicle target: 2024 Kia Carnival 4th Gen, HKG CAN-FD, torque-steering platform.

## Branch Verdict

Use SNITHPilot/StarPilot-derived code as the base for this vehicle, then selectively pull from comma openpilot and sunnypilot.

Do not use pure comma openpilot master as the daily branch for this Carnival goal. comma openpilot has the strongest core model pipeline, especially 0.11+ learned-simulator E2E longitudinal and 0.11.2 big-model support, but it does not include the Carnival-specific radar confirmation, torque guard, MADS-style workflow, and StarPilot/SNITH tuning needed for this vehicle.

Do not use pure sunnypilot dev as the daily branch without the Carnival-specific patches. sunnypilot dev is the stronger reference for HKG UX, MADS, custom longitudinal tuning, radar-track feature direction, and settings ergonomics. It is not, by itself, proven against this Carnival's logged behavior.

Recommended architecture:

1. comma openpilot model/runtime improvements for E2E policy and big-model support.
2. sunnypilot HKG/MADS/settings ideas where they are compatible.
3. SNITHPilot Carnival-specific lateral torque, radar confirmation, lead-depart, and stop-line tuning.

## Safety Position

This branch should remain radar-confirmed and model-led. The radar stream must not become independent radar-only braking.

Current evidence supports:

- Radar distance/lateral confirmation.
- Visual radar points.
- Stop-and-go object presence confirmation.
- Phantom-brake protection by not blindly promoting every raw radar point.
- Cut-in-like lateral movement diagnostics.

Current evidence does not support:

- Promoting raw radar velocity into primary longitudinal control.
- Removing all "alpha" caution from openpilot longitudinal.
- Treating yellow-light handling as guaranteed semantic detection.

## UI And Parameter Setup

Recommended driver-facing setup:

- Enable openpilot/sunnypilot longitudinal only for planned tests.
- Enable Experimental Mode for intersection testing, not as an always-on guarantee.
- Enable Conditional Experimental / Conditional Chill behavior when available.
- Prefer Predictive custom longitudinal tuning where available. Avoid Dynamic until logs show braking and launch are too soft.
- Enable MADS via the HKG LDA-button workflow.
- Enable Lead Vehicle Metrics and Radar Point Display only for validation.

Recommended follow behavior:

- Use Standard or Relaxed personality for family/passenger comfort.
- Avoid Aggressive for stop-and-go refinement.
- Increase following distance before tuning code if occupants feel close to the lead car.

## Current Carnival Constants To Preserve

These values are already tuned toward smoother Carnival behavior and should not be changed without route-level evidence.

Longitudinal lead departure:

- `CARNIVAL_LEAD_DEPART_MIN_ACCEL = 1.00`
- `CARNIVAL_LEAD_DEPART_CREEP_RELEASE_MIN_ACCEL = 0.30`
- `CARNIVAL_LEAD_DEPART_ACCEL_HOLD_TIME = 2.5`
- `CARNIVAL_LEAD_DEPART_ACCEL_HOLD_MAX_EGO_SPEED = 5.5`

Radar-confirmed stop hold:

- `CARNIVAL_RADAR_STOP_HOLD_ENTER_MAX_EGO_SPEED = 3.2`
- `CARNIVAL_RADAR_STOP_HOLD_HOLD_MAX_EGO_SPEED = 3.8`
- `CARNIVAL_RADAR_STOP_HOLD_ENTER_MAX_DISTANCE = 9.5`
- `CARNIVAL_RADAR_STOP_HOLD_HOLD_MAX_DISTANCE = 10.5`
- `CARNIVAL_RADAR_STOP_HOLD_MAX_LEAD_SPEED = 1.0`
- `CARNIVAL_RADAR_STOP_HOLD_MAX_LATERAL_OFFSET = 1.75`
- `CARNIVAL_RADAR_STOP_HOLD_MIN_BRAKE = 0.55`
- `CARNIVAL_RADAR_STOP_HOLD_MAX_BRAKE = 0.95`

Intersection red-light stop-line guard:

- `CARNIVAL_RED_LIGHT_STOP_LINE_MIN_DECEL = 0.95`
- `CARNIVAL_RED_LIGHT_STOP_LINE_MAX_DECEL = 3.00`
- `CARNIVAL_RED_LIGHT_STOP_LINE_BRAKE_MARGIN = 0.10`
- `CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_SPEED = 12.0`
- `CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_LENGTH = 20.0`
- `CARNIVAL_PRE_RED_STOP_EVIDENCE_MAX_LENGTH = 200.0`
- `CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_BRAKE = 0.75`
- `CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_DROP = 15.0`
- `CARNIVAL_PRE_RED_STOP_EVIDENCE_MIN_RATIO = 0.35`
- `CARNIVAL_PRE_RED_STOP_EVIDENCE_HOLD_TIME = 3.0`

Lateral/EPS protection:

- `CARNIVAL_4TH_GEN_HIGH_ANGLE_TAPER_START = 85.0`
- `CARNIVAL_4TH_GEN_HIGH_ANGLE_TAPER_FULL = 220.0`
- `CARNIVAL_4TH_GEN_HIGH_ANGLE_TORQUE_MIN_SCALE = 0.35`
- `CARNIVAL_4TH_GEN_EPS_GUARD_MIN_SPEED = 17.0`
- `CARNIVAL_4TH_GEN_EPS_GUARD_TORQUE_FRACTION = 0.92`
- `CARNIVAL_4TH_GEN_EPS_GUARD_TRIGGER_FRAMES = 70`
- `CARNIVAL_4TH_GEN_EPS_GUARD_HOLD_FRAMES = 120`
- `CARNIVAL_4TH_GEN_EPS_GUARD_CAP_LOW = 0.94`
- `CARNIVAL_4TH_GEN_EPS_GUARD_CAP_HIGH = 0.88`

Torque override:

- `KIA_CARNIVAL_4TH_GEN = [1.63, 1.75, 0.134]`

## Intersection Handling

The most reliable path is model-led E2E, with conservative stop-line guards.

Use Experimental Mode to let the E2E model slow for stop signs, red lights, stopped traffic, and turns. Do not tune yellow lights as a hard trigger. Yellow-light behavior should be treated as advisory: the model may slow if the scene implies a stop, but the driver must be ready to brake.

The red-light stop-line guard should only activate when:

- The vehicle is a Carnival 4th Gen.
- Red-light state is present.
- There is no active lead-control context.
- The model stop-line distance is finite and plausible.
- Required decel is within the configured safe range.

Avoid fixes that brake from a color bit alone. That creates false-positive highway braking risk.

## Lead Detection And Stop-And-Go

The intended fusion model is:

1. Vision/model decides the lead/scene context.
2. Carnival radar confirms distance and lateral object presence.
3. Raw radar velocity remains diagnostic unless proven stable.
4. Close stopped-lead hold uses radar confirmation only at low speed and close range.

Current readiness gate names:

- `publishReady`: radar confirmation stream is present and usable for publishing.
- `tandemReady`: radar-confirmed, model-led use is ready.
- `visualRadarTrackReady`: live radar points can support UI display.
- `phantomBrakeGuardReady`: raw radar points exist without being blindly promoted.
- `cutInTrackingEvidence`: logs contain lateral movement evidence useful for cut-in diagnostics.
- `velocityControlReady`: raw radar velocity is safe enough for primary velocity control. This should remain false unless logs prove otherwise.

Expected healthy state:

```text
publishReady=True
tandemReady=True
visualRadarTrackReady=True
phantomBrakeGuardReady=True
cutInTrackingEvidence=True
velocityControlReady=False
readinessConclusion=radar_confirmed_model_led_ready
```

## Radar Verification Over SSH

Run from the comma:

```bash
cd /data/openpilot
git rev-parse --short HEAD
PYTHONPATH=/data/openpilot:/data/openpilot/analysis \
  /usr/local/venv/bin/python analysis/run_carnival_radar_longitudinal_readiness.py \
  --radar-only /data/media/0/realdata/<route-segment>/rlog.zst
```

For several route segments:

```bash
PYTHONPATH=/data/openpilot:/data/openpilot/analysis \
  /usr/local/venv/bin/python analysis/run_carnival_radar_longitudinal_readiness.py \
  --radar-only \
  /data/media/0/realdata/<route>--26/rlog.zst \
  /data/media/0/realdata/<route>--27/rlog.zst \
  /data/media/0/realdata/<route>--28/rlog.zst \
  --out /data/openpilot/analysis/carnival-radar-readiness.json
```

UI checks:

- Enable Radar Point Display.
- Verify radar points align with real lead vehicles.
- Verify lead metrics remain stable when a lead is visually obvious.
- Verify radar points alone do not create sudden braking without a plausible vision/model lead.

## Lateral Precision

Use MADS for steering-only or lateral-first validation. The 2024 Carnival 4th Gen is torque steering, not the 2025 angle-steering Carnival path.

Targets:

- Stable lane center on marked highway lanes.
- No steering assist temporarily unavailable warnings during ordinary curves.
- No sudden lateral release during sustained curve torque.
- Avoid over-tightening torque until EPS warning behavior is fully stable.

Do not remove the EPS guard to chase a stronger feel. If the car cannot hold a curve without EPS warnings, tune the lateral controller and torque ramping around the guard, not through it.

## Real-World Test Protocol

Phase 1: Offroad/static

- Confirm branch and commit.
- Confirm no startup process errors.
- Confirm `RadarTracksUI` can be toggled without UI crash.
- Confirm no dirty runtime source files except generated theme/log artifacts.

Phase 2: Low-speed residential, no passengers

- MADS lateral only.
- 10-25 mph curves.
- Hands ready.
- Abort on any steering assist unavailable warning.

Phase 3: Stop-and-go lead

- Longitudinal on.
- Experimental on only if intentionally testing E2E.
- Follow a lead from 0-30 mph.
- Check for launch delay, jerk, and stop distance.

Pass indicators:

- No hard launch spike.
- No late brake into a stopped lead.
- `startCmdAccel` and `startPlanAccel` are nonzero during lead departure.
- `egoMoveLongControlState` transitions without driver gas.

Phase 4: Highway lead and cut-in

- Use moderate following distance.
- Observe one normal cut-in only if traffic naturally creates it.
- Verify no phantom brake when radar points appear off-path.
- Run radar readiness report after the drive.

Phase 5: Intersections

- Test familiar low-risk intersections.
- Treat yellow lights as manual responsibility.
- For red lights, verify model path/stop-line behavior before trusting decel.
- If the vehicle stops too far from the white line, adjust stop-line evidence thresholds only after logs show the model consistently tracks the line.

## Tuning Rules

Change one family at a time:

1. Lead launch: `CARNIVAL_LEAD_DEPART_*`.
2. Stop hold: `CARNIVAL_RADAR_STOP_HOLD_*`.
3. Red-light stop-line guard: `CARNIVAL_RED_LIGHT_*` and `CARNIVAL_PRE_RED_*`.
4. Lateral EPS protection: `CARNIVAL_4TH_GEN_EPS_*`.
5. Torque override: `opendbc_repo/opendbc/car/torque_data/override.toml`.

Do not tune from feel alone. Every tuning pass should include:

- Before/after route IDs.
- Relevant warning timestamps.
- `scan_longitudinal_quality.py` output.
- `run_carnival_radar_longitudinal_readiness.py` output.
- Targeted unit tests for changed code.

## Current Bottom Line

The correct goal is not "Tesla-level autonomy." The practical target is a smooth and conservative driver-assistance tune:

- openpilot E2E handles scene-level longitudinal intent.
- Carnival radar confirms real object geometry.
- SNITHPilot smooths stop-and-go and prevents false radar-only braking.
- MADS and Carnival torque tuning keep lateral stable without EPS faults.

This is the safe path toward Tesla-like smoothness without pretending the stack has Tesla-like semantic traffic-light certainty.
