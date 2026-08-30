# 2024 Kia Carnival Control Center

The comma 3x now exposes a dedicated `Carnival` settings page between `StarPilot` and `Device`. It contains:

- Carnival Confidence Monitor live state.
- Read-only Self-Tuning Drive Profile recommendations.
- Radar-Vision Lead Fusion HUD state and enable control.
- EPS Fault Predictor live risk and enable control.
- Route Replay Scorecard with on-device run status and scores.

Compact route scoring is enabled by default. The existing manager checks for a settled new route every 10 seconds and starts the low-priority `carnival_analyzerd` only while work is pending, so there is no persistent analysis process. Press `Run Now` to repeat the latest route manually.

The on-device path prefers qlogs and skips raw CAN object retention. A 33-segment route completed in 32.6 seconds with a 72 MB peak on comma 3x. Full raw-radar decoding remains available from this script on a PC without `--compact`.

- `/data/media/0/carnival_reports/carnival-report-*.md`
- `/data/media/0/carnival_reports/carnival-report-*.json`

The latest scorecard, read-only suggestion profile, report path, and any error are also stored in Params for the native UI. Analysis never runs onroad.
The on-device scorecard prefers each segment's compact qlog to keep CPU and memory bounded; the full-rate rlogs remain untouched for deep PC diagnostics.

## Optional PC Workflow

The original one-command workflow remains available for pulling comma logs and producing a local report.

## Common Use

After a drive, with SSH enabled on the comma:

```bash
python tools/carnival/collect_and_report.py --device 192.168.68.68 --latest 1 --recent-segments-per-route 8
```

This copies only new `rlog/qlog` files, analyzes the latest route, and writes:

- `drive_reports/carnival-report-*.md`
- `drive_reports/carnival-report-*.json`

## Faster Local Recheck

```bash
python tools/carnival/collect_and_report.py --skip-pull --latest 1 --recent-segments-per-route 8
```

## Specific Route

```bash
python tools/carnival/collect_and_report.py --skip-pull --route 00000006--b531e9e9eb --recent-segments-per-route 8
```

## What It Checks

- Route mode: stock SCC vs openpilot longitudinal.
- Hidden Carnival radar candidate: `bus 1`, `0x180-0x184`, primarily `0x180` slot 1.
- Radar distance quality, coverage, and distance-derived velocity quality.
- Lateral events: steer override, steering availability warnings, torque clipping, steering intervention rate.
- Longitudinal events: lead source, min lead distance, min TTC, brake/gas intervention, harsh brake commands.
- Route scorecard: lateral, longitudinal, radar, and overall scores.
- Counts for interventions, uncorroborated brake events, missed-stop interventions, creep, and steering saturation.
- Small evidence-bounded tuning recommendations. They are never applied while driving.

## Live Carnival State

`carnivald` publishes `carnivalState` at model rate. The on-road Carnival Fusion HUD shows:

- vision versus radar-confirmed lead state and stale radar;
- cut-in candidate count;
- stop approach/hold state;
- combined confidence and EPS risk.

The confidence monitor cannot change acceleration or braking. The EPS predictor is lateral-only and remains bounded by the existing platform guard. StarPilot owns stopping, launch, red-light, follow-distance, and curve-speed behavior.

## Self-Tuning Profile

Preview read-only suggestions from a generated report:

```bash
python tools/carnival/self_tune_profile.py drive_reports/carnival-report-YYYYMMDD-HHMMSS.json --device 192.168.68.68
```

The profile is diagnostic only. It never changes StarPilot settings locally or over SSH. Torque, follow distance, stop distance, lane offset, curve speed, radar velocity, and controller constants remain under their normal StarPilot owners.

The radar candidate remains shadow-only until lateral and raw velocity fields are decoded safely.
