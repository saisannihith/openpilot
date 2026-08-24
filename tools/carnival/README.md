# 2024 Kia Carnival Control Center

The comma 3x now exposes a dedicated `Carnival` settings page between `StarPilot` and `Device`. It contains:

- Carnival Confidence Governor live state and enable control.
- Self-Tuning Drive Profile recommendations with guarded apply and one-tap revert.
- Radar-Vision Lead Fusion HUD state and enable control.
- Intersection Stop Controller live state and enable control.
- EPS Fault Predictor live risk and enable control.
- Route Replay Scorecard with on-device run status and scores.

Route replay is on demand by default. Press `Run Now`, or explicitly enable `CarnivalAutoAnalyze`, and the low-priority offroad `carnival_analyzerd` process writes:

- `/data/media/0/carnival_reports/carnival-report-*.md`
- `/data/media/0/carnival_reports/carnival-report-*.json`

The latest scorecard, pending profile, applied snapshot, report path, and any error are also stored in Params for the native UI. Analysis never runs onroad.
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

The confidence governor only softens positive acceleration for a low-confidence, vision-only lead. It cannot add braking. The EPS predictor is bounded to a 12% pre-taper before the existing platform guard, and the dedicated intersection controller only owns the final low-speed hold/release phase.

## Self-Tuning Profile

Preview bounded changes from a generated report:

```bash
python tools/carnival/self_tune_profile.py drive_reports/carnival-report-YYYYMMDD-HHMMSS.json --device 192.168.68.68
```

Apply only the allowlisted resolved changes and write a before/after snapshot:

```bash
python tools/carnival/self_tune_profile.py drive_reports/carnival-report-YYYYMMDD-HHMMSS.json --device 192.168.68.68 --apply
```

Automatic analysis and automatic application are disabled by default. When explicitly enabled in the Carnival UI, only tiny allowlisted follow-time and force-stop-offset changes can apply offroad. A before/after snapshot is always written for revert. Torque, lane offset, curve speed, radar velocity, and controller constants remain recommendation-only until directional and fault evidence is strong enough.

The radar candidate remains shadow-only until lateral and raw velocity fields are decoded safely.
