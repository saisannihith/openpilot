# 2024 Kia Carnival Debug Automation

One-command workflow for pulling comma logs and producing a SNITHPilot tuning report.

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

The radar candidate remains shadow-only until lateral and raw velocity fields are decoded safely.
