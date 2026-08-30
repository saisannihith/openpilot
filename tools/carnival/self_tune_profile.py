#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import shlex
import subprocess
from typing import Any


FOLLOW_KEYS = ("AggressiveFollow", "StandardFollow", "RelaxedFollow", "TrafficFollow")
SAFE_LIMITS = {
  "AggressiveFollow": (0.5, 3.0),
  "StandardFollow": (0.5, 3.0),
  "RelaxedFollow": (0.5, 3.0),
  "TrafficFollow": (0.5, 2.5),
  "ForceStopDistanceOffset": (-20.0, 20.0),
}


def latest_report(path: Path) -> dict[str, Any]:
  payload = json.loads(path.read_text(encoding="utf-8"))
  if not isinstance(payload, list) or not payload:
    raise ValueError("report JSON must contain at least one route")
  return payload[-1]


def build_delta_plan(report: dict[str, Any]) -> dict[str, float]:
  score = report.get("scorecard", {})
  long_data = report.get("longitudinal", {})
  missed = int(score.get("missed_stop_events", 0))
  min_ttc = long_data.get("min_ttc")
  plan: dict[str, float] = {}
  if missed > 0 or (isinstance(min_ttc, (int, float)) and min_ttc < 2.0):
    plan.update(dict.fromkeys(FOLLOW_KEYS, 0.05))
  if missed > 0:
    plan["ForceStopDistanceOffset"] = 1.0
  return plan


def resolve_values(current: dict[str, float], deltas: dict[str, float]) -> dict[str, dict[str, Any]]:
  resolved: dict[str, dict[str, Any]] = {}
  for key, delta in deltas.items():
    if key not in SAFE_LIMITS or key not in current:
      continue
    before = float(current[key])
    low, high = SAFE_LIMITS[key]
    after: float | int = max(low, min(high, before + float(delta)))
    if key == "ForceStopDistanceOffset":
      after = int(round(after))
    else:
      after = round(after, 3)
    if float(after) != before:
      resolved[key] = {"before": before, "delta": float(delta), "after": after}
  return resolved


def read_param_values(params, keys) -> dict[str, float]:
  values: dict[str, float] = {}
  for key in keys:
    raw = params.get(key, return_default=True)
    if raw is None:
      continue
    text = raw.decode() if isinstance(raw, bytes) else str(raw)
    values[key] = float(text)
  return values


def ssh_base(device: str, ssh_key: Path | None) -> list[str]:
  command = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=10"]
  if ssh_key is not None:
    command += ["-i", str(ssh_key), "-o", "IdentitiesOnly=yes"]
  return command + [f"comma@{device}"]


def remote_param(device: str, key: str, ssh_key: Path | None) -> str:
  code = f'from openpilot.common.params import Params; print((Params().get({key!r}, return_default=True) or b"").decode())'
  result = subprocess.run(ssh_base(device, ssh_key) + [f"cd /data/openpilot && /usr/local/venv/bin/python3 -c {shlex.quote(code)}"],
                          check=True, text=True, capture_output=True, timeout=20)
  return result.stdout.strip()


def resolve_plan(device: str, deltas: dict[str, float], ssh_key: Path | None) -> dict[str, dict[str, Any]]:
  current = {key: float(remote_param(device, key, ssh_key)) for key in deltas}
  return resolve_values(current, deltas)


def main() -> int:
  parser = argparse.ArgumentParser(description="Generate a read-only Carnival tuning suggestion report.")
  parser.add_argument("report", type=Path, help="JSON output from collect_and_report.py")
  parser.add_argument("--device", help="comma SSH IP; required to resolve current values")
  parser.add_argument("--ssh-key", type=Path, default=Path.home() / ".ssh" / "id_ed25519")
  parser.add_argument("--snapshot-dir", type=Path, default=Path("drive_reports"))
  args = parser.parse_args()

  report = latest_report(args.report)
  deltas = build_delta_plan(report)
  result: dict[str, Any] = {"route": report.get("route", ""), "deltas": deltas, "readOnly": True}
  if args.device and deltas:
    key = args.ssh_key if args.ssh_key.exists() else None
    resolved = resolve_plan(args.device, deltas, key)
    result["resolved"] = resolved

  args.snapshot_dir.mkdir(parents=True, exist_ok=True)
  snapshot = args.snapshot_dir / f"carnival-profile-{datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
  snapshot.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  print(f"Wrote {snapshot}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
