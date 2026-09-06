#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode


def safe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def as_text(value: Any) -> str:
  try:
    return str(value)
  except Exception:
    return ""


def snapshot(value: Any, fields: tuple[str, ...]) -> dict[str, Any]:
  result: dict[str, Any] = {}
  for field in fields:
    item = safe_attr(value, field, None)
    if item is None:
      continue
    if isinstance(item, (str, int, float, bool)):
      result[field] = item
    else:
      try:
        result[field] = list(item)
      except Exception:
        result[field] = as_text(item)
  return result


def route_name(path: Path) -> str:
  return path.parent.name


def inspect(path: Path) -> dict[str, Any]:
  start_ns: int | None = None
  latest: dict[str, Any] = {}
  last_alert: tuple[Any, ...] | None = None
  last_events: tuple[str, ...] | None = None
  alerts: list[dict[str, Any]] = []
  events: list[dict[str, Any]] = []

  for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=True):
    mono_ns = int(msg.logMonoTime)
    if start_ns is None:
      start_ns = mono_ns
    which = msg.which()
    if which in ("carState", "carControl", "controlsState", "selfdriveState", "starpilotSelfdriveState"):
      latest[which] = getattr(msg, which)

    if which in ("selfdriveState", "starpilotSelfdriveState"):
      state = getattr(msg, which)
      current = (
        which,
        as_text(safe_attr(state, "alertText1", "")),
        as_text(safe_attr(state, "alertText2", "")),
        as_text(safe_attr(state, "alertType", "")),
        as_text(safe_attr(state, "alertStatus", "")),
        as_text(safe_attr(state, "alertSize", "")),
      )
      if current != last_alert and any(current[1:]):
        car = latest.get("carState")
        cc = latest.get("carControl")
        alerts.append({
          "t": round((mono_ns - start_ns) / 1e9, 3),
          "state": current[0],
          "alertText1": current[1],
          "alertText2": current[2],
          "alertType": current[3],
          "alertStatus": current[4],
          "alertSize": current[5],
          "latActive": bool(safe_attr(cc, "latActive", False)),
          "steerFaultTemporary": bool(safe_attr(car, "steerFaultTemporary", False)),
          "steeringPressed": bool(safe_attr(car, "steeringPressed", False)),
          "steeringTorque": float(safe_attr(car, "steeringTorque", 0.0) or 0.0),
          "steeringAngleDeg": float(safe_attr(car, "steeringAngleDeg", 0.0) or 0.0),
          "vEgo": float(safe_attr(car, "vEgo", 0.0) or 0.0),
        })
      last_alert = current

    if which == "controlsState":
      state = msg.controlsState
      current_events = tuple(as_text(event) for event in safe_attr(state, "events", []))
      if current_events != last_events and current_events:
        events.append({
          "t": round((mono_ns - start_ns) / 1e9, 3),
          "events": current_events,
          "enabled": bool(safe_attr(state, "enabled", False)),
          "active": bool(safe_attr(state, "active", False)),
          "latActive": bool(safe_attr(latest.get("carControl"), "latActive", False)),
          "steeringPressed": bool(safe_attr(latest.get("carState"), "steeringPressed", False)),
        })
      last_events = current_events

  return {"route": route_name(path), "alerts": alerts, "controlEvents": events}


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+", type=Path)
  parser.add_argument("--out", type=Path)
  args = parser.parse_args()
  result = {"routes": [inspect(path) for path in args.logs]}
  text = json.dumps(result, indent=2, sort_keys=True)
  print(text)
  if args.out:
    args.out.write_text(text + "\n", encoding="utf-8")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
