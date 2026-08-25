#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from openpilot.tools.lib.logreader import LogReader, ReadMode


SERVICES = {
  "carState", "carControl", "carOutput", "controlsState", "longitudinalPlan",
  "starpilotPlan", "radarState", "modelV2", "carnivalState",
}


def attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def finite(value: Any, default: float = 0.0) -> float:
  try:
    value = float(value)
  except Exception:
    return default
  return value if math.isfinite(value) else default


def segment(path: Path) -> int:
  for value in (path.parent.name, path.name):
    parts = value.split("--")
    if len(parts) >= 3:
      try:
        return int(parts[2].split(".", 1)[0])
      except ValueError:
        pass
  return -1


def lead_dict(lead: Any) -> dict[str, Any]:
  status = bool(attr(lead, "status", False))
  return {
    "status": status,
    "dRel": round(finite(attr(lead, "dRel")), 3) if status else None,
    "vRel": round(finite(attr(lead, "vRel")), 3) if status else None,
    "vLead": round(finite(attr(lead, "vLead")), 3) if status else None,
    "radar": bool(attr(lead, "radar", False)) if status else False,
    "modelProb": round(finite(attr(lead, "modelProb")), 3) if status else None,
  }


def sample(latest: dict[str, Any], mono_ns: int, route_start_ns: int, seg: int) -> dict[str, Any] | None:
  required = ("carState", "carControl", "controlsState", "longitudinalPlan", "starpilotPlan", "radarState")
  if not all(name in latest for name in required):
    return None

  cs = latest["carState"]
  cc = latest["carControl"]
  controls = latest["controlsState"]
  plan = latest["longitudinalPlan"]
  sp = latest["starpilotPlan"]
  radar = latest["radarState"]
  model_action = attr(latest.get("modelV2"), "action")
  output = attr(latest.get("carOutput"), "actuatorsOutput")
  carnival = latest.get("carnivalState")

  return {
    "t": round((mono_ns - route_start_ns) / 1e9, 3),
    "segment": seg,
    "vEgo": round(finite(attr(cs, "vEgo")), 3),
    "aEgo": round(finite(attr(cs, "aEgo")), 3),
    "standstill": bool(attr(cs, "standstill", False)),
    "cruiseStandstill": bool(attr(attr(cs, "cruiseState"), "standstill", False)),
    "gasPressed": bool(attr(cs, "gasPressed", False)),
    "brakePressed": bool(attr(cs, "brakePressed", False)),
    "longActive": bool(attr(cc, "longActive", False)),
    "longControlState": int(attr(attr(controls, "longControlState"), "raw", -1)),
    "shouldStop": bool(attr(plan, "shouldStop", False)),
    "planAccel": round(finite(attr(plan, "aTarget")), 3),
    "cmdAccel": round(finite(attr(attr(cc, "actuators"), "accel")), 3),
    "outAccel": round(finite(attr(output, "accel", attr(attr(cc, "actuators"), "accel", 0.0))), 3),
    "redLight": bool(attr(sp, "redLight", False)),
    "forcingStop": bool(attr(sp, "forcingStop", False)),
    "forcingStopLength": round(finite(attr(sp, "forcingStopLength")), 3),
    "modelShouldStop": bool(attr(model_action, "shouldStop", False)),
    "leadOne": lead_dict(attr(radar, "leadOne")),
    "leadTwo": lead_dict(attr(radar, "leadTwo")),
    "carnivalStopState": str(attr(carnival, "stopState", "missing")) if carnival is not None else "missing",
    "carnivalHold": bool(attr(carnival, "stopHoldActive", False)) if carnival is not None else False,
  }


def compact(s: dict[str, Any]) -> dict[str, Any]:
  return {key: s[key] for key in (
    "t", "segment", "vEgo", "aEgo", "standstill", "cruiseStandstill", "gasPressed", "brakePressed",
    "longControlState", "shouldStop", "planAccel", "cmdAccel", "outAccel", "redLight", "forcingStop",
    "forcingStopLength", "modelShouldStop", "leadOne", "leadTwo", "carnivalStopState", "carnivalHold",
  )}


def window(samples: list[dict[str, Any]], index: int, seconds: float = 3.0) -> list[dict[str, Any]]:
  center = samples[index]["t"]
  candidates = [s for s in samples if abs(s["t"] - center) <= seconds]
  # About 5 Hz is enough to expose the state handoff without a huge report.
  result: list[dict[str, Any]] = []
  last_t = -1e9
  for s in candidates:
    if s["t"] - last_t >= 0.18 or abs(s["t"] - center) < 0.06:
      result.append(compact(s))
      last_t = s["t"]
  return result


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("paths", nargs="+", type=Path)
  parser.add_argument("--output", type=Path)
  args = parser.parse_args()

  expanded_paths: list[Path] = []
  for path in args.paths:
    if path.is_dir():
      expanded_paths.extend(path.rglob("qlog*"))
    elif "*" in path.name:
      expanded_paths.extend(path.parent.glob(path.name))
    else:
      expanded_paths.append(path)
  paths = sorted(expanded_paths, key=segment)
  latest: dict[str, Any] = {}
  samples: list[dict[str, Any]] = []
  route_start_ns: int | None = None
  carnival_messages = 0
  manager: dict[str, dict[str, Any]] = {}

  for path in paths:
    seg = segment(path)
    for msg in LogReader(str(path), default_mode=ReadMode.AUTO, sort_by_time=True):
      which = msg.which()
      mono_ns = int(msg.logMonoTime)
      if route_start_ns is None and which in SERVICES:
        route_start_ns = mono_ns
      if which in SERVICES:
        latest[which] = getattr(msg, which)
        carnival_messages += int(which == "carnivalState")
      elif which == "managerState":
        for process in attr(msg.managerState, "processes", []):
          name = str(attr(process, "name", ""))
          if name in ("carnivald", "controlsd", "plannerd"):
            manager[name] = {
              "running": bool(attr(process, "running", False)),
              "shouldBeRunning": bool(attr(process, "shouldBeRunning", False)),
              "exitCode": int(attr(process, "exitCode", 0)),
            }
      if which == "longitudinalPlan" and route_start_ns is not None:
        value = sample(latest, mono_ns, route_start_ns, seg)
        if value is not None:
          samples.append(value)

  events: list[dict[str, Any]] = []
  standstill_since: float | None = None
  moved_after_stop = False
  for i, current in enumerate(samples):
    previous = samples[i - 1] if i else current
    clear_stop_context = False
    if current["standstill"] and not previous["standstill"]:
      standstill_since = current["t"]
      moved_after_stop = False
      events.append({"type": "standstill-entry", "at": compact(current), "window": window(samples, i)})
    if standstill_since is not None and not current["standstill"] and previous["standstill"]:
      events.append({"type": "standstill-exit", "heldFor": round(current["t"] - standstill_since, 2),
                     "at": compact(current), "window": window(samples, i)})
      clear_stop_context = True
    if standstill_since is not None and not moved_after_stop and current["vEgo"] > 0.12:
      moved_after_stop = True
      stopping_evidence = current["shouldStop"] or current["redLight"] or current["forcingStop"] or current["modelShouldStop"]
      events.append({"type": "creep-or-release", "stopEvidence": stopping_evidence,
                     "heldFor": round(current["t"] - standstill_since, 2), "at": compact(current),
                     "window": window(samples, i)})
    if standstill_since is not None and current["gasPressed"] and not previous["gasPressed"]:
      events.append({"type": "manual-gas-from-stop", "heldFor": round(current["t"] - standstill_since, 2),
                     "at": compact(current), "window": window(samples, i)})
    if standstill_since is not None and current["brakePressed"] and not previous["brakePressed"]:
      events.append({"type": "manual-brake-from-stop", "heldFor": round(current["t"] - standstill_since, 2),
                     "at": compact(current), "window": window(samples, i)})
    if previous["shouldStop"] and not current["shouldStop"] and current["vEgo"] < 0.5:
      events.append({"type": "should-stop-cleared", "at": compact(current), "window": window(samples, i)})
    if (current["longControlState"] == 2 and not current["shouldStop"] and current["planAccel"] > 0.15 and
        current["cmdAccel"] < -0.1):
      if not events or events[-1].get("type") != "release-deadlock" or current["t"] - events[-1]["at"]["t"] > 1.0:
        events.append({"type": "release-deadlock", "at": compact(current), "window": window(samples, i)})
    if clear_stop_context:
      standstill_since = None
      moved_after_stop = False

  result = {
    "files": len(paths),
    "samples": len(samples),
    "duration": round(samples[-1]["t"] - samples[0]["t"], 2) if samples else 0.0,
    "carnivalStateMessages": carnival_messages,
    "lastManagerState": manager,
    "events": events,
  }
  encoded = json.dumps(result, indent=2)
  if args.output:
    args.output.write_text(encoded + "\n")
  else:
    print(encoded)


if __name__ == "__main__":
  main()
