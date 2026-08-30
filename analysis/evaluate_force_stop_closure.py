#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

from openpilot.starpilot.controls.lib.starpilot_vcruise import (
  FORCE_STOP_APPROACH_DECEL,
  FAR_APPROACH_CONFIRM_MAX_WINDOW_M,
  FAR_APPROACH_CONFIRM_MIN_CLOSURE_RATIO,
  FAR_APPROACH_CONFIRM_MIN_TRAVEL_M,
  FAR_APPROACH_CONFIRM_RESET_GAP_M,
)

FORCE_STOP_HANDOFF_M = 6.0
from openpilot.tools.lib.logreader import LogReader


def evaluate(path: Path):
  segment = int(path.parent.name.rsplit("--", 1)[-1])
  v_ego = 0.0
  start_time = None
  episode = None
  episodes = []

  for msg in LogReader(str(path)):
    if start_time is None:
      start_time = msg.logMonoTime / 1e9
    t = segment * 60.0 + msg.logMonoTime / 1e9 - start_time
    which = msg.which()
    if which == "carState":
      v_ego = max(0.0, float(msg.carState.vEgo))
      continue
    if which != "starpilotPlan":
      continue

    plan = msg.starpilotPlan
    candidate = bool(plan.redLight and not plan.forcingStop)
    model_length = float(plan.forcingStopLength)
    if not candidate or model_length <= 0.0:
      if episode is not None:
        episodes.append(episode)
        episode = None
      continue

    if episode is None:
      episode = {
        "start": t, "end": t, "anchor": model_length, "last_d": model_length,
        "last_t": t, "travel": 0.0, "max_v": v_ego, "confirmed": False,
        "confirm_t": None, "confirm_d": None, "confirm_v": None, "confirm_target": None,
      }
      continue

    if episode["confirmed"]:
      if model_length > episode["anchor"] + FAR_APPROACH_CONFIRM_RESET_GAP_M:
        episode.update(anchor=model_length, travel=0.0, confirmed=False,
                       confirm_t=None, confirm_d=None, confirm_v=None, confirm_target=None)
      else:
        episode.update(end=t, last_d=model_length, last_t=t, max_v=max(episode["max_v"], v_ego))
        continue

    dt = max(0.0, t - episode["last_t"])
    episode["travel"] += v_ego * dt
    if (model_length > episode["anchor"] + FAR_APPROACH_CONFIRM_RESET_GAP_M
        or episode["travel"] >= FAR_APPROACH_CONFIRM_MAX_WINDOW_M):
      episode["anchor"] = model_length
      episode["travel"] = 0.0
      episode.update(confirm_t=None, confirm_d=None, confirm_v=None, confirm_target=None)
    model_drop = episode["anchor"] - model_length
    was_confirmed = episode["confirmed"]
    episode["confirmed"] |= bool(
      episode["travel"] >= FAR_APPROACH_CONFIRM_MIN_TRAVEL_M
      and model_drop >= episode["travel"] * FAR_APPROACH_CONFIRM_MIN_CLOSURE_RATIO
    )
    if episode["confirmed"] and not was_confirmed:
      episode["confirm_t"] = t
      episode["confirm_d"] = model_length
      episode["confirm_v"] = v_ego
      episode["confirm_target"] = math.sqrt(
        2.0 * FORCE_STOP_APPROACH_DECEL * max(model_length - FORCE_STOP_HANDOFF_M, 0.0)
      )
    episode.update(end=t, last_d=model_length, last_t=t, max_v=max(episode["max_v"], v_ego))

  if episode is not None:
    episodes.append(episode)
  return episodes


def main():
  parser = argparse.ArgumentParser(description="Evaluate far-stop closure evidence in recorded routes")
  parser.add_argument("paths", nargs="+", type=Path)
  args = parser.parse_args()

  for path in args.paths:
    for episode in evaluate(path):
      duration = episode["end"] - episode["start"]
      if duration < 0.4:
        continue
      print(
        f"{path.parent.name} t={episode['start']:.1f}-{episode['end']:.1f}s "
        + f"vMax={episode['max_v']:.1f}m/s d={episode['anchor']:.1f}->{episode['last_d']:.1f}m "
        + f"travel={episode['travel']:.1f}m confirmed={episode['confirmed']} "
        + (f"bind@{episode['confirm_t']:.1f}s d={episode['confirm_d']:.1f}m "
           + f"v={episode['confirm_v']:.1f}->{episode['confirm_target']:.1f}m/s"
           if episode["confirm_t"] is not None else "")
      )


if __name__ == "__main__":
  main()
