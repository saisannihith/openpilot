#!/usr/bin/env python3
"""Estimate command-to-aEgo delay from recorded openpilot-long routes."""
import argparse
import json
from pathlib import Path

import numpy as np

from openpilot.tools.lib.logreader import LogReader


DT = 0.05
MAX_LAG = 1.20
MIN_RUN_SECONDS = 10.0
WINDOW_SECONDS = 12.0
WINDOW_STEP_SECONDS = 3.0
MIN_COMMAND_RANGE = 0.50
MIN_ACCEL_RANGE = 0.25
MIN_CORRELATION = 0.60
MIN_PEAK_MARGIN = 0.025


def _correlation(x, y):
  x = np.asarray(x, dtype=np.float64)
  y = np.asarray(y, dtype=np.float64)
  x -= np.mean(x)
  y -= np.mean(y)
  denom = np.linalg.norm(x) * np.linalg.norm(y)
  return float(np.dot(x, y) / denom) if denom > 1e-9 else float("nan")


def _window_estimate(command, actual):
  if np.ptp(command) < MIN_COMMAND_RANGE or np.ptp(actual) < MIN_ACCEL_RANGE:
    return None

  # Remove slow grade/drag/load trends while preserving actuator transitions.
  smooth_n = max(3, int(round(2.0 / DT)))
  kernel = np.ones(smooth_n, dtype=np.float64) / smooth_n
  command_hp = command - np.convolve(command, kernel, mode="same")
  actual_hp = actual - np.convolve(actual, kernel, mode="same")

  max_lag_steps = int(round(MAX_LAG / DT))
  scores = [_correlation(command_hp, actual_hp)]
  for lag in range(1, max_lag_steps + 1):
    scores.append(_correlation(command_hp[:-lag], actual_hp[lag:]))
  scores = np.asarray(scores)
  if not np.isfinite(scores).any():
    return None

  best_idx = int(np.nanargmax(scores))
  # An optimum at either search boundary is not an identified delay. In closed-loop driving,
  # command and aEgo commonly peak at zero lag because the controller reacts to aEgo itself.
  if best_idx in (0, max_lag_steps):
    return None
  best_corr = float(scores[best_idx])
  competitors = scores.copy()
  competitors[max(0, best_idx - 1):best_idx + 2] = np.nan
  second_corr = float(np.nanmax(competitors)) if np.isfinite(competitors).any() else -1.0
  if best_corr < MIN_CORRELATION or best_corr - second_corr < MIN_PEAK_MARGIN:
    return None
  return {"delay": best_idx * DT, "correlation": best_corr, "peakMargin": best_corr - second_corr}


def _valid_sample(cc, cs):
  return bool(
    cc is not None and cc.longActive and not cs.gasPressed and not cs.brakePressed and not cs.standstill and
    cs.vEgo >= 2.0 and abs(cc.actuators.accel) >= 0.12 and abs(cs.steeringAngleDeg) <= 15.0 and
    np.isfinite((cc.actuators.accel, cs.aEgo, cs.vEgo, cs.steeringAngleDeg)).all()
  )


def _extract_runs(path):
  runs, run = [], []
  latest_cc = None
  latest_cc_t = -np.inf
  last_sample_t = -np.inf
  for msg in LogReader(str(path)):
    t = float(msg.logMonoTime) * 1e-9
    which = msg.which()
    if which == "carControl":
      latest_cc = msg.carControl
      latest_cc_t = t
    elif which == "carState":
      cs = msg.carState
      valid = t - latest_cc_t <= 0.15 and _valid_sample(latest_cc, cs)
      if not valid or t - last_sample_t > 0.20:
        if run:
          runs.append(run)
        run = []
      if valid and (not run or t - run[-1][0] >= DT * 0.8):
        run.append((t, float(latest_cc.actuators.accel), float(cs.aEgo)))
        last_sample_t = t
  if run:
    runs.append(run)
  return runs


def analyze(paths):
  estimates = []
  usable_runs = 0
  for path in paths:
    for run in _extract_runs(path):
      if len(run) < int(MIN_RUN_SECONDS / DT):
        continue
      raw = np.asarray(run)
      times = np.arange(raw[0, 0], raw[-1, 0], DT)
      if len(times) < int(MIN_RUN_SECONDS / DT):
        continue
      usable_runs += 1
      command = np.interp(times, raw[:, 0], raw[:, 1])
      actual = np.interp(times, raw[:, 0], raw[:, 2])
      window_n = int(WINDOW_SECONDS / DT)
      step_n = int(WINDOW_STEP_SECONDS / DT)
      for start in range(0, len(times) - window_n + 1, step_n):
        estimate = _window_estimate(command[start:start + window_n], actual[start:start + window_n])
        if estimate is not None:
          estimate["path"] = str(path)
          estimate["startMonoTime"] = float(times[start])
          estimates.append(estimate)

  delays = np.asarray([e["delay"] for e in estimates], dtype=np.float64)
  median = float(np.median(delays)) if len(delays) else None
  mad = float(np.median(np.abs(delays - median))) if len(delays) else None
  accepted = bool(len(delays) >= 8 and mad is not None and mad <= 0.10 and
                  median is not None and DT < median < MAX_LAG - DT)
  return {
    "contract": "shadow-only; no control change is justified by this report alone",
    "files": len(paths),
    "usableRuns": usable_runs,
    "acceptedWindows": len(estimates),
    "medianDelay": median,
    "medianAbsoluteDeviation": mad,
    "stableEstimate": accepted,
    "windows": estimates,
  }


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("logs", nargs="+", type=Path)
  parser.add_argument("--output", type=Path)
  args = parser.parse_args()
  report = analyze(args.logs)
  payload = json.dumps(report, indent=2)
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(payload + "\n")
  print(payload)


if __name__ == "__main__":
  main()
