#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import types
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
namespace = types.ModuleType("openpilot")
namespace.__path__ = [str(ROOT)]  # type: ignore[attr-defined]
sys.modules["openpilot"] = namespace

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.tools.lib.logreader import LogReader, ReadMode


def attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def main() -> None:
  parser = argparse.ArgumentParser(description="Replay logged lead lifecycle inputs through the current MPC lead processor")
  parser.add_argument("rlog", type=Path)
  parser.add_argument("--start", type=float, default=0.0)
  parser.add_argument("--end", type=float, default=1e9)
  parser.add_argument("--min-allowed-v", type=float)
  parser.add_argument("--min-increase", type=float)
  parser.add_argument("--lead-index", type=int, choices=(0, 1), default=0)
  parser.add_argument("--summary-only", action="store_true")
  args = parser.parse_args()

  latest: dict[str, Any] = {}
  start_ns: int | None = None
  mpc = LongitudinalMpc()
  rows: list[tuple[float, bool, bool, float, bool, float, float, float, float, float]] = []

  for msg in LogReader(str(args.rlog), default_mode=ReadMode.RLOG, sort_by_time=True):
    which = msg.which()
    if start_ns is None and which == "carState":
      start_ns = int(msg.logMonoTime)
    if which in ("carState", "starpilotPlan", "radarState", "modelV2"):
      latest[which] = getattr(msg, which)
    if which != "longitudinalPlan" or start_ns is None or len(latest) != 4:
      continue

    t = (int(msg.logMonoTime) - start_ns) / 1e9
    car_state = latest["carState"]
    starpilot_plan = latest["starpilotPlan"]
    radar_state = latest["radarState"]
    model = latest["modelV2"]
    v_ego = float(car_state.vEgo)
    mpc.set_cur_state(v_ego, float(car_state.aEgo))
    mpc.set_weights(v_ego=v_ego)

    tracking = bool(starpilot_plan.trackingLead)
    model_leads = list(attr(model, "leadsV3", []))
    lead_zero = radar_state.leadOne
    lead_one = radar_state.leadTwo
    trajectory_zero = mpc.process_lead(
      lead_zero,
      tracking,
      t_follow=float(starpilot_plan.tFollow),
      lead_index=0,
      model_lead=model_leads[0] if model_leads else None,
    )
    trajectory_one = mpc.process_lead(
      lead_one,
      tracking,
      t_follow=float(starpilot_plan.tFollow),
      lead_index=1,
      model_lead=model_leads[1] if len(model_leads) > 1 else None,
    )

    if args.start <= t <= args.end:
      rows.append((
        t,
        tracking,
        bool(lead_zero.status),
        float(lead_zero.vLead) if lead_zero.status else 0.0,
        bool(lead_one.status),
        float(lead_one.vLead) if lead_one.status else 0.0,
        float(trajectory_zero[0, 1]),
        float(trajectory_one[0, 1]),
        float(getattr(msg.longitudinalPlan, "leadTrajectoryV0", [0.0])[0]),
        float(getattr(msg.longitudinalPlan, "leadTrajectoryV1", [0.0])[0]),
      ))

  if not args.summary_only:
    print("t tracking status0 rawV0 status1 rawV1 replayV0 replayV1 loggedV0 loggedV1")
    for row in rows:
      replay_v = row[6 + args.lead_index]
      logged_v = row[8 + args.lead_index]
      if args.min_increase is not None and replay_v - logged_v < args.min_increase:
        continue
      print(
        f"{row[0]:7.3f} {int(row[1])} {int(row[2])} {row[3]:6.2f} {int(row[4])} {row[5]:6.2f} "
        f"{row[6]:8.2f} {row[7]:8.2f} {row[8]:8.2f} {row[9]:8.2f}"
      )

  replay_index = 6 + args.lead_index
  logged_index = 8 + args.lead_index
  replay_min = min((row[replay_index] for row in rows), default=float("inf"))
  logged_min = min((row[logged_index] for row in rows), default=float("inf"))
  max_increase = max((row[replay_index] - row[logged_index] for row in rows), default=0.0)
  max_decrease = max((row[logged_index] - row[replay_index] for row in rows), default=0.0)
  changed = sum(abs(row[replay_index] - row[logged_index]) > 0.5 for row in rows)
  print(
    f"summary segment={args.rlog.parent.name} leadIndex={args.lead_index} samples={len(rows)} "
    f"replayMinV={replay_min:.3f} loggedMinV={logged_min:.3f} "
    f"maxIncrease={max_increase:.3f} maxDecrease={max_decrease:.3f} changedOver0.5={changed}"
  )
  if args.min_allowed_v is not None and replay_min < args.min_allowed_v:
    raise SystemExit(f"replay lead velocity {replay_min:.3f} is below {args.min_allowed_v:.3f}")


if __name__ == "__main__":
  main()
