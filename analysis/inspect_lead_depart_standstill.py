#!/usr/bin/env python3
from __future__ import annotations

import argparse

from openpilot.tools.lib.logreader import LogReader, ReadMode


def main() -> int:
  parser = argparse.ArgumentParser(description="Print standstill/longcontrol state around a lead-departure event.")
  parser.add_argument("qlog")
  parser.add_argument("--start", type=float, default=44.5)
  parser.add_argument("--end", type=float, default=51.0)
  args = parser.parse_args()

  latest = {}
  start_ns = None
  for msg in LogReader(args.qlog, default_mode=ReadMode.QLOG):
    which = msg.which()
    if start_ns is None and which in ("carState", "longitudinalPlan", "controlsState"):
      start_ns = msg.logMonoTime
    if start_ns is None:
      continue
    if which in ("carState", "carControl", "controlsState", "longitudinalPlan", "starpilotPlan", "radarState"):
      latest[which] = getattr(msg, which)
    if which != "longitudinalPlan" or not all(k in latest for k in ("carState", "controlsState", "longitudinalPlan", "radarState")):
      continue

    t = (msg.logMonoTime - start_ns) / 1e9
    if not args.start <= t <= args.end:
      continue

    cs = latest["carState"]
    controls = latest["controlsState"]
    plan = latest["longitudinalPlan"]
    lead = latest["radarState"].leadOne
    plan_accel = plan.accels[0] if len(plan.accels) else 0.0
    source = getattr(plan, "longitudinalPlanSource", "")
    print(
      "t={:.2f} v={:.2f} standstill={} cruiseStandstill={} brake={} gas={} "
      "longState={} enabled={} active={} shouldStop={} planA={:.2f} source={} "
      "lead={} d={:.2f} vLead={:.2f} aLead={:.2f}".format(
        t,
        cs.vEgo,
        cs.standstill,
        cs.cruiseState.standstill,
        cs.brakePressed,
        cs.gasPressed,
        controls.longControlState.raw,
        getattr(controls, "enabled", None),
        getattr(controls, "active", None),
        plan.shouldStop,
        plan_accel,
        source,
        lead.status,
        lead.dRel,
        lead.vLead,
        lead.aLeadK,
      )
    )

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
