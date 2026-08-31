#!/usr/bin/env python3
"""Produce one bounded verdict for the 2024 Carnival R0100 integration."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from openpilot.selfdrive.controls.radard import get_RadarState_from_carnival_confirmation
from openpilot.tools.lib.logreader import LogReader, ReadMode
from opendbc.car.hyundai.radar_interface import (
  CARNIVAL_4TH_GEN_OBJECT_BUS,
  CARNIVAL_4TH_GEN_OBJECT_END_ADDR,
  CARNIVAL_4TH_GEN_OBJECT_LEN,
  CARNIVAL_4TH_GEN_OBJECT_START_ADDR,
  CARNIVAL_4TH_GEN_RELATIVE_ACCELERATION_SPEC,
  CARNIVAL_4TH_GEN_TRACK_ID_BASE,
  CARNIVAL_4TH_GEN_LATERAL_VELOCITY_SPEC,
  carnival_radar_frame_valid,
  carnival_radar_object_valid,
  decode_carnival_radar_object,
)


def load_json(path: Path) -> dict[str, Any]:
  with path.open(encoding="utf-8") as stream:
    return json.load(stream)


def scan_sample_log(path: Path) -> dict[str, int]:
  counts = {"frames": 0, "crcValid": 0, "crcInvalid": 0, "activeObjects": 0}
  for msg in LogReader(str(path), default_mode=ReadMode.AUTO_INTERACTIVE, sort_by_time=False):
    if msg.which() != "can":
      continue
    for can in msg.can:
      address = int(can.address)
      dat = bytes(can.dat)
      if (int(can.src) != CARNIVAL_4TH_GEN_OBJECT_BUS or
          not CARNIVAL_4TH_GEN_OBJECT_START_ADDR <= address <= CARNIVAL_4TH_GEN_OBJECT_END_ADDR or
          len(dat) != CARNIVAL_4TH_GEN_OBJECT_LEN):
        continue
      counts["frames"] += 1
      if not carnival_radar_frame_valid(address, dat):
        counts["crcInvalid"] += 1
        continue
      counts["crcValid"] += 1
      counts["activeObjects"] += sum(
        carnival_radar_object_valid(decode_carnival_radar_object(dat, offset))
        for offset in (0, 128)
      )
  return counts


def verify_model_led_fusion() -> dict[str, Any]:
  radar_v_rel = -7.5
  model_v_ego = 20.0
  model_abs_speed = 16.25
  track = SimpleNamespace(
    dRel=31.0,
    yRel=0.2,
    vRel=radar_v_rel,
    identifier=CARNIVAL_4TH_GEN_TRACK_ID_BASE + 7,
    is_potential_fcw=lambda probability: probability > 0.9,
  )
  lead = SimpleNamespace(v=[model_abs_speed], a=[-0.8])
  result = get_RadarState_from_carnival_confirmation(track, lead, 19.5, model_v_ego, 0.8)
  expected_model_v_rel = model_abs_speed - model_v_ego
  return {
    "radarVRel": radar_v_rel,
    "modelVRel": expected_model_v_rel,
    "publishedVRel": result["vRel"],
    "publishedAcceleration": result["aLeadK"],
    "modelVelocityUsed": math.isclose(result["vRel"], expected_model_v_rel, abs_tol=1e-9),
    "rawRadarVelocityNotSubstituted": not math.isclose(result["vRel"], radar_v_rel, abs_tol=1e-9),
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--historical-metadata", type=Path, required=True)
  parser.add_argument("--fresh-metadata", type=Path, required=True)
  parser.add_argument("--velocity-sweep", type=Path, required=True)
  parser.add_argument("--sample-log", type=Path, required=True)
  parser.add_argument("--out", type=Path, required=True)
  args = parser.parse_args()

  historical = load_json(args.historical_metadata)
  fresh = load_json(args.fresh_metadata)
  velocity = load_json(args.velocity_sweep)
  historical_aggregate = historical["aggregate"]
  fresh_routes = fresh["routes"]
  generic_policies = [
    policy for policy in velocity["policies"]
    if not policy["policy"].get("primary_only", False)
  ]
  fake_primary_policies = [
    policy["policy"]["name"] for policy in velocity["policies"]
    if policy["policy"].get("primary_only", False)
  ]
  generic_actuation_candidates = [
    policy["policy"]["name"] for policy in generic_policies
    if policy.get("actuationReady", False)
  ]
  generic_policy_names = {policy["policy"]["name"] for policy in generic_policies}
  harmful_generic_samples = sum(policy.get("harmfulSamplesOver0_5", 0) for policy in generic_policies)
  fresh_truth_samples = sum(route["associationPolicy"]["truthSamples"] for route in fresh_routes.values())
  fresh_exact_samples = sum(route["associationPolicy"]["bestInnovationExact"] for route in fresh_routes.values())
  sample = scan_sample_log(args.sample_log)
  fusion = verify_model_led_fusion()

  checks = {
    "historicalCorpusLarge": historical_aggregate["activeSlotSamples"] >= 1_000_000,
    "requiredRadarPointContractDecoded": historical_aggregate["openpilotMinimumRadarContractComplete"],
    "historicalStateGateRejected": not historical_aggregate["stateCandidateReady"],
    "freshAssociationHasIndependentTruth": fresh_truth_samples >= 1_000,
    "freshAssociationExact": fresh_truth_samples > 0 and fresh_exact_samples == fresh_truth_samples,
    "genericRawVelocitySubstitutionRejected": not generic_actuation_candidates,
    "genericRawVelocityHasRecordedHarm": harmful_generic_samples > 0,
    "legacyFakePrimaryPoliciesExcluded": not generic_policy_names.intersection(fake_primary_policies),
    "sampleFramesPresent": sample["frames"] > 0,
    "sampleCrcClean": sample["crcValid"] > 0 and sample["crcInvalid"] == 0,
    "sampleActiveObjectsPresent": sample["activeObjects"] > 0,
    "optionalLayoutMatchesIndependentDecode": (
      CARNIVAL_4TH_GEN_LATERAL_VELOCITY_SPEC == (104, 9, 0.05, 0.6) and
      CARNIVAL_4TH_GEN_RELATIVE_ACCELERATION_SPEC == (115, 9, 0.1, 0.0)
    ),
    "modelVelocityUsedForControl": fusion["modelVelocityUsed"],
    "rawRadarVelocityNotSubstituted": fusion["rawRadarVelocityNotSubstituted"],
  }
  report = {
    "status": "pass" if all(checks.values()) else "fail",
    "checks": checks,
    "evidence": {
      "historicalActiveObjects": historical_aggregate["activeSlotSamples"],
      "historicalModelSelectedObjects": historical_aggregate["modelSelectedSamples"],
      "freshAssociationTruthSamples": fresh_truth_samples,
      "freshAssociationExactSamples": fresh_exact_samples,
      "velocitySweepMatchedSamples": velocity["matchedSamples"],
      "harmfulGenericRawVelocitySamples": harmful_generic_samples,
      "excludedFakePrimaryPolicies": fake_primary_policies,
      "sampleLog": str(args.sample_log),
      "sample": sample,
      "fusion": fusion,
    },
    "contract": {
      "publication": "CRC-valid persistent R0100 identity, distance, lateral position, and relative velocity",
      "association": "standard model-first normalized innovation with previous-track hysteresis",
      "control": "R0100 geometry plus model velocity and acceleration",
      "optionalDynamics": "decoded in the DBC but intentionally not published until independently validated",
      "notRequiredByRadarPoint": ["OEM classification", "OEM lane assignment", "OEM primary selector"],
    },
  }
  args.out.parent.mkdir(parents=True, exist_ok=True)
  args.out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  print(json.dumps(report, indent=2, sort_keys=True))
  return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
