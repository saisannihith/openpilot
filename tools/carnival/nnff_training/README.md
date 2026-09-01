# Carnival NNFF Training

This pipeline builds reproducible, route-held-out NNFF candidates for the 2024
Kia Carnival. It is intentionally separate from the live steering controller.

It performs the following automatically:

1. Pins and validates the external NNLC tooling revision.
2. Applies reviewed fixes for deterministic training, temporal jerk scaling,
   and StarPilot's exact 18-input positional contract.
3. Extracts rlogs without allowing temporal features to cross segment bounds.
4. Excludes driver overrides, EPS faults, saturation, lane changes, inactive
   control, standstill, and context windows contaminated by those states.
5. Preserves route, segment, software commit, and fingerprint provenance.
6. Generates one full dataset plus leave-one-route-out training folds.
7. Trains every candidate with a fixed seed.
8. Validates held-out error, sign agreement, finite output, monotonic response,
   zero-demand output, odd symmetry, and normalized output bounds.
9. Produces a static-training decision only; static tests never claim shadow or
   actuation readiness.
10. Audits the trained network against StarPilot's live model-planner input
    contract and records the exact driving-model identity from each route.

Run from WSL:

```bash
cd ~/snithpilot-ui-openpilot
bash tools/carnival/nnff_training/run_pipeline.sh
```

Useful alternatives:

```bash
# Validate extraction and coverage without training
bash tools/carnival/nnff_training/run_pipeline.sh --prepare-only

# Force CPU training
bash tools/carnival/nnff_training/run_pipeline.sh --cpu
```

Outputs are written beneath `~/nnff-runs/`; `~/nnff-runs/latest` points to the
most recent run. The pipeline never copies a model into StarPilot and never
changes the comma device. A passing static candidate still requires a live
contract audit and reviewed shadow telemetry before any actuation test.

Static validation is not driving-model independent. NNFF consumes future
`modelV2` acceleration and orientation values, so every intended driving-model
family must pass `audit_runtime_compatibility.py` on matching logs before a
shadow test. Do not infer RDF or other model compatibility from a candidate
trained and replayed only with CD210.

## Speed-dependent torque gate

The physical-response parquet can also evaluate a speed-dependent torque
profile without giving it steering authority:

```bash
python analysis/analyze_carnival_speed_dependent_torque.py \
  ~/torque-speed-physical-20260901/clean_with_provenance.parquet \
  --output analysis/carnival_speed_dependent_torque_report.json
```

The analyzer mirrors torqued's total-least-squares fit, balances complete
routes and steering buckets, and leaves one route out at a time. Exit status 0
means every actuation gate passed; status 2 means the candidate was rejected.
The committed Carnival report rejects the current six-bin candidate, so no
speed-dependent runtime path is enabled.
