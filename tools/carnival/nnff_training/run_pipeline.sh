#!/usr/bin/env bash
set -euo pipefail

export PATH="$HOME/.local/bin:$HOME/.juliaup/bin:$PATH"

OPENPILOT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PIPELINE_ROOT="$OPENPILOT_ROOT/tools/carnival/nnff_training"
NNLC_TOOLS="${NNLC_TOOLS:-$HOME/openpilot-nnlc-tools}"
NNLC_TOOLS_URL="https://github.com/amzoo/openpilot-nnlc-tools.git"
NNLC_TOOLS_COMMIT="c69c380ea10f61060fee43c3edb063aa8470b775"
RLOG_ROOT="${RLOG_ROOT:-$OPENPILOT_ROOT/drive-logs}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$HOME/nnff-runs}"
RUN_NAME="$(date +%Y%m%d-%H%M%S)-carnival-nnff"
RUN_DIR="$OUTPUT_ROOT/$RUN_NAME"
MODE="gpu"
PREPARE_ONLY=0

usage() {
  cat <<EOF
Usage: $0 [--cpu] [--prepare-only] [--rlogs PATH] [--output PATH]

Build and validate a 2024 Kia Carnival NNFF candidate. This script never
deploys a model or changes steering control.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cpu) MODE="cpu"; shift ;;
    --prepare-only) PREPARE_ONLY=1; shift ;;
    --rlogs) RLOG_ROOT="$2"; shift 2 ;;
    --output) OUTPUT_ROOT="$2"; RUN_DIR="$OUTPUT_ROOT/$RUN_NAME"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 64 ;;
  esac
done

mkdir -p "$OUTPUT_ROOT" "$RUN_DIR"
exec > >(tee "$RUN_DIR/pipeline.log") 2>&1

echo "SNITHPilot root: $OPENPILOT_ROOT"
echo "NNLC tools:      $NNLC_TOOLS"
echo "Rlogs:           $RLOG_ROOT"
echo "Output:          $RUN_DIR"
echo "Training mode:   $MODE"

if [[ ! -d "$NNLC_TOOLS/.git" ]]; then
  git clone "$NNLC_TOOLS_URL" "$NNLC_TOOLS"
fi

actual_commit="$(git -C "$NNLC_TOOLS" rev-parse HEAD)"
if [[ "$actual_commit" != "$NNLC_TOOLS_COMMIT" ]]; then
  echo "Refusing unreviewed NNLC tools commit $actual_commit" >&2
  echo "Expected $NNLC_TOOLS_COMMIT" >&2
  exit 65
fi

if ! grep -q 'NNLC_SEED' "$NNLC_TOOLS/training/latmodel_temporal.jl"; then
  git -C "$NNLC_TOOLS" apply "$PIPELINE_ROOT/nnlc_tools.patch"
fi
grep -q '/ 0.3f0' "$NNLC_TOOLS/training/latmodel_temporal.jl"
grep -q 'LatControlNNFF.*positional production contract' "$NNLC_TOOLS/training/latmodel_temporal.jl"

cd "$NNLC_TOOLS"
if [[ ! -x .venv/bin/python ]]; then
  uv venv --python 3.11
fi
uv pip install -e '.[dev,train]' pyarrow
uv run pytest -q -m 'not slow'

if ! command -v julia >/dev/null 2>&1; then
  export PATH="$HOME/.juliaup/bin:$PATH"
fi
if ! command -v julia >/dev/null 2>&1; then
  curl -fsSL https://install.julialang.org | sh -s -- -y --default-channel 1.11
  export PATH="$HOME/.juliaup/bin:$PATH"
fi

if ! julia -e 'using Flux, CSV, DataFrames, JSON' >/dev/null 2>&1; then
  julia training/install_packages.jl
fi

uv run python "$PIPELINE_ROOT/prepare_dataset.py" "$RLOG_ROOT" "$RUN_DIR" || prep_status=$?
prep_status="${prep_status:-0}"
if [[ "$prep_status" -ne 0 && "$prep_status" -ne 2 ]]; then
  exit "$prep_status"
fi

uv run nnlc-visualize "$RUN_DIR/training_inputs/KIA_CARNIVAL_4TH_GEN.csv" \
  -o "$RUN_DIR/coverage.png" --torque-scatter

ln -sfn "$RUN_DIR" "$OUTPUT_ROOT/latest"
if [[ "$PREPARE_ONLY" -eq 1 ]]; then
  echo "Preparation complete. Review $RUN_DIR/dataset_manifest.json"
  exit "$prep_status"
fi

train_args=()
if [[ "$MODE" == "cpu" ]]; then
  train_args+=(--cpu)
fi
NNLC_SEED=20240828 bash training/run.sh "$RUN_DIR/training_inputs" "${train_args[@]}"

set +e
uv run python "$PIPELINE_ROOT/validate_models.py" "$RUN_DIR"
validation_status=$?
set -e

echo
echo "Pipeline complete."
echo "Dataset:   $RUN_DIR/dataset_manifest.json"
echo "Validation: $RUN_DIR/validation_report.json"
echo "Latest:    $OUTPUT_ROOT/latest"
echo "No model was deployed to SNITHPilot or the comma device."
exit "$validation_status"
