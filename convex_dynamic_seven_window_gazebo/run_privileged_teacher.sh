#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd)
CHECKPOINT=${1:-$ROOT/../convex_timevarying_window/privileged_safe_teacher/results/smoke_mixed_20260911/teacher.pt}
"$ROOT/run_px4_nmpc_headless.sh"
conda run --no-capture-output -n wyh python "$ROOT/privileged_teacher_experiment.py" \
  --checkpoint "$CHECKPOINT" --recovery-fraction 0.80 --frequency 50
