#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"

source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
cd "${REPO_ROOT}"
python -m nonconvex_timevarying_window.rot_sync_sc_togt.icra_experiments.03_sync_single.run_experiment \
  --outdir "${SCRIPT_DIR}/focused_results" "$@"
