#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE/code"
export PYTHONPATH="$HERE/code"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec /home/jack/anaconda3/envs/wyh/bin/python -m convex_timevarying_window.comparisons.论文实验.实验2.higher_swing.matrix_worker "$@"
