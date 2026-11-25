#!/usr/bin/env bash
set -euo pipefail
cd /home/jack/wyh/时变窗口
P=convex_timevarying_window/comparisons/论文实验/实验2/robust_ipopt_full_20261117
mapfile -t levels < <(jq -r '.levels[]' "$P/source_inputs/PROTOCOL.json")
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$P/bin/native_sip_matrix" --campaign-robust "$P/source_inputs" "$P/results" "$P/bin/sip_planner" "$P/bin/mesh_check" "$P/bin/curvature_initials" "${levels[@]}"
