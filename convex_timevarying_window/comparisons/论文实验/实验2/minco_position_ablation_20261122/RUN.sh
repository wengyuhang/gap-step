#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
root=$(pwd)
reference=$(realpath ../robust_ipopt_full_20261117)
source="$reference/source_inputs"
mkdir "$root/run.lock"
work=$(mktemp -d /dev/shm/minco_position_ablation_20261122.XXXXXX)
printf '%s\n' "$work" > "$root/SCRATCH_PATH"
printf '%s\n' "$$" > "$root/COORDINATOR_PID"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 SIP_MINCO_POSITION_ONLY=1
"$root/bin/audit" > "$root/MODEL_TEST.log"
mapfile -t levels < <(jq -r '.levels[]' "$source/PROTOCOL.json")
for level in "${levels[@]}"; do
    echo "START $level"
    "$root/bin/curvature_initials" "$source" "$level" "$work/inputs/$level" --robust-ipopt-profile > "$work/$level.prepare.log" 2>&1
    "$root/bin/native_sip_matrix" "$work/inputs/$level" "$source/assets/$level/scene.json" \
        "$work/$level" "$root/bin/sip_planner" "$root/bin/mesh_check" > "$work/$level.run.log" 2>&1
    tar --zstd -cf "$root/results/$level.tar.zst.tmp" -C "$work" \
        "$level" "inputs/$level" "$level.prepare.log" "$level.run.log"
    tar --zstd -tf "$root/results/$level.tar.zst.tmp" > /dev/null
    cp "$work/$level/SUMMARY.json" "$root/results/$level.json.tmp"
    mv "$root/results/$level.tar.zst.tmp" "$root/results/$level.tar.zst"
    mv "$root/results/$level.json.tmp" "$root/results/$level.json"
    "$root/bin/audit" "$work" "$reference" "$root"
    echo "COMPLETE $level"
done
printf '%s\n' complete > "$root/COMPLETION"
