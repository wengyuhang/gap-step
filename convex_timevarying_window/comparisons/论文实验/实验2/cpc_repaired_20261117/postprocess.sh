#!/bin/bash
set -u
cd '/home/jack/wyh/时变窗口' || exit 1
E='convex_timevarying_window/comparisons/论文实验/实验2'
N="$E/cpc_repaired_20261117"
coordinator=$(cat "$N/COORDINATOR_PID")
exec 9>"$N/results/postprocessor.lock"
flock -n 9 || exit 2
while true; do
 changed=0
 for folder in "$N"/results/*; do
  if [ -f "$folder/result.json" ] && [ ! -f "$folder/assessment.json" ] && jq -e 'has("nominal_time_s")' "$folder/result.json" >/dev/null 2>&1; then
   level=$(basename "$folder")
   "$N/bin/cpc_audit" "$folder/result.json" "$E/robust_ipopt_full_20261117/source_inputs/assets/$level/scene.json" "$folder/assessment.json" || exit 3
   changed=1
  fi
 done
 if [ "$changed" = 1 ] || ! kill -0 "$coordinator" 2>/dev/null; then
  "$N/bin/cpc_report" "$E" || exit 4
  "$N/bin/cpc_diagnostics_report" "$N" "$E/robust_ipopt_full_20261117/source_inputs" || exit 5
 fi
 if ! kill -0 "$coordinator" 2>/dev/null; then
  sha256sum -c "$N/FROZEN_SHA256.txt" > "$N/FINAL_HASH_CHECK.log" || exit 6
  jq '{complete,cpc_completed,counts}' "$E/six_methods_cpc_repaired_20261117/SUMMARY.json" > "$N/POSTPROCESS_COMPLETION.json"
  exit 0
 fi
 sleep 40
done
