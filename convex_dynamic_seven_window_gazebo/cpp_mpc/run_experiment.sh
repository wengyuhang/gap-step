#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
REFERENCE=${1:-$ROOT/trajectories/our_method_x500_accepted_100hz.npz}
FREQUENCY=${2:-100}
DURATION=${3:-4}
CONTAINER=convex_seven_togt_experiment
PARTITION=convex_seven_togt_experiment_partition
WORLD=convex_seven_dynamic_px4_togt_nmpc
RUN_DIR="$ROOT/results/px4_togt_cpp_mpc/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"

"$ROOT/run_px4_nmpc_headless.sh"
for setting in \
  'MPC_XY_VEL_MAX 25' 'MPC_XY_CRUISE 15' 'MPC_Z_VEL_MAX_UP 12' \
  'MPC_Z_VEL_MAX_DN 12' 'MPC_ACC_HOR 15' 'MPC_ACC_HOR_MAX 15' \
  'MPC_ACC_UP_MAX 12' 'MPC_ACC_DOWN_MAX 12' 'MPC_JERK_AUTO 50' \
  'MPC_JERK_MAX 50' 'MPC_TILTMAX_AIR 45' 'MPC_THR_HOVER .5'; do
  read -r name value <<<"$setting"
  docker exec "$CONTAINER" /opt/px4-gazebo/bin/px4-param set "$name" "$value" >/dev/null
done

conda run -n wyh python "$ROOT/cpp_mpc/export_assets.py" \
  --reference "$REFERENCE" --output "$ROOT/cpp_mpc/generated" >/dev/null
"$ROOT/cpp_mpc/build.sh" >/dev/null

shapes=(rectangle circle pentagon circle hexagon circle rectangle)
pids=()
for index in $(seq 1 7); do
  printf -v gate '%02d' "$index"
  topic="/world/$WORLD/model/gate_${gate}_W${index}_${shapes[$((index-1))]}/link/frame/sensor/frame_contact/contact"
  docker exec -e GZ_PARTITION="$PARTITION" "$CONTAINER" gz topic -e -t "$topic" \
    >"$RUN_DIR/gate_${gate}_contacts.pbtxt" 2>/dev/null &
  pids+=("$!")
done
cleanup() {
  for pid in "${pids[@]}"; do kill "$pid" >/dev/null 2>&1 || true; done
}
trap cleanup EXIT

export CASADIPATH="$ROOT/.runtime/togt_mpc_python/casadi"
"$ROOT/cpp_mpc/controller" "$ROOT/cpp_mpc/generated/reference.csv" \
  "$ROOT/cpp_mpc/generated/solver.casadi" "$ROOT/cpp_mpc/generated/rollout.casadi" \
  "$RUN_DIR/telemetry.csv" "$FREQUENCY" "$DURATION" | tee "$RUN_DIR/controller.txt"
cleanup
trap - EXIT
conda run --no-capture-output -n wyh python "$ROOT/cpp_mpc/summarize.py" "$RUN_DIR"
echo "$RUN_DIR"
