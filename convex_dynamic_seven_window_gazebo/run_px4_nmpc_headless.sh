#!/usr/bin/env bash
set -euo pipefail
ROOT=$(LVL=$(dirname "$0"); cd "$LVL" && pwd)
CONTAINER=convex_seven_togt_experiment
PARTITION=convex_seven_togt_experiment_partition
WORLD=convex_seven_dynamic_px4_togt_nmpc
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" --network host \
  --env GZ_PARTITION="$PARTITION" --env GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build \
  --env PX4_GZ_STANDALONE=1 --env PX4_GZ_WORLD="$WORLD" \
  --env PX4_GZ_MODEL_POSE='-16,4,-6,0,0,0' \
  --volume "$ROOT:/workspace:ro" --workdir /opt/px4-gazebo \
  --entrypoint bash px4io/px4-sitl-gazebo:latest -lc \
  "gz sim -s -r -v 2 /workspace/$WORLD.sdf >/tmp/gazebo.log 2>&1 & exec /opt/px4-gazebo/bin/px4-gazebo -d ." >/dev/null
for _ in $(seq 1 40); do
  if docker exec "$CONTAINER" /opt/px4-gazebo/bin/px4-commander status >/dev/null 2>&1; then sleep 2; echo "$CONTAINER ready"; exit 0; fi
  sleep .5
done
docker logs --tail 100 "$CONTAINER" >&2
exit 1
