#!/usr/bin/env bash
# Same world, vehicle and window clock as run_px4_nmpc_headless.sh, plus a
# Gazebo GUI client so the two NMPC-tracked flights can be watched live. The
# GUI is a separate process and does not change the 4 ms physics step, real-time
# factor 1.0, or the 100 Hz control period.
set -euo pipefail
ROOT=$(LVL=$(dirname "$0"); cd "$LVL" && pwd)
CONTAINER=convex_seven_togt_experiment
PARTITION=convex_seven_togt_experiment_partition
WORLD=convex_seven_dynamic_px4_togt_nmpc

if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY is set.' >&2
  exit 1
fi
if [[ ! -S "/tmp/.X11-unix/X${DISPLAY#*:}" ]]; then
  echo "DISPLAY=$DISPLAY has no matching X socket in /tmp/.X11-unix." >&2
  exit 1
fi

cd "$ROOT"
conda run -n wyh python export_world.py >/dev/null
xhost +si:localuser:root >/dev/null
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" --network host \
  --env DISPLAY --env GZ_PARTITION="$PARTITION" \
  --env GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build \
  --env PX4_GZ_STANDALONE=1 --env PX4_GZ_WORLD="$WORLD" \
  --env PX4_GZ_MODEL_POSE='-16,4,-6,0,0,0' \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$ROOT:/workspace:ro" --workdir /opt/px4-gazebo \
  --device /dev/dri \
  --entrypoint bash px4io/px4-sitl-gazebo:latest -lc \
  "gz sim -s -r -v 2 /workspace/$WORLD.sdf >/tmp/gazebo.log 2>&1 & gz sim -g -v 3 >/tmp/gazebo_gui.log 2>&1 & exec /opt/px4-gazebo/bin/px4-gazebo -d ." >/dev/null
for _ in $(seq 1 40); do
  if docker exec "$CONTAINER" /opt/px4-gazebo/bin/px4-commander status >/dev/null 2>&1; then sleep 2; echo "$CONTAINER ready with GUI on DISPLAY=$DISPLAY"; exit 0; fi
  sleep .5
done
docker logs --tail 100 "$CONTAINER" >&2
exit 1
