#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
IMAGE='px4io/px4-sitl-gazebo:latest'
CONTAINER='convex_seven_dynamic_px4'
WORLD='convex_seven_dynamic_px4.sdf'
SPAWN='-16,4,-6,0,0,0'
PARTITION='convex_seven_dynamic_px4_partition'

cd "$ROOT"
conda run -n wyh python export_world.py
./build_plugin.sh >/dev/null
conda run -n wyh python validate_assets.py >/dev/null

if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY is set.' >&2
  exit 1
fi
xhost +si:localuser:root >/dev/null
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" --network host \
  --env DISPLAY --env GZ_PARTITION="$PARTITION" \
  --env GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build \
  --env PX4_GZ_STANDALONE=1 \
  --env PX4_GZ_WORLD=convex_seven_dynamic_px4 \
  --env PX4_GZ_MODEL_POSE="$SPAWN" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$ROOT:/workspace:ro" \
  --workdir /opt/px4-gazebo --device /dev/dri \
  --entrypoint bash "$IMAGE" -lc \
  "gz sim -s -r -v 3 /workspace/$WORLD >/tmp/convex_seven_gazebo.log 2>&1 & gz sim -g -v 3 >/tmp/convex_seven_gui.log 2>&1 & exec /opt/px4-gazebo/bin/px4-gazebo -d ." \
  >/dev/null

for _ in $(seq 1 60); do
  if docker exec "$CONTAINER" /opt/px4-gazebo/bin/px4-commander status >/dev/null 2>&1; then
    break
  fi
  if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
    echo 'PX4 container exited during startup:' >&2
    docker logs "$CONTAINER" >&2
    exit 1
  fi
  sleep 1
done
if ! docker exec "$CONTAINER" /opt/px4-gazebo/bin/px4-commander status >/dev/null 2>&1; then
  echo 'PX4 did not finish startup within 60 seconds.' >&2
  docker logs --tail 120 "$CONTAINER" >&2
  exit 1
fi

if [[ -x "$ROOT/.runtime/QGroundControl-x86_64.AppImage" ]]; then
  "$ROOT/run_qgroundcontrol.sh"
else
  echo 'QGroundControl is not installed; run ./setup_manual_control.sh.'
fi
echo 'PX4 x500 is running in the convex dynamic seven-window world.'
echo 'Keyboard control: ./run_keyboard_teleop.sh'
echo 'Stop: ./stop_px4_manual.sh'

