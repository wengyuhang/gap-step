#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
IMAGE='px4io/px4-sitl-gazebo:latest'
CONTAINER='seven_unique_px4_manual'
WORLD='seven_unique_px4_manual.sdf'
SPAWN='-16,4,-6,0,0,0'

cd "$ROOT"
env MPLCONFIGDIR=/tmp/seven_unique_mpl XDG_CACHE_HOME=/tmp/seven_unique_cache \
  conda run -n wyh python export_world.py
env MPLCONFIGDIR=/tmp/seven_unique_mpl XDG_CACHE_HOME=/tmp/seven_unique_cache \
  conda run -n wyh python validate_course.py >/dev/null

if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY is set.' >&2
  exit 1
fi

xhost +si:localuser:root >/dev/null
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" --network host \
  --env DISPLAY \
  --env PX4_GZ_STANDALONE=1 \
  --env PX4_GZ_WORLD=seven_unique_px4_manual \
  --env PX4_GZ_MODEL_POSE="$SPAWN" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$ROOT:/workspace:ro" \
  --workdir /opt/px4-gazebo --device /dev/dri \
  --entrypoint bash "$IMAGE" -lc \
  "gz sim -s -r -v 3 /workspace/$WORLD >/tmp/seven_unique_gazebo.log 2>&1 & gz sim -g -v 3 >/tmp/seven_unique_gazebo_gui.log 2>&1 & exec /opt/px4-gazebo/bin/px4-gazebo -d ." \
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

echo 'PX4 x500 is running in the seven-window world.'
echo 'Mouse: enable QGroundControl Settings > General > Virtual joystick.'
echo 'Keyboard: run ./run_keyboard_teleop.sh in a terminal.'
echo 'Stop: ./stop_px4_manual.sh'
