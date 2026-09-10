#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
IMAGE='ghcr.io/j-rivero/gazebo:harmonic-full'
CONTAINER='convex_seven_dynamic_gui'
PARTITION='convex_seven_dynamic_gui_partition'
MODE=${1:-gui}

cd "$ROOT"
conda run -n wyh python export_world.py
./build_plugin.sh >/dev/null
conda run -n wyh python validate_assets.py >/dev/null

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
cleanup() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  if [[ "$MODE" == "gui" ]]; then
    xhost -si:localuser:root >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$MODE" == "headless" ]]; then
  exec docker run --rm --name "$CONTAINER" --network host \
    -e GZ_PARTITION="$PARTITION" \
    -e GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build \
    -v "$ROOT:/workspace:ro" -w /workspace \
    "$IMAGE" gz sim -s -r -v 3 convex_seven_dynamic_physics.sdf
fi
if [[ "$MODE" != "gui" ]]; then
  echo 'usage: ./run_track.sh [gui|headless]' >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY is set.' >&2
  exit 1
fi
xhost +si:localuser:root >/dev/null
docker run --name "$CONTAINER" --network host \
  -e DISPLAY -e GZ_PARTITION="$PARTITION" \
  -e GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$ROOT:/workspace:ro" -w /workspace --device /dev/dri \
  "$IMAGE" gz sim -r -v 3 convex_seven_dynamic_physics.sdf

