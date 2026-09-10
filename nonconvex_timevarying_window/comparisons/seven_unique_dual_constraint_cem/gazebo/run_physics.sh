#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
IMAGE='ghcr.io/j-rivero/gazebo:harmonic-full'
CONTAINER='seven_unique_track_gui'

cd "$ROOT"
env MPLCONFIGDIR=/tmp/seven_unique_mpl XDG_CACHE_HOME=/tmp/seven_unique_cache \
  conda run -n wyh python export_world.py
env MPLCONFIGDIR=/tmp/seven_unique_mpl XDG_CACHE_HOME=/tmp/seven_unique_cache \
  conda run -n wyh python validate_world.py >/dev/null

if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY is set.' >&2
  exit 1
fi
xhost +si:localuser:root >/dev/null
cleanup() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  xhost -si:localuser:root >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" --network host \
  --env DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$ROOT:/workspace:rw" --workdir /workspace --device /dev/dri \
  "$IMAGE" gz sim -r -v 3 seven_unique_physics.sdf >/dev/null
docker logs -f "$CONTAINER"
