#!/usr/bin/env bash
set -euo pipefail

CONTAINER='seven_unique_px4_manual'
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
xhost -si:localuser:root >/dev/null 2>&1 || true
echo 'PX4/Gazebo simulation stopped.'
