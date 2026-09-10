#!/usr/bin/env bash
set -euo pipefail
docker rm -f convex_seven_dynamic_px4 >/dev/null 2>&1 || true
xhost -si:localuser:root >/dev/null 2>&1 || true
echo 'PX4/Gazebo simulation stopped.'

