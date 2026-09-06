#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd)
IMAGE='ghcr.io/j-rivero/gazebo:harmonic-full'
docker rm -f sc_sip_gz_server >/dev/null 2>&1 || true
docker run -d --name sc_sip_gz_server --network host -v "$ROOT:/workspace:ro" -w /workspace \
  "$IMAGE" gz sim -s -r wide_scrambled_fast_closed_loop.sdf
trap 'docker rm -f sc_sip_gz_server >/dev/null 2>&1 || true' EXIT
python3 "$ROOT/motion_bridge.py" &
BRIDGE=$!
trap 'kill "$BRIDGE" 2>/dev/null || true; docker rm -f sc_sip_gz_server >/dev/null 2>&1 || true' EXIT INT TERM
gz-harmonic -g
