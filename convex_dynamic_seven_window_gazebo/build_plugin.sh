#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd)
IMAGE='convex-seven-gz-harmonic-dev:local'
docker build -q -f "$ROOT/Dockerfile.build" -t "$IMAGE" "$ROOT" >/dev/null
docker run --rm -v "$ROOT:/workspace:rw" -w /workspace "$IMAGE" \
  bash -lc 'cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2'
