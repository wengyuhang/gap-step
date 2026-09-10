#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
QGC="$ROOT/.runtime/QGroundControl-x86_64.AppImage"
LOG="$ROOT/.runtime/qgroundcontrol.log"

if [[ ! -x "$QGC" ]]; then
  echo "QGroundControl is not installed at $QGC" >&2
  echo "Run ./setup_manual_control.sh first." >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY is set.' >&2
  exit 1
fi
if pgrep -f "$QGC" >/dev/null 2>&1; then
  echo 'QGroundControl is already running.'
  exit 0
fi

mkdir -p "$ROOT/.runtime"
nohup env APPIMAGE_EXTRACT_AND_RUN=1 "$QGC" >"$LOG" 2>&1 </dev/null &
echo "QGroundControl started (log: $LOG)"
