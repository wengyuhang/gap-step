#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
QGC="$ROOT/.runtime/QGroundControl-x86_64.AppImage"
QGC_APP="$ROOT/.runtime/qgc-appdir/AppRun"
LOG="$ROOT/.runtime/qgroundcontrol.log"
PID_FILE="$ROOT/.runtime/qgroundcontrol.pid"

if [[ ! -x "$QGC" || ! -x "$QGC_APP" ]]; then
  echo "QGroundControl is not fully installed under $ROOT/.runtime" >&2
  echo "Run ./setup_manual_control.sh first." >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY is set.' >&2
  exit 1
fi
if pgrep -f "$ROOT/.runtime/qgc-appdir/usr/bin/QGroundControl" >/dev/null 2>&1; then
  echo 'QGroundControl is already running.'
  exit 0
fi

mkdir -p "$ROOT/.runtime"
nohup env DISPLAY="$DISPLAY" QT_QPA_PLATFORM=xcb \
  "$QGC_APP" >"$LOG" 2>&1 </dev/null &
echo $! >"$PID_FILE"
echo "QGroundControl started (log: $LOG)"
