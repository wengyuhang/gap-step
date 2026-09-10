#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
RUNTIME="$ROOT/.runtime"
QGC="$RUNTIME/QGroundControl-x86_64.AppImage"
QGC_URL='https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage'
VENV="$RUNTIME/teleop-venv"

mkdir -p "$RUNTIME"
if [[ ! -s "$QGC" ]]; then
  echo 'Downloading the official QGroundControl Linux AppImage...'
  curl --fail --location --retry 3 --output "$QGC.part" "$QGC_URL"
  mv "$QGC.part" "$QGC"
fi
chmod +x "$QGC"

if [[ ! -x "$VENV/bin/python" ]]; then
  python3 -m venv "$VENV"
fi
if ! "$VENV/bin/python" -c 'import pymavlink' >/dev/null 2>&1; then
  "$VENV/bin/pip" install --disable-pip-version-check \
    --index-url https://pypi.org/simple pymavlink
fi

echo "QGroundControl: $QGC"
echo "Keyboard teleop: $VENV/bin/python keyboard_teleop.py"
