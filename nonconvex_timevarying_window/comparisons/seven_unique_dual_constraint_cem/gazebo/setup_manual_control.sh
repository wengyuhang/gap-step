#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
RUNTIME="$ROOT/.runtime"
QGC="$RUNTIME/QGroundControl-x86_64.AppImage"
QGC_APPDIR="$RUNTIME/qgc-appdir"
QGC_URL='https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage'
VENV="$RUNTIME/teleop-venv"
QGC_CONFIG="$HOME/.config/QGroundControl/QGroundControl Daily.ini"

mkdir -p "$RUNTIME"
if [[ ! -s "$QGC" ]]; then
  echo 'Downloading the official QGroundControl Linux AppImage...'
  curl --fail --location --retry 3 --output "$QGC.part" "$QGC_URL"
  mv "$QGC.part" "$QGC"
fi
chmod +x "$QGC"
if [[ ! -x "$QGC_APPDIR/AppRun" ]]; then
  echo 'Extracting QGroundControl for reliable background launch...'
  rm -rf "$RUNTIME/squashfs-root" "$QGC_APPDIR"
  (cd "$RUNTIME" && "$QGC" --appimage-extract >/dev/null)
  mv "$RUNTIME/squashfs-root" "$QGC_APPDIR"
fi

if [[ ! -x "$VENV/bin/python" ]]; then
  python3 -m venv "$VENV"
fi
if ! "$VENV/bin/python" -c 'import pymavlink' >/dev/null 2>&1; then
  "$VENV/bin/pip" install --disable-pip-version-check \
    --index-url https://pypi.org/simple pymavlink
fi

# This setup is specifically for manual mouse control, so show QGC's two
# on-screen sticks on first launch as well as subsequent launches.
python3 - "$QGC_CONFIG" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
lines = path.read_text(encoding="utf-8").splitlines() if path.exists() else []
if "[App]" not in lines:
    if lines and lines[-1]:
        lines.append("")
    lines.extend(("[App]", "virtualJoystick=true",
                  "virtualJoystickAutoCenterThrottle=true"))
else:
    start = lines.index("[App]")
    end = next((i for i in range(start + 1, len(lines))
                if lines[i].startswith("[")), len(lines))
    group = lines[start + 1:end]
    for name, value in (("virtualJoystick", "true"),
                        ("virtualJoystickAutoCenterThrottle", "true")):
        match = next((i for i, line in enumerate(group)
                      if line.startswith(name + "=")), None)
        if match is None:
            group.append(f"{name}={value}")
        else:
            group[match] = f"{name}={value}"
    lines[start + 1:end] = group
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo "QGroundControl: $QGC"
echo "Keyboard teleop: $VENV/bin/python keyboard_teleop.py"
