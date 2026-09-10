#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
PYTHON="$ROOT/.runtime/teleop-venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo 'Keyboard control dependencies are missing. Run ./setup_manual_control.sh first.' >&2
  exit 1
fi
exec "$PYTHON" "$ROOT/keyboard_teleop.py"
