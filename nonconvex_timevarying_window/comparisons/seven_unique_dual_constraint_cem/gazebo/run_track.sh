#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")" && pwd)
MODE=${1:-physics}

if [[ "$MODE" == "physics" ]]; then
  exec "$ROOT/run_physics.sh"
fi

cd "$ROOT"
env MPLCONFIGDIR=/tmp/seven_unique_mpl XDG_CACHE_HOME=/tmp/seven_unique_cache \
  conda run -n wyh python export_world.py --with-replay
env MPLCONFIGDIR=/tmp/seven_unique_mpl XDG_CACHE_HOME=/tmp/seven_unique_cache \
  conda run -n wyh python validate_world.py >/dev/null

case "$MODE" in
  gui)
    exec gz-harmonic -r -v 3 seven_unique_race_preview.sdf
    ;;
  exact-gui)
    exec gz-harmonic -r -v 3 seven_unique_high_fidelity.sdf
    ;;
  headless)
    exec gz-harmonic --headless -r -v 3 seven_unique_high_fidelity.sdf
    ;;
  *)
    echo "usage: $0 [gui|exact-gui|physics|headless]" >&2
    exit 2
    ;;
esac
