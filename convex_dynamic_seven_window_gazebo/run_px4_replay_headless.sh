#!/usr/bin/env bash
set -euo pipefail

ROOT=$(LVL=$(dirname "$0"); cd "$LVL" && pwd)
REFERENCE=${1:?usage: run_px4_replay_headless.sh /absolute/path/to/reference.csv}
case "$REFERENCE" in "$ROOT"/*) ;; *) echo "reference must be inside $ROOT" >&2; exit 2;; esac
CONTAINER=convex_seven_replay
PARTITION=convex_seven_replay_partition
WORLD=convex_seven_dynamic_px4_replay
CONTAINER_REFERENCE=/workspace/${REFERENCE#"$ROOT"/}

"$ROOT/build_plugin.sh" >/dev/null

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" --network host \
  --env GZ_PARTITION="$PARTITION" --env GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build \
  --env REPLAY_REFERENCE="$CONTAINER_REFERENCE" \
  --env GZ_SIM_RESOURCE_PATH=/opt/px4-gazebo/share/gz/models \
  --volume "$ROOT:/workspace:ro" --workdir /workspace \
  --entrypoint bash px4io/px4-sitl-gazebo:latest -lc \
  "exec gz sim -s -r -v 2 /workspace/$WORLD.sdf >/tmp/gazebo.log 2>&1" >/dev/null
for _ in $(seq 1 40); do
  if docker exec -e GZ_PARTITION="$PARTITION" "$CONTAINER" gz model --list 2>/dev/null | grep -q -- '- x500_0'; then
    echo "$CONTAINER ready"
    exit 0
  fi
  sleep .5
done
docker logs --tail 100 "$CONTAINER" >&2
exit 1
