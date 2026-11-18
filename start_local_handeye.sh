#!/usr/bin/env bash
# Local Docker launcher; reuses the installed teleop ROS/DDS configuration.
set -euo pipefail
export HANDEYE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export TELEOP_REPO="${TELEOP_REPO:-/home/descfly/ZH/gello_upper_body_teleop}"
export HANDEYE_UID="$(id -u)"
export HANDEYE_GID="$(id -g)"
if [[ ! -f "$TELEOP_REPO/docker/compose.yaml" ]]; then
  echo "Set TELEOP_REPO to the installed teleop repository." >&2
  exit 1
fi
cd "$TELEOP_REPO/docker"
exec docker compose --profile tools \
  -f compose.yaml -f "$HANDEYE_DIR/docker/compose.handeye.yaml" \
  run --rm --no-deps tools -lc \
  'source /opt/ros/humble/setup.bash &&
   cmake -S native -B native/build-docker &&
   cmake --build native/build-docker -j2 &&
   exec ./start_franka_handeye.sh "$@"' \
  handeye --port "${HANDEYE_PORT:-8766}" "$@"
