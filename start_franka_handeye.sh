#!/usr/bin/env bash
set -e
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Usage: TELEOP_REPO=/path/to/gello_upper_body_teleop ./start_franka_handeye.sh
# Existing services own the hardware. This launcher never starts or enables them.
if [[ -n "${TELEOP_REPO:-}" ]]; then
  if [[ ! -d "$TELEOP_REPO" ]]; then
    echo "TELEOP_REPO does not exist: $TELEOP_REPO" >&2; exit 1
  fi
  if [[ -z "${ROS_DOMAIN_ID:-}" && -f "$TELEOP_REPO/docker/.env" ]]; then
    HAND_EYE_DOMAIN="$(sed -n 's/^TELEOP_ROS_DOMAIN_ID=\([0-9]*\).*$/\1/p' "$TELEOP_REPO/docker/.env" | head -1)"
    export ROS_DOMAIN_ID="${HAND_EYE_DOMAIN:-0}"
  fi
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for setup_file in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -f "$setup_file" ]]; then source "$setup_file"; break; fi
  done
fi
if [[ -n "${TELEOP_REPO:-}" && -f "$TELEOP_REPO/ros_ws/install/setup.bash" ]]; then
  source "$TELEOP_REPO/ros_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
cd "$ROOT_DIR"
exec "${HANDEYE_PYTHON:-python3}" -m handeye_web.server "$@"
