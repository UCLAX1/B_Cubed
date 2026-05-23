#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$WS_DIR/install/setup.bash}"
BUILD_FIRST="${BUILD_FIRST:-false}"
IMU_TOPIC="${IMU_TOPIC:-imu/data}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/log/pi_imu_only}"

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

source_setup_file "$ROS_SETUP"

if [[ "$BUILD_FIRST" == "true" || ! -f "$INSTALL_SETUP" ]]; then
  cd "$WS_DIR"
  colcon build --packages-select bb8_balance_controller
fi

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "Workspace setup file not found after build: $INSTALL_SETUP" >&2
  exit 1
fi

source_setup_file "$INSTALL_SETUP"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

exec ros2 launch bb8_balance_controller sense_hat_imu.launch.py \
  "imu_topic:=$IMU_TOPIC" \
  "$@"
