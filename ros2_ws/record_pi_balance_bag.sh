#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$WS_DIR/install/setup.bash}"
SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"

BAG_DIR="${BAG_DIR:-$WS_DIR/bags/pi_balance_${SESSION_STAMP}}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
RAW_TILT_TOPIC="${RAW_TILT_TOPIC:-/sense_hat/raw}"
NAV_CMD_TOPIC="${NAV_CMD_TOPIC:-/cmd_vel}"
MANUAL_CMD_TOPIC="${MANUAL_CMD_TOPIC:-/cmd_vel_manual}"
BALANCED_CMD_TOPIC="${BALANCED_CMD_TOPIC:-/cmd_vel_balanced}"
MOTOR_POWERS_TOPIC="${MOTOR_POWERS_TOPIC:-/kiwi_drive/motor_powers}"
MOTOR_STATUS_TOPIC="${MOTOR_STATUS_TOPIC:-/kiwi_drive/status}"
BALANCE_STATUS_TOPIC="${BALANCE_STATUS_TOPIC:-/balance/status}"
ODOM_TOPIC="${ODOM_TOPIC:-}"

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

if [[ -f "$INSTALL_SETUP" ]]; then
  source_setup_file "$INSTALL_SETUP"
fi

topics=(
  "$IMU_TOPIC"
  "$RAW_TILT_TOPIC"
  "$NAV_CMD_TOPIC"
  "$MANUAL_CMD_TOPIC"
  "$BALANCED_CMD_TOPIC"
  "$MOTOR_POWERS_TOPIC"
  "$MOTOR_STATUS_TOPIC"
  "$BALANCE_STATUS_TOPIC"
)

if [[ -n "${ODOM_TOPIC//[[:space:]]/}" ]]; then
  topics+=("$ODOM_TOPIC")
fi

mkdir -p "$(dirname "$BAG_DIR")"
exec ros2 bag record -o "$BAG_DIR" "${topics[@]}" "$@"
