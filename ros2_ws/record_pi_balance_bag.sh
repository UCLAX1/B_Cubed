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
TOPIC_WAIT_TIMEOUT_SEC="${TOPIC_WAIT_TIMEOUT_SEC:-30}"
REQUIRED_TOPICS="${REQUIRED_TOPICS:-$IMU_TOPIC $BALANCE_STATUS_TOPIC}"

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

wait_for_topic_message() {
  local topic="$1"

  echo "Waiting for one message on $topic..."
  if ! timeout "${TOPIC_WAIT_TIMEOUT_SEC}s" \
    ros2 topic echo --once "$topic" >/dev/null 2>&1; then
    echo "Timed out waiting for $topic." >&2
    echo "Start ./launch_pi_balanced_drive.sh first, then run this recorder." >&2
    exit 1
  fi
}

for required_topic in $REQUIRED_TOPICS; do
  wait_for_topic_message "$required_topic"
done

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
