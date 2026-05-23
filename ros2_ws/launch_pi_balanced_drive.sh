#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$WS_DIR/install/setup.bash}"

BUILD_FIRST="${BUILD_FIRST:-false}"
SETUP_CAN="${SETUP_CAN:-false}"

START_SENSE_HAT="${START_SENSE_HAT:-true}"
START_MOTOR_CONTROL="${START_MOTOR_CONTROL:-false}"
HARDWARE_ENABLED="${HARDWARE_ENABLED:-false}"
ALLOW_PD_FALLBACK="${ALLOW_PD_FALLBACK:-false}"

NAV_CMD_TOPIC="${NAV_CMD_TOPIC:-cmd_vel}"
MANUAL_CMD_TOPIC="${MANUAL_CMD_TOPIC:-cmd_vel_manual}"
BALANCED_CMD_TOPIC="${BALANCED_CMD_TOPIC:-cmd_vel_balanced}"
IMU_TOPIC="${IMU_TOPIC:-imu/data}"

CAN_CHANNEL="${CAN_CHANNEL:-can0}"
CAN_INTERFACE="${CAN_INTERFACE:-socketcan}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
POLICY_PATH="${POLICY_PATH:-}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/log/pi_balanced_drive}"

bool_is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

build_balance_stack() {
  cd "$WS_DIR"
  colcon build --packages-select bb8_balance_controller low_level_runner
}

setup_can_interface() {
  local sudo_cmd=()

  if (( EUID != 0 )); then
    sudo_cmd=(sudo)
  fi

  "${sudo_cmd[@]}" ip link set "$CAN_CHANNEL" down >/dev/null 2>&1 || true
  "${sudo_cmd[@]}" ip link set "$CAN_CHANNEL" type can bitrate "$CAN_BITRATE"
  "${sudo_cmd[@]}" ip link set "$CAN_CHANNEL" up
}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

source_setup_file "$ROS_SETUP"

if bool_is_true "$BUILD_FIRST" || [[ ! -f "$INSTALL_SETUP" ]]; then
  build_balance_stack
fi

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "Workspace setup file not found after build: $INSTALL_SETUP" >&2
  exit 1
fi

source_setup_file "$INSTALL_SETUP"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

if bool_is_true "$SETUP_CAN"; then
  setup_can_interface
fi

launch_args=(
  "start_sense_hat:=$START_SENSE_HAT"
  "start_motor_control:=$START_MOTOR_CONTROL"
  "hardware_enabled:=$HARDWARE_ENABLED"
  "allow_pd_fallback:=$ALLOW_PD_FALLBACK"
  "nav_cmd_topic:=$NAV_CMD_TOPIC"
  "manual_cmd_topic:=$MANUAL_CMD_TOPIC"
  "balanced_cmd_topic:=$BALANCED_CMD_TOPIC"
  "imu_topic:=$IMU_TOPIC"
  "can_channel:=$CAN_CHANNEL"
  "can_interface:=$CAN_INTERFACE"
  "can_bitrate:=$CAN_BITRATE"
)

if [[ -n "${POLICY_PATH//[[:space:]]/}" ]]; then
  launch_args=("policy_path:=$POLICY_PATH" "${launch_args[@]}")
fi

exec ros2 launch bb8_balance_controller pi_balanced_drive.launch.py \
  "${launch_args[@]}" \
  "$@"
