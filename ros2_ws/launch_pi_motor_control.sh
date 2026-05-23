#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$WS_DIR/install/setup.bash}"

BUILD_FIRST="${BUILD_FIRST:-false}"
SETUP_CAN="${SETUP_CAN:-false}"

HARDWARE_ENABLED="${HARDWARE_ENABLED:-true}"
NAV_CMD_TOPIC="${NAV_CMD_TOPIC:-cmd_vel}"
MANUAL_CMD_TOPIC="${MANUAL_CMD_TOPIC:-cmd_vel_manual}"
CAN_CHANNEL="${CAN_CHANNEL:-can0}"
CAN_INTERFACE="${CAN_INTERFACE:-socketcan}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
INIT_POS_PATH="${INIT_POS_PATH:-~/.ros/b_cubed_motor_init_pos.json}"
PARAMS_FILE="${PARAMS_FILE:-}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/log/pi_low_level_motor_control}"

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

build_low_level_runner() {
  cd "$WS_DIR"
  colcon build --packages-select low_level_runner
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
  build_low_level_runner
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
  "hardware_enabled:=$HARDWARE_ENABLED"
  "nav_cmd_topic:=$NAV_CMD_TOPIC"
  "manual_cmd_topic:=$MANUAL_CMD_TOPIC"
  "can_channel:=$CAN_CHANNEL"
  "can_interface:=$CAN_INTERFACE"
  "can_bitrate:=$CAN_BITRATE"
  "init_pos_path:=$INIT_POS_PATH"
)

if [[ -n "${PARAMS_FILE//[[:space:]]/}" ]]; then
  launch_args=("params_file:=$PARAMS_FILE" "${launch_args[@]}")
fi

exec ros2 launch low_level_runner pi_low_level_motor_control.launch.py \
  "${launch_args[@]}" \
  "$@"
