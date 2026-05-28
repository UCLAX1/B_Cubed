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
ROLL_OFFSET_RAD="${ROLL_OFFSET_RAD:-0.0}"
PITCH_OFFSET_RAD="${PITCH_OFFSET_RAD:-0.0}"

CAN_CHANNEL="${CAN_CHANNEL:-can0}"
CAN_INTERFACE="${CAN_INTERFACE:-socketcan}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
POLICY_PATH="${POLICY_PATH:-}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/log/pi_balanced_drive}"
STATUS_INTERVAL_SEC="${STATUS_INTERVAL_SEC:-10}"
COMMAND_MONITOR_ENABLED="${COMMAND_MONITOR_ENABLED:-true}"
COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC="${COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC:-2}"
COMMAND_MONITOR_TOPICS="${COMMAND_MONITOR_TOPICS:-$MANUAL_CMD_TOPIC $NAV_CMD_TOPIC $BALANCED_CMD_TOPIC /jet_cmd /kiwi_drive/motor_powers /balance/status /kiwi_drive/status}"
launch_pid=""

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

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" >/dev/null 2>&1; then
    echo "Shutting down Pi balance launch..."
    kill -INT "$launch_pid" >/dev/null 2>&1 || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      if ! kill -0 "$launch_pid" >/dev/null 2>&1; then
        break
      fi
      sleep 1
    done
    if kill -0 "$launch_pid" >/dev/null 2>&1; then
      echo "Escalating Pi balance launch shutdown..."
      kill -TERM "$launch_pid" >/dev/null 2>&1 || true
    fi
  fi
  wait >/dev/null 2>&1 || true
}

topic_visible() {
  local topic="$1"

  ros2 topic list 2>/dev/null | grep -Fxq "$topic"
}

print_command_topic_info() {
  local topic
  local info

  if ! bool_is_true "$COMMAND_MONITOR_ENABLED"; then
    return 0
  fi

  echo "Pi command topic endpoints:"
  for topic in $COMMAND_MONITOR_TOPICS; do
    if ! topic_visible "$topic"; then
      echo "  $topic: not visible"
      continue
    fi
    info="$(ros2 topic info "$topic" 2>/dev/null | tr '\n' '; ' || true)"
    echo "  $topic: $info"
  done
}

print_command_samples() {
  local topic
  local sample

  if ! bool_is_true "$COMMAND_MONITOR_ENABLED"; then
    return 0
  fi

  echo "Pi command monitor:"
  for topic in $COMMAND_MONITOR_TOPICS; do
    if ! topic_visible "$topic"; then
      echo "  $topic: not visible"
      continue
    fi

    sample="$(
      timeout "$COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC" \
        ros2 topic echo "$topic" --once 2>/dev/null || true
    )"
    if [[ -n "${sample//[[:space:]]/}" ]]; then
      echo "  $topic:"
      sed 's/^/    /' <<<"$sample" | head -20
    else
      echo "  $topic: visible, no sample within ${COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC}s"
    fi
  done
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
  "roll_offset_rad:=$ROLL_OFFSET_RAD"
  "pitch_offset_rad:=$PITCH_OFFSET_RAD"
  "can_channel:=$CAN_CHANNEL"
  "can_interface:=$CAN_INTERFACE"
  "can_bitrate:=$CAN_BITRATE"
)

if [[ -n "${POLICY_PATH//[[:space:]]/}" ]]; then
  launch_args=("policy_path:=$POLICY_PATH" "${launch_args[@]}")
fi

trap cleanup EXIT INT TERM

echo "B_Cubed Pi balanced drive launch"
echo "  ros_domain_id=${ROS_DOMAIN_ID:-default}"
echo "  rmw=${RMW_IMPLEMENTATION:-default}"
echo "  cyclonedds_uri=${CYCLONEDDS_URI:-default}"
echo "  start_sense_hat=$START_SENSE_HAT"
echo "  start_motor_control=$START_MOTOR_CONTROL"
echo "  hardware_enabled=$HARDWARE_ENABLED"
echo "  nav_cmd_topic=$NAV_CMD_TOPIC"
echo "  manual_cmd_topic=$MANUAL_CMD_TOPIC"
echo "  balanced_cmd_topic=$BALANCED_CMD_TOPIC"
echo "  can_channel=$CAN_CHANNEL"
echo "  can_interface=$CAN_INTERFACE"
echo "  can_bitrate=$CAN_BITRATE"
echo "  command_monitor_enabled=$COMMAND_MONITOR_ENABLED"
echo "  command_monitor_topics=$COMMAND_MONITOR_TOPICS"

ros2 launch bb8_balance_controller pi_balanced_drive.launch.py \
  "${launch_args[@]}" \
  "$@" &
launch_pid="$!"
echo "Pi balance launch process started: pid=$launch_pid"

next_status_report=$SECONDS
while :; do
  if ! kill -0 "$launch_pid" >/dev/null 2>&1; then
    if wait "$launch_pid"; then
      status=0
    else
      status=$?
    fi
    echo "Pi balance launch exited with status $status." >&2
    exit "$status"
  fi

  if (( SECONDS >= next_status_report )); then
    print_command_topic_info
    print_command_samples
    next_status_report=$((SECONDS + STATUS_INTERVAL_SEC))
  fi

  sleep 2
done
