#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$WS_DIR/install/setup.bash}"

BUILD_FIRST="${BUILD_FIRST:-false}"
IMU_TOPIC="${IMU_TOPIC:-imu/data}"
IMU_STARTUP_TIMEOUT_SEC="${IMU_STARTUP_TIMEOUT_SEC:-30}"

START_MOTOR_CONTROL="${START_MOTOR_CONTROL:-false}"
HARDWARE_ENABLED="${HARDWARE_ENABLED:-false}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/log/pi_balance_test}"

imu_pid=""

cleanup() {
  if [[ -n "$imu_pid" ]] && kill -0 "$imu_pid" >/dev/null 2>&1; then
    kill "$imu_pid" >/dev/null 2>&1 || true
    wait "$imu_pid" >/dev/null 2>&1 || true
  fi
}

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

build_test_stack() {
  cd "$WS_DIR"
  colcon build --packages-select bb8_balance_controller low_level_runner
}

wait_for_imu() {
  local topic="/${IMU_TOPIC#/}"
  local deadline=$((SECONDS + IMU_STARTUP_TIMEOUT_SEC))
  local topic_type=""

  echo "Waiting for $topic to be discovered..."
  while (( SECONDS < deadline )); do
    if [[ -n "$imu_pid" ]] && ! kill -0 "$imu_pid" >/dev/null 2>&1; then
      echo "IMU launcher exited before publishing $topic." >&2
      return 1
    fi

    topic_type="$(ros2 topic type "$topic" 2>/dev/null || true)"
    if [[ -n "${topic_type//[[:space:]]/}" ]]; then
      echo "Discovered $topic as $topic_type."
      echo "Waiting for one message on $topic..."
      timeout "${IMU_STARTUP_TIMEOUT_SEC}s" \
        ros2 topic echo --once "$topic" >/dev/null
      return $?
    fi
    sleep 1
  done

  echo "Timed out waiting for $topic discovery." >&2
  return 1
}

trap cleanup EXIT INT TERM

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

source_setup_file "$ROS_SETUP"

if [[ "$BUILD_FIRST" == "true" || ! -f "$INSTALL_SETUP" ]]; then
  build_test_stack
fi

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "Workspace setup file not found after build: $INSTALL_SETUP" >&2
  exit 1
fi

source_setup_file "$INSTALL_SETUP"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

echo "Starting Sense HAT IMU publisher..."
BUILD_FIRST=false IMU_TOPIC="$IMU_TOPIC" ROS_LOG_DIR="$ROS_LOG_DIR/imu" \
  "$WS_DIR/launch_pi_imu_only.sh" &
imu_pid="$!"

if ! wait_for_imu; then
  echo "Timed out waiting for $IMU_TOPIC; stopping test launch." >&2
  exit 1
fi

echo "Starting balance stack with external IMU publisher."
BUILD_FIRST=false \
START_SENSE_HAT=false \
START_MOTOR_CONTROL="$START_MOTOR_CONTROL" \
HARDWARE_ENABLED="$HARDWARE_ENABLED" \
IMU_TOPIC="$IMU_TOPIC" \
ROS_LOG_DIR="$ROS_LOG_DIR/balance" \
  "$WS_DIR/launch_pi_balanced_drive.sh" \
  "$@"
