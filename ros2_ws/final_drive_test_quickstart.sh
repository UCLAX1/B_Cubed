#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$SCRIPT_DIR/install/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

JETSON_HOST="${JETSON_HOST:-ubuntu}"
PI_HOST="${PI_HOST:-ubuntu-1}"
JETSON_ADDR="${JETSON_ADDR:-$JETSON_HOST}"
PI_ADDR="${PI_ADDR:-$PI_HOST}"
SSH_CONNECT_TIMEOUT="${SSH_CONNECT_TIMEOUT:-5}"
TOPIC_WAIT_TIMEOUT_SEC="${TOPIC_WAIT_TIMEOUT_SEC:-8}"

required_topics=(
  "/zed/zed_node/pose"
  "/scan"
  "/cmd_vel_balanced"
  "/balance/status"
  "/imu/data"
)

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

check_ping() {
  local label="$1"
  local host="$2"

  printf 'Checking %s reachability (%s)... ' "$label" "$host"
  if ping -c 1 -W 2 "$host" >/dev/null 2>&1; then
    echo "ok"
  else
    echo "failed"
    return 1
  fi
}

check_ssh() {
  local label="$1"
  local host="$2"

  printf 'Checking %s SSH (%s)... ' "$label" "$host"
  if ssh -o BatchMode=yes -o ConnectTimeout="$SSH_CONNECT_TIMEOUT" "$host" 'true' >/dev/null 2>&1; then
    echo "ok"
  else
    echo "failed"
    return 1
  fi
}

check_remote_service() {
  local label="$1"
  local host="$2"
  local service="$3"

  printf 'Checking %s service %s... ' "$label" "$service"
  if ssh -o BatchMode=yes -o ConnectTimeout="$SSH_CONNECT_TIMEOUT" "$host" \
    "systemctl is-active --quiet '$service'" >/dev/null 2>&1; then
    echo "active"
  else
    echo "not active"
    return 1
  fi
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + TOPIC_WAIT_TIMEOUT_SEC))

  printf 'Checking ROS topic %s... ' "$topic"
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "visible"
      return 0
    fi
    sleep 1
  done
  echo "missing"
  return 1
}

check_balance_status() {
  local status

  printf 'Checking RL balance status... '
  status="$(timeout "$TOPIC_WAIT_TIMEOUT_SEC" ros2 topic echo --once /balance/status std_msgs/msg/String 2>/dev/null || true)"
  if grep -Eq 'backend=(npz|onnx)' <<<"$status"; then
    echo "policy active"
    grep -Eo 'safe=[^,]+|reason=[^,]+|backend=[^,]+' <<<"$status" | paste -sd ' ' -
    return 0
  fi

  echo "policy not confirmed"
  [[ -n "$status" ]] && echo "$status"
  return 1
}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

export ROS_DOMAIN_ID
source_setup_file "$ROS_SETUP"
if [[ -f "$INSTALL_SETUP" ]]; then
  source_setup_file "$INSTALL_SETUP"
fi

failures=0

check_ping "Jetson" "$JETSON_ADDR" || ((failures++))
check_ping "Pi" "$PI_ADDR" || ((failures++))
check_ssh "Jetson" "$JETSON_HOST" || ((failures++))
check_ssh "Pi" "$PI_HOST" || ((failures++))
check_remote_service "Jetson" "$JETSON_HOST" "b_cubed_jetson.service" || ((failures++))
check_remote_service "Pi" "$PI_HOST" "b_cubed_pi.service" || ((failures++))

for topic in "${required_topics[@]}"; do
  wait_for_topic "$topic" || ((failures++))
done

check_balance_status || ((failures++))

if (( failures > 0 )); then
  echo
  echo "Quickstart checks failed ($failures issue(s)). Keep HARDWARE_ENABLED=false."
  exit 1
fi

cat <<EOF

Quickstart checks passed.

Hardware is still gated off by default. To run the drivetrain after the robot is
physically supported and the operator is ready:

  sudo systemctl edit b_cubed_pi.service

Set:

  [Service]
  Environment=HARDWARE_ENABLED=true

Then:

  sudo systemctl restart b_cubed_pi.service
EOF
