#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$WS_DIR/install/setup.bash}"

BUILD_FIRST="${BUILD_FIRST:-false}"
SETUP_CAN="${SETUP_CAN:-${CAN_SETUP:-false}}"
CAN_SETUP="${CAN_SETUP:-$SETUP_CAN}"

START_SENSE_HAT="${START_SENSE_HAT:-true}"
START_BALANCE_CONTROLLER="${START_BALANCE_CONTROLLER:-true}"
START_MOTOR_CONTROL="${START_MOTOR_CONTROL:-false}"
HARDWARE_ENABLED="${HARDWARE_ENABLED:-false}"
MOSFET_POWER_ENABLED="${MOSFET_POWER_ENABLED:-$HARDWARE_ENABLED}"
MOSFET_GPIO_PIN="${MOSFET_GPIO_PIN:-16}"
ALLOW_PD_FALLBACK="${ALLOW_PD_FALLBACK:-false}"
ALLOW_MISSING_IMU="${ALLOW_MISSING_IMU:-false}"
SENSE_HAT_DETACH_KERNEL="${SENSE_HAT_DETACH_KERNEL:-true}"
SENSE_HAT_I2C_DEVICES="${SENSE_HAT_I2C_DEVICES:-1-001c 1-005c 1-006a}"

NAV_CMD_TOPIC="${NAV_CMD_TOPIC:-cmd_vel}"
MANUAL_CMD_TOPIC="${MANUAL_CMD_TOPIC:-cmd_vel_manual}"
BALANCED_CMD_TOPIC="${BALANCED_CMD_TOPIC:-cmd_vel_balanced}"
IMU_TOPIC="${IMU_TOPIC:-imu/data}"
ROLL_OFFSET_RAD="${ROLL_OFFSET_RAD:-0.0}"
PITCH_OFFSET_RAD="${PITCH_OFFSET_RAD:-0.0}"
BALANCE_NAV_TIMEOUT_SEC="${BALANCE_NAV_TIMEOUT_SEC:-0.50}"
BALANCE_MANUAL_TIMEOUT_SEC="${BALANCE_MANUAL_TIMEOUT_SEC:-0.30}"
MOTOR_NAV_CMD_TIMEOUT_SEC="${MOTOR_NAV_CMD_TIMEOUT_SEC:-0.50}"
MOTOR_MANUAL_CMD_TIMEOUT_SEC="${MOTOR_MANUAL_CMD_TIMEOUT_SEC:-0.30}"
MOTOR_IPC_WATCHDOG_TIMEOUT_SEC="${MOTOR_IPC_WATCHDOG_TIMEOUT_SEC:-0.30}"
MOTOR_POWER_SLEW_RATE_PER_SEC="${MOTOR_POWER_SLEW_RATE_PER_SEC:-4.0}"
MOTOR_POWER_DEADBAND="${MOTOR_POWER_DEADBAND:-0.04}"
MOTOR_MIN_POWER="${MOTOR_MIN_POWER:-0.0}"

CAN_CHANNEL="${CAN_CHANNEL:-can0}"
CAN_INTERFACE="${CAN_INTERFACE:-socketcan}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
CAN_DEVICE="${CAN_DEVICE:-/dev/ttyACM0}"
SLCAND_SPEED="${SLCAND_SPEED:-s8}"
POLICY_PATH="${POLICY_PATH:-}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/log/pi_balanced_drive}"
STATUS_INTERVAL_SEC="${STATUS_INTERVAL_SEC:-10}"
COMMAND_MONITOR_ENABLED="${COMMAND_MONITOR_ENABLED:-true}"
COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC="${COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC:-2}"
COMMAND_MONITOR_SHOW_MISSING="${COMMAND_MONITOR_SHOW_MISSING:-summary}"
COMMAND_MONITOR_TOPICS="${COMMAND_MONITOR_TOPICS:-$MANUAL_CMD_TOPIC $NAV_CMD_TOPIC $IMU_TOPIC}"
NETWORK_MODE="${NETWORK_MODE:-auto}"
PI_TAILSCALE_IP="${PI_TAILSCALE_IP:-100.80.7.37}"
PI_HOTSPOT_IP="${PI_HOTSPOT_IP:-10.42.0.166}"
JETSON_TAILSCALE_IP="${JETSON_TAILSCALE_IP:-100.86.7.33}"
JETSON_HOTSPOT_IP="${JETSON_HOTSPOT_IP:-10.42.0.1}"
CYCLONEDDS_AUTOCONFIG="${CYCLONEDDS_AUTOCONFIG:-true}"
CYCLONEDDS_CONFIG_PATH="${CYCLONEDDS_CONFIG_PATH:-$HOME/cyclonedds.xml}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
launch_pid=""
mosfet_guard_pid=""
dds_network_mode=""
dds_local_address=""
dds_peer_addresses=()
ros_launch_extra_args=()

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

  if ! ip link show "$CAN_CHANNEL" >/dev/null 2>&1; then
    if [[ ! -e "$CAN_DEVICE" ]]; then
      echo "CAN device not found: $CAN_DEVICE" >&2
      exit 1
    fi
    echo "Creating $CAN_CHANNEL from $CAN_DEVICE with slcand..."
    "${sudo_cmd[@]}" slcand -o "-$SLCAND_SPEED" "$CAN_DEVICE" "$CAN_CHANNEL"
    sleep 0.5
  fi

  if ! ip link show "$CAN_CHANNEL" >/dev/null 2>&1; then
    echo "CAN interface was not created: $CAN_CHANNEL" >&2
    exit 1
  fi

  "${sudo_cmd[@]}" ip link set "$CAN_CHANNEL" up
  ip -details link show "$CAN_CHANNEL"
}

start_mosfet_power_guard() {
  if ! bool_is_true "$MOSFET_POWER_ENABLED"; then
    return 0
  fi

  echo "Turning MOSFET ON (BCM GPIO $MOSFET_GPIO_PIN)"
  python3 -c '
import signal
import sys
from time import sleep

from gpiozero import DigitalOutputDevice

pin = int(sys.argv[1])
mosfet = DigitalOutputDevice(pin)

def stop(_signum=None, _frame=None):
    print(f"Turning MOSFET OFF (BCM GPIO {pin})", flush=True)
    mosfet.off()
    mosfet.close()
    raise SystemExit(0)

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
mosfet.on()

while True:
    sleep(3600)
' "$MOSFET_GPIO_PIN" &
  mosfet_guard_pid="$!"
  sleep 0.5

  if ! kill -0 "$mosfet_guard_pid" >/dev/null 2>&1; then
    wait "$mosfet_guard_pid" || true
    echo "MOSFET power guard failed to start." >&2
    exit 1
  fi
}

stop_mosfet_power_guard() {
  if [[ -z "$mosfet_guard_pid" ]]; then
    return 0
  fi

  if kill -0 "$mosfet_guard_pid" >/dev/null 2>&1; then
    kill -TERM "$mosfet_guard_pid" >/dev/null 2>&1 || true
    wait "$mosfet_guard_pid" >/dev/null 2>&1 || true
  fi
  mosfet_guard_pid=""
}

detach_sense_hat_kernel_drivers() {
  local sudo_cmd=()
  local device
  local unbind_path

  if (( EUID != 0 )); then
    sudo_cmd=(sudo)
  fi

  echo "Detaching Sense HAT kernel drivers..."
  for device in $SENSE_HAT_I2C_DEVICES; do
    unbind_path="/sys/bus/i2c/devices/$device/driver/unbind"
    if [[ -L "/sys/bus/i2c/devices/$device/driver" ]]; then
      if printf '%s\n' "$device" | "${sudo_cmd[@]}" tee "$unbind_path" >/dev/null 2>&1; then
        echo "  detached $device"
      else
        echo "  could not detach $device; continuing"
      fi
    else
      echo "  $device is not bound to a kernel driver"
    fi
  done

  sleep 0.5
}

has_ip_address() {
  local address="$1"

  hostname -I | tr ' ' '\n' | grep -Fxq "$address"
}

unique_peer_addresses() {
  local peer
  local seen=" "

  for peer in "$@"; do
    if [[ -z "${peer//[[:space:]]/}" ]]; then
      continue
    fi
    if [[ "$seen" == *" $peer "* ]]; then
      continue
    fi
    seen+="$peer "
    printf '%s\n' "$peer"
  done
}

write_cyclonedds_config() {
  local local_address="$1"
  shift
  local peer
  local peers=()

  mapfile -t peers < <(unique_peer_addresses "$local_address" "$@")

  {
    cat <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="$local_address" priority="default" multicast="false" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
EOF
    for peer in "${peers[@]}"; do
      echo "        <Peer address=\"$peer\" />"
    done
    cat <<EOF
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
  } > "$CYCLONEDDS_CONFIG_PATH"

  export CYCLONEDDS_URI="file://$CYCLONEDDS_CONFIG_PATH"
  echo "Wrote CycloneDDS config: $CYCLONEDDS_CONFIG_PATH"
  echo "  local_address=$local_address"
  echo "  peers=${peers[*]}"
}

configure_dds_network() {
  local requested_mode="${NETWORK_MODE,,}"

  export RMW_IMPLEMENTATION
  export ROS_DOMAIN_ID

  if ! bool_is_true "$CYCLONEDDS_AUTOCONFIG"; then
    dds_network_mode="manual"
    return 0
  fi

  case "$requested_mode" in
    auto)
      if has_ip_address "$PI_HOTSPOT_IP"; then
        dds_network_mode="hotspot"
        dds_local_address="$PI_HOTSPOT_IP"
        dds_peer_addresses=("$JETSON_HOTSPOT_IP" "$JETSON_TAILSCALE_IP")
      elif has_ip_address "$PI_TAILSCALE_IP"; then
        dds_network_mode="tailscale"
        dds_local_address="$PI_TAILSCALE_IP"
        dds_peer_addresses=("$JETSON_TAILSCALE_IP" "$JETSON_HOTSPOT_IP")
      else
        echo "No configured Pi DDS address is present on this device." >&2
        echo "  expected one of: $PI_TAILSCALE_IP $PI_HOTSPOT_IP" >&2
        echo "  current IPs: $(hostname -I)" >&2
        exit 1
      fi
      ;;
    hotspot)
      if ! has_ip_address "$PI_HOTSPOT_IP"; then
        echo "NETWORK_MODE=hotspot requested, but $PI_HOTSPOT_IP is not present." >&2
        echo "  current IPs: $(hostname -I)" >&2
        exit 1
      fi
      dds_network_mode="hotspot"
      dds_local_address="$PI_HOTSPOT_IP"
      dds_peer_addresses=("$JETSON_HOTSPOT_IP" "$JETSON_TAILSCALE_IP")
      ;;
    tailscale|wifi)
      if ! has_ip_address "$PI_TAILSCALE_IP"; then
        echo "NETWORK_MODE=$NETWORK_MODE requested, but $PI_TAILSCALE_IP is not present." >&2
        echo "  current IPs: $(hostname -I)" >&2
        exit 1
      fi
      dds_network_mode="tailscale"
      dds_local_address="$PI_TAILSCALE_IP"
      dds_peer_addresses=("$JETSON_TAILSCALE_IP" "$JETSON_HOTSPOT_IP")
      ;;
    none)
      dds_network_mode="none"
      return 0
      ;;
    *)
      echo "NETWORK_MODE must be auto, hotspot, tailscale, wifi, or none; got: $NETWORK_MODE" >&2
      exit 1
      ;;
  esac

  write_cyclonedds_config "$dds_local_address" "${dds_peer_addresses[@]}"
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
  stop_mosfet_power_guard
  wait >/dev/null 2>&1 || true
}

topic_visible() {
  local topic="$1"

  ros2 topic list 2>/dev/null | grep -Fxq "$(normalize_ros_topic "$topic")"
}

normalize_ros_topic() {
  local topic="$1"

  if [[ "$topic" == /* ]]; then
    printf '%s\n' "$topic"
  else
    printf '/%s\n' "$topic"
  fi
}

print_command_topic_info() {
  local topic
  local normalized_topic
  local info
  local available_topics
  local missing=()

  if ! bool_is_true "$COMMAND_MONITOR_ENABLED"; then
    return 0
  fi

  available_topics="$(ros2 topic list 2>/dev/null || true)"
  echo "Pi command topic endpoints:"
  for topic in $COMMAND_MONITOR_TOPICS; do
    normalized_topic="$(normalize_ros_topic "$topic")"
    if ! grep -Fxq "$normalized_topic" <<<"$available_topics"; then
      missing+=("$topic")
      continue
    fi
    info="$(ros2 topic info "$normalized_topic" 2>/dev/null | tr '\n' '; ' || true)"
    echo "  $topic: $info"
  done

  if (( ${#missing[@]} > 0 )); then
    case "${COMMAND_MONITOR_SHOW_MISSING,,}" in
      true|yes|on|summary)
        echo "  missing: ${missing[*]}"
        ;;
    esac
  fi
}

print_command_samples() {
  local topic
  local normalized_topic
  local sample
  local available_topics
  local missing=()

  if ! bool_is_true "$COMMAND_MONITOR_ENABLED"; then
    return 0
  fi

  available_topics="$(ros2 topic list 2>/dev/null || true)"
  echo "Pi command monitor:"
  for topic in $COMMAND_MONITOR_TOPICS; do
    normalized_topic="$(normalize_ros_topic "$topic")"
    if ! grep -Fxq "$normalized_topic" <<<"$available_topics"; then
      missing+=("$topic")
      continue
    fi

    sample="$(
      timeout "$COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC" \
        ros2 topic echo "$normalized_topic" --once 2>/dev/null || true
    )"
    if [[ -n "${sample//[[:space:]]/}" ]]; then
      echo "  $topic:"
      sed 's/^/    /' <<<"$sample" | head -20
    else
      echo "  $topic: visible, no sample within ${COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC}s"
    fi
  done

  if (( ${#missing[@]} > 0 )); then
    case "${COMMAND_MONITOR_SHOW_MISSING,,}" in
      true|yes|on|summary)
        echo "  missing: ${missing[*]}"
        ;;
    esac
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --hotspot|--tailscale|--wifi|--none|--auto)
      NETWORK_MODE="${1#--}"
      shift
      ;;
    --network-mode)
      if [[ $# -lt 2 ]]; then
        echo "--network-mode requires a value." >&2
        exit 1
      fi
      NETWORK_MODE="$2"
      shift 2
      ;;
    --network-mode=*)
      NETWORK_MODE="${1#--network-mode=}"
      shift
      ;;
    *)
      ros_launch_extra_args+=("$1")
      shift
      ;;
  esac
done

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

configure_dds_network

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

trap cleanup EXIT INT TERM
start_mosfet_power_guard

if bool_is_true "$START_SENSE_HAT" && bool_is_true "$SENSE_HAT_DETACH_KERNEL"; then
  detach_sense_hat_kernel_drivers
fi

if bool_is_true "$SETUP_CAN"; then
  setup_can_interface
fi

launch_args=(
  "start_sense_hat:=$START_SENSE_HAT"
  "start_balance_controller:=$START_BALANCE_CONTROLLER"
  "start_motor_control:=$START_MOTOR_CONTROL"
  "hardware_enabled:=$HARDWARE_ENABLED"
  "allow_pd_fallback:=$ALLOW_PD_FALLBACK"
  "allow_missing_imu:=$ALLOW_MISSING_IMU"
  "nav_cmd_topic:=$NAV_CMD_TOPIC"
  "manual_cmd_topic:=$MANUAL_CMD_TOPIC"
  "balanced_cmd_topic:=$BALANCED_CMD_TOPIC"
  "imu_topic:=$IMU_TOPIC"
  "roll_offset_rad:=$ROLL_OFFSET_RAD"
  "pitch_offset_rad:=$PITCH_OFFSET_RAD"
  "balance_nav_timeout_sec:=$BALANCE_NAV_TIMEOUT_SEC"
  "balance_manual_timeout_sec:=$BALANCE_MANUAL_TIMEOUT_SEC"
  "motor_nav_cmd_timeout_sec:=$MOTOR_NAV_CMD_TIMEOUT_SEC"
  "motor_manual_cmd_timeout_sec:=$MOTOR_MANUAL_CMD_TIMEOUT_SEC"
  "motor_ipc_watchdog_timeout_sec:=$MOTOR_IPC_WATCHDOG_TIMEOUT_SEC"
  "motor_power_slew_rate_per_sec:=$MOTOR_POWER_SLEW_RATE_PER_SEC"
  "motor_power_deadband:=$MOTOR_POWER_DEADBAND"
  "motor_min_power:=$MOTOR_MIN_POWER"
  "can_channel:=$CAN_CHANNEL"
  "can_interface:=$CAN_INTERFACE"
  "can_bitrate:=$CAN_BITRATE"
)

if [[ -n "${POLICY_PATH//[[:space:]]/}" ]]; then
  launch_args=("policy_path:=$POLICY_PATH" "${launch_args[@]}")
fi

echo "B_Cubed Pi balanced drive launch"
echo "  ros_domain_id=${ROS_DOMAIN_ID:-default}"
echo "  rmw=${RMW_IMPLEMENTATION:-default}"
echo "  cyclonedds_uri=${CYCLONEDDS_URI:-default}"
echo "  network_mode=$NETWORK_MODE"
echo "  dds_network_mode=${dds_network_mode:-default}"
echo "  dds_local_address=${dds_local_address:-default}"
echo "  current_ips=$(hostname -I)"
echo "  start_sense_hat=$START_SENSE_HAT"
echo "  start_balance_controller=$START_BALANCE_CONTROLLER"
echo "  sense_hat_detach_kernel=$SENSE_HAT_DETACH_KERNEL"
echo "  sense_hat_i2c_devices=$SENSE_HAT_I2C_DEVICES"
echo "  start_motor_control=$START_MOTOR_CONTROL"
echo "  hardware_enabled=$HARDWARE_ENABLED"
echo "  mosfet_power_enabled=$MOSFET_POWER_ENABLED"
echo "  mosfet_gpio_pin=$MOSFET_GPIO_PIN"
echo "  allow_missing_imu=$ALLOW_MISSING_IMU"
echo "  nav_cmd_topic=$NAV_CMD_TOPIC"
echo "  manual_cmd_topic=$MANUAL_CMD_TOPIC"
echo "  balanced_cmd_topic=$BALANCED_CMD_TOPIC"
echo "  balance_nav_timeout_sec=$BALANCE_NAV_TIMEOUT_SEC"
echo "  balance_manual_timeout_sec=$BALANCE_MANUAL_TIMEOUT_SEC"
echo "  motor_nav_cmd_timeout_sec=$MOTOR_NAV_CMD_TIMEOUT_SEC"
echo "  motor_manual_cmd_timeout_sec=$MOTOR_MANUAL_CMD_TIMEOUT_SEC"
echo "  motor_ipc_watchdog_timeout_sec=$MOTOR_IPC_WATCHDOG_TIMEOUT_SEC"
echo "  motor_power_slew_rate_per_sec=$MOTOR_POWER_SLEW_RATE_PER_SEC"
echo "  motor_power_deadband=$MOTOR_POWER_DEADBAND"
echo "  motor_min_power=$MOTOR_MIN_POWER"
echo "  can_setup=$CAN_SETUP"
echo "  can_channel=$CAN_CHANNEL"
echo "  can_interface=$CAN_INTERFACE"
echo "  can_bitrate=$CAN_BITRATE"
echo "  can_device=$CAN_DEVICE"
echo "  expected_kernel=5.15"
echo "  current_kernel=$(uname -r)"
echo "  command_monitor_enabled=$COMMAND_MONITOR_ENABLED"
echo "  command_monitor_show_missing=$COMMAND_MONITOR_SHOW_MISSING"
echo "  command_monitor_topics=$COMMAND_MONITOR_TOPICS"

ros2 launch bb8_balance_controller pi_balanced_drive.launch.py \
  "${launch_args[@]}" \
  "${ros_launch_extra_args[@]}" &
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
