#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"
REPO_DIR="$(cd -- "$WS_DIR/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-$WS_DIR/install/setup.bash}"

BUILD_FIRST="${BUILD_FIRST:-false}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/log/headless}"
NPROC_VALUE="$(command -v nproc >/dev/null 2>&1 && nproc || echo 4)"

NETWORK_MODE="${NETWORK_MODE:-auto}"
HOTSPOT_CONNECTION="${HOTSPOT_CONNECTION:-Hotspot}"
WIFI_CONNECTION="${WIFI_CONNECTION:-eduroam}"
HOTSPOT_KEEP_ACTIVE_ON_EXIT="${HOTSPOT_KEEP_ACTIVE_ON_EXIT:-true}"
JETSON_TAILSCALE_IP="${JETSON_TAILSCALE_IP:-100.86.7.33}"
JETSON_HOTSPOT_IP="${JETSON_HOTSPOT_IP:-10.42.0.1}"
PI_TAILSCALE_IP="${PI_TAILSCALE_IP:-100.80.7.37}"
PI_HOTSPOT_IP="${PI_HOTSPOT_IP:-10.42.0.166}"
NETWORK_SWITCH_TIMEOUT_SEC="${NETWORK_SWITCH_TIMEOUT_SEC:-30}"
WRITE_CYCLONEDDS_CONFIG="${WRITE_CYCLONEDDS_CONFIG:-true}"
CYCLONEDDS_CONFIG_PATH="${CYCLONEDDS_CONFIG_PATH:-$HOME/cyclonedds.xml}"
if [[ "${CYCLONEDDS_URI:-}" == file://* ]]; then
  CYCLONEDDS_CONFIG_PATH="${CYCLONEDDS_URI#file://}"
fi
CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$CYCLONEDDS_CONFIG_PATH}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

DEFAULT_LAUNCH_MAX_CORES="$NPROC_VALUE"
if (( DEFAULT_LAUNCH_MAX_CORES > 4 )); then
  DEFAULT_LAUNCH_MAX_CORES=4
fi
DEFAULT_BUILD_PARALLEL_WORKERS="$DEFAULT_LAUNCH_MAX_CORES"

BUILD_PARALLEL_WORKERS="${BUILD_PARALLEL_WORKERS:-$DEFAULT_BUILD_PARALLEL_WORKERS}"
LAUNCH_MAX_CORES="${LAUNCH_MAX_CORES:-$DEFAULT_LAUNCH_MAX_CORES}"
LAUNCH_CPU_SET="${LAUNCH_CPU_SET:-}"
LAUNCH_THREAD_LIMIT="${LAUNCH_THREAD_LIMIT:-2}"
LAUNCH_NICE="${LAUNCH_NICE:-10}"
LAUNCH_IONICE_PRIORITY="${LAUNCH_IONICE_PRIORITY:-7}"
LAUNCH_MEMORY_LIMIT_MB="${LAUNCH_MEMORY_LIMIT_MB:-0}"
LAUNCH_MEMORY_LIMIT_KB=0

SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"
MAP_OUTPUT_DIR="${MAP_OUTPUT_DIR:-$WS_DIR/maps}"
MAP_SESSION_NAME="${MAP_SESSION_NAME:-handheld_${SESSION_STAMP}}"
MAP_PREFIX="${MAP_PREFIX:-$MAP_OUTPUT_DIR/$MAP_SESSION_NAME}"
LOCALIZATION_FLAG_FILE="${LOCALIZATION_FLAG_FILE:-$WS_DIR/.b_cubed_localization}"
MAP_FILE_NAME="${MAP_FILE_NAME:-$MAP_PREFIX}"

START_WRAPPER="${START_WRAPPER:-true}"
KILL_STALE_ZED_WRAPPER="${KILL_STALE_ZED_WRAPPER:-true}"
ZED_STARTUP_GRACE_SEC="${ZED_STARTUP_GRACE_SEC:-8}"
CAMERA_MODEL="${CAMERA_MODEL:-zedm}"
ZED_GRAB_RESOLUTION="${ZED_GRAB_RESOLUTION:-VGA}"
ZED_GRAB_FRAME_RATE="${ZED_GRAB_FRAME_RATE:-30}"
ZED_PARAM_OVERRIDES="${ZED_PARAM_OVERRIDES:-general.grab_resolution:=${ZED_GRAB_RESOLUTION};general.grab_frame_rate:=${ZED_GRAB_FRAME_RATE};pos_tracking.pos_tracking_enabled:=true;pos_tracking.area_memory:=true;pos_tracking.two_d_mode:=true;debug.use_pub_timestamps:=true}"
WRAPPER_LAUNCH="${WRAPPER_LAUNCH:-ros2 launch zed_wrapper zed_camera.launch.py camera_model:=${CAMERA_MODEL} publish_tf:=false publish_map_tf:=false param_overrides:='${ZED_PARAM_OVERRIDES}'}"

INPUT_POSE_TOPIC="${INPUT_POSE_TOPIC:-/zed/zed_node/pose}"
INPUT_POSE_COV_TOPIC="${INPUT_POSE_COV_TOPIC:-/zed/zed_node/pose_with_covariance}"
INPUT_ODOM_TOPIC="${INPUT_ODOM_TOPIC:-/zed/zed_node/odom}"
INPUT_IMAGE_TOPIC="${INPUT_IMAGE_TOPIC:-/zed/zed_node/rgb/color/rect/image/compressed}"
INPUT_IMAGE_IS_COMPRESSED="${INPUT_IMAGE_IS_COMPRESSED:-true}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/zed/zed_node/point_cloud/cloud_registered}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
REQUIRE_POSE_COV_TOPIC="${REQUIRE_POSE_COV_TOPIC:-false}"
TOPIC_WAIT_TIMEOUT_SEC="${TOPIC_WAIT_TIMEOUT_SEC:-180}"
STATUS_INTERVAL_SEC="${STATUS_INTERVAL_SEC:-30}"
COMMAND_MONITOR_ENABLED="${COMMAND_MONITOR_ENABLED:-true}"
COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC="${COMMAND_MONITOR_SAMPLE_TIMEOUT_SEC:-2}"

BASE_FRAME="${BASE_FRAME:-zed_camera_link}"
BASE_TO_CAMERA_TRANSLATION="${BASE_TO_CAMERA_TRANSLATION:-0.0,0.0,0.0}"
BASE_TO_CAMERA_RPY="${BASE_TO_CAMERA_RPY:-0.0,0.0,0.0}"
FLATTEN_NAVIGATION_TO_2D="${FLATTEN_NAVIGATION_TO_2D:-true}"
NAVIGATION_PLANE_Z="${NAVIGATION_PLANE_Z:-0.0}"

ENABLE_NAV2="${ENABLE_NAV2:-false}"
ENABLE_PLANNER_ONLY="${ENABLE_PLANNER_ONLY:-true}"
ENABLE_PLANNING_CONSOLE="${ENABLE_PLANNING_CONSOLE:-true}"
PLANNING_CONSOLE_HOST="${PLANNING_CONSOLE_HOST:-0.0.0.0}"
PLANNING_CONSOLE_PORT="${PLANNING_CONSOLE_PORT:-8080}"
KILL_STALE_PLANNING_CONSOLE="${KILL_STALE_PLANNING_CONSOLE:-true}"
MANUAL_CMD_TOPIC="${MANUAL_CMD_TOPIC:-cmd_vel_manual}"
COMMAND_MONITOR_TOPICS="${COMMAND_MONITOR_TOPICS:-$MANUAL_CMD_TOPIC /cmd_vel /cmd_vel_balanced /jet_cmd /kiwi_drive/motor_powers /balance/status /kiwi_drive/status}"

START_PERSON_TRACKING="${START_PERSON_TRACKING:-false}"
PERSON_TRACKING_IMAGE_TOPIC="${PERSON_TRACKING_IMAGE_TOPIC:-$INPUT_IMAGE_TOPIC}"
PERSON_TRACKING_ENGINE_PATH="${PERSON_TRACKING_ENGINE_PATH:-$REPO_DIR/models/yolo11n.engine}"
SHOW_PERSON_TRACKING_WINDOW="${SHOW_PERSON_TRACKING_WINDOW:-false}"
PUBLISH_PERSON_TRACKING_IMAGE="${PUBLISH_PERSON_TRACKING_IMAGE:-true}"
PERSON_TRACKING_TOPIC="${PERSON_TRACKING_TOPIC:-/person_tracking/detections}"
PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC="${PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC:-/person_tracking/annotated_image/compressed}"
PERSON_TRACKING_CONFIDENCE_THRESHOLD="${PERSON_TRACKING_CONFIDENCE_THRESHOLD:-0.4}"
PERSON_TRACKING_NMS_THRESHOLD="${PERSON_TRACKING_NMS_THRESHOLD:-0.45}"

ENABLE_TRACKING_VISUALIZATION="${ENABLE_TRACKING_VISUALIZATION:-false}"
SHOW_TRACKING_WINDOW="${SHOW_TRACKING_WINDOW:-false}"
PUBLISH_TRACKING_IMAGE="${PUBLISH_TRACKING_IMAGE:-false}"

SLAM_MODE="${SLAM_MODE:-mapping}"
child_pids=()
child_names=()
child_logs=()

usage() {
  cat <<EOF
Usage: $0 [--mapping|--localization] [--map-file /path/to/map_prefix] [--no-wrapper]

Environment overrides:
  ROS_DOMAIN_ID, MAP_PREFIX, LOCALIZATION_FLAG_FILE, ENABLE_NAV2,
  ENABLE_PLANNING_CONSOLE, PLANNING_CONSOLE_HOST, PLANNING_CONSOLE_PORT.
  KILL_STALE_ZED_WRAPPER, TOPIC_WAIT_TIMEOUT_SEC, STATUS_INTERVAL_SEC.
  COMMAND_MONITOR_ENABLED, COMMAND_MONITOR_TOPICS.
  NETWORK_MODE=auto|tailscale|hotspot|wifi|none, HOTSPOT_CONNECTION, WIFI_CONNECTION.
  HOTSPOT_KEEP_ACTIVE_ON_EXIT=true keeps the Jetson hotspot up after Ctrl-C.
EOF
}

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

cleanup() {
  local pid
  local still_running=()

  trap - EXIT INT TERM
  if (( ${#child_pids[@]} > 0 )); then
    echo "Shutting down launch processes..."
  fi

  for pid in "${child_pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -INT "$pid" >/dev/null 2>&1 || true
    fi
  done

  for _ in 1 2 3 4 5 6 7 8 9 10; do
    still_running=()
    for pid in "${child_pids[@]}"; do
      if kill -0 "$pid" >/dev/null 2>&1; then
        still_running+=("$pid")
      fi
    done
    if (( ${#still_running[@]} == 0 )); then
      break
    fi
    sleep 1
  done

  for pid in "${still_running[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      echo "Escalating shutdown for pid $pid"
      kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done

  sleep 2
  still_running=()
  for pid in "${child_pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      still_running+=("$pid")
    fi
  done

  for pid in "${still_running[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      echo "Force-stopping pid $pid"
      kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
  done

  wait >/dev/null 2>&1 || true
  keep_selected_network_mode_active || true
}

run_sudo_command() {
  "$@" && return 0

  if (( EUID == 0 )); then
    return $?
  fi

  sudo -n "$@"
}

has_ipv4_address() {
  local address="$1"

  ip -4 addr show | grep -q "inet ${address}/"
}

wait_for_ipv4_address() {
  local address="$1"
  local deadline=$((SECONDS + NETWORK_SWITCH_TIMEOUT_SEC))

  while (( SECONDS < deadline )); do
    if has_ipv4_address "$address"; then
      return 0
    fi
    sleep 1
  done

  return 1
}

write_cyclonedds_config() {
  local local_address="$1"
  shift
  local peer

  if ! bool_is_true "$WRITE_CYCLONEDDS_CONFIG"; then
    return 0
  fi

  mkdir -p "$(dirname "$CYCLONEDDS_CONFIG_PATH")"
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
    echo "        <Peer address=\"$local_address\" />"
    for peer in "$@"; do
      if [[ "$peer" == "$local_address" ]]; then
        continue
      fi
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
  } >"$CYCLONEDDS_CONFIG_PATH"
  echo "Wrote CycloneDDS config: $CYCLONEDDS_CONFIG_PATH"
  echo "  local_address=$local_address"
  echo "  peers=$local_address $*"
}

configure_network_mode() {
  case "$NETWORK_MODE" in
    auto)
      if has_ipv4_address "$JETSON_HOTSPOT_IP"; then
        NETWORK_MODE="hotspot"
        write_cyclonedds_config "$JETSON_HOTSPOT_IP" "$PI_HOTSPOT_IP" "$PI_TAILSCALE_IP"
      elif has_ipv4_address "$JETSON_TAILSCALE_IP"; then
        NETWORK_MODE="tailscale"
        write_cyclonedds_config "$JETSON_TAILSCALE_IP" "$PI_TAILSCALE_IP" "$PI_HOTSPOT_IP"
      else
        echo "NETWORK_MODE=auto could not find a configured Jetson address." >&2
        echo "  expected one of: $JETSON_HOTSPOT_IP $JETSON_TAILSCALE_IP" >&2
        echo "Available IPv4 addresses:" >&2
        ip -4 addr show >&2
        exit 1
      fi
      ;;
    hotspot)
      echo "Switching Jetson network to hotspot profile '$HOTSPOT_CONNECTION'..."
      if ! command -v nmcli >/dev/null 2>&1; then
        echo "nmcli is required for NETWORK_MODE=hotspot." >&2
        exit 1
      fi
      if ! nmcli -t -f NAME connection show | grep -Fxq "$HOTSPOT_CONNECTION"; then
        echo "NetworkManager connection not found: $HOTSPOT_CONNECTION" >&2
        exit 1
      fi
      run_sudo_command nmcli connection up "$HOTSPOT_CONNECTION"
      if ! wait_for_ipv4_address "$JETSON_HOTSPOT_IP"; then
        echo "Hotspot did not expose $JETSON_HOTSPOT_IP within ${NETWORK_SWITCH_TIMEOUT_SEC}s." >&2
        echo "Available IPv4 addresses:" >&2
        ip -4 addr show >&2
        exit 1
      fi
      write_cyclonedds_config "$JETSON_HOTSPOT_IP" "$PI_HOTSPOT_IP" "$PI_TAILSCALE_IP"
      ;;
    tailscale|wifi)
      if [[ "$NETWORK_MODE" == "wifi" ]]; then
        echo "Switching Jetson network to Wi-Fi profile '$WIFI_CONNECTION'..."
        if command -v nmcli >/dev/null 2>&1; then
          run_sudo_command nmcli connection up "$WIFI_CONNECTION"
        fi
      fi
      if ! wait_for_ipv4_address "$JETSON_TAILSCALE_IP"; then
        echo "Tailscale address $JETSON_TAILSCALE_IP is not available." >&2
        echo "Available IPv4 addresses:" >&2
        ip -4 addr show >&2
        exit 1
      fi
      write_cyclonedds_config "$JETSON_TAILSCALE_IP" "$PI_TAILSCALE_IP" "$PI_HOTSPOT_IP"
      ;;
    none)
      echo "Skipping network and CycloneDDS reconfiguration."
      ;;
    *)
      echo "NETWORK_MODE must be auto, hotspot, tailscale, wifi, or none; got: $NETWORK_MODE" >&2
      exit 2
      ;;
  esac
}

keep_selected_network_mode_active() {
  if [[ "$NETWORK_MODE" != "hotspot" ]] ||
    ! bool_is_true "$HOTSPOT_KEEP_ACTIVE_ON_EXIT"; then
    return 0
  fi
  if ! command -v nmcli >/dev/null 2>&1; then
    return 0
  fi

  if nmcli -t -f NAME connection show --active 2>/dev/null |
    grep -Fxq "$HOTSPOT_CONNECTION" &&
    wait_for_ipv4_address "$JETSON_HOTSPOT_IP"; then
    echo "Leaving Jetson hotspot active: $HOTSPOT_CONNECTION ($JETSON_HOTSPOT_IP)"
    return 0
  fi

  echo "Re-asserting Jetson hotspot profile '$HOTSPOT_CONNECTION' before exit..."
  if run_sudo_command nmcli connection up "$HOTSPOT_CONNECTION" &&
    wait_for_ipv4_address "$JETSON_HOTSPOT_IP"; then
    echo "Jetson hotspot remains active: $HOTSPOT_CONNECTION ($JETSON_HOTSPOT_IP)"
  else
    echo "Warning: could not verify that hotspot '$HOTSPOT_CONNECTION' stayed active." >&2
  fi
}

planning_console_url_host() {
  if [[ "$PLANNING_CONSOLE_HOST" != "0.0.0.0" && "$PLANNING_CONSOLE_HOST" != "::" ]]; then
    printf '%s\n' "$PLANNING_CONSOLE_HOST"
    return 0
  fi

  case "$NETWORK_MODE" in
    hotspot)
      printf '%s\n' "$JETSON_HOTSPOT_IP"
      ;;
    tailscale|wifi)
      printf '%s\n' "$JETSON_TAILSCALE_IP"
      ;;
    *)
      hostname -I | awk '{print $1}'
      ;;
  esac
}

launch_background() {
  local name="$1"
  shift
  local log_file="$ROS_LOG_DIR/${name}.log"
  local command_display

  echo "Starting $name; log: $log_file"
  printf -v command_display '%q ' "$@"
  echo "  command: ${command_display% }"
  (
    cd "$WS_DIR"
    apply_resource_environment
    if (( LAUNCH_MEMORY_LIMIT_KB > 0 )); then
      ulimit -v "$LAUNCH_MEMORY_LIMIT_KB"
    fi
    exec "$@"
  ) >>"$log_file" 2>&1 &
  child_pids+=("$!")
  child_names+=("$name")
  child_logs+=("$log_file")
  echo "  pid: ${child_pids[-1]}"
}

child_index_by_name() {
  local wanted_name="$1"
  local i

  for i in "${!child_names[@]}"; do
    if [[ "${child_names[$i]}" == "$wanted_name" ]]; then
      printf '%s\n' "$i"
      return 0
    fi
  done

  return 1
}

child_running_by_name() {
  local wanted_name="$1"
  local index

  index="$(child_index_by_name "$wanted_name" || true)"
  if [[ -z "$index" ]]; then
    return 1
  fi

  kill -0 "${child_pids[$index]}" >/dev/null 2>&1
}

child_status_by_name() {
  local wanted_name="$1"
  local index
  local pid

  index="$(child_index_by_name "$wanted_name" || true)"
  if [[ -z "$index" ]]; then
    echo "unknown"
    return 1
  fi

  pid="${child_pids[$index]}"
  if kill -0 "$pid" >/dev/null 2>&1; then
    echo "running pid=$pid"
    return 0
  fi

  echo "stopped pid=$pid"
  return 1
}

sanitize_nonnegative_int() {
  local variable_name="$1"
  local default_value="$2"
  local value="${!variable_name}"

  if ! [[ "$value" =~ ^[0-9]+$ ]]; then
    printf -v "$variable_name" '%s' "$default_value"
  fi
}

configure_resource_limits() {
  sanitize_nonnegative_int BUILD_PARALLEL_WORKERS "$DEFAULT_BUILD_PARALLEL_WORKERS"
  sanitize_nonnegative_int LAUNCH_MAX_CORES "$DEFAULT_LAUNCH_MAX_CORES"
  sanitize_nonnegative_int LAUNCH_THREAD_LIMIT 2
  sanitize_nonnegative_int LAUNCH_NICE 10
  sanitize_nonnegative_int LAUNCH_IONICE_PRIORITY 7
  sanitize_nonnegative_int LAUNCH_MEMORY_LIMIT_MB 0

  if (( BUILD_PARALLEL_WORKERS < 1 )); then
    BUILD_PARALLEL_WORKERS=1
  fi
  if (( BUILD_PARALLEL_WORKERS > NPROC_VALUE )); then
    BUILD_PARALLEL_WORKERS="$NPROC_VALUE"
  fi

  if (( LAUNCH_MAX_CORES < 1 )); then
    LAUNCH_MAX_CORES=1
  fi
  if (( LAUNCH_MAX_CORES > NPROC_VALUE )); then
    LAUNCH_MAX_CORES="$NPROC_VALUE"
  fi
  if [[ -z "${LAUNCH_CPU_SET//[[:space:]]/}" ]]; then
    if (( LAUNCH_MAX_CORES == 1 )); then
      LAUNCH_CPU_SET="0"
    else
      LAUNCH_CPU_SET="0-$((LAUNCH_MAX_CORES - 1))"
    fi
  fi

  if (( LAUNCH_THREAD_LIMIT < 1 )); then
    LAUNCH_THREAD_LIMIT=1
  fi
  if (( LAUNCH_NICE > 19 )); then
    LAUNCH_NICE=19
  fi
  if (( LAUNCH_IONICE_PRIORITY > 7 )); then
    LAUNCH_IONICE_PRIORITY=7
  fi

  LAUNCH_MEMORY_LIMIT_KB=$((LAUNCH_MEMORY_LIMIT_MB * 1024))
}

apply_resource_environment() {
  export OMP_NUM_THREADS="$LAUNCH_THREAD_LIMIT"
  export OPENBLAS_NUM_THREADS="$LAUNCH_THREAD_LIMIT"
  export MKL_NUM_THREADS="$LAUNCH_THREAD_LIMIT"
  export NUMEXPR_NUM_THREADS="$LAUNCH_THREAD_LIMIT"
  export OPENCV_FOR_THREADS_NUM="$LAUNCH_THREAD_LIMIT"
  export VECLIB_MAXIMUM_THREADS="$LAUNCH_THREAD_LIMIT"
  export MALLOC_ARENA_MAX=2
  export CUDA_MODULE_LOADING=LAZY
}

run_limited() {
  (
    apply_resource_environment
    if (( LAUNCH_MEMORY_LIMIT_KB > 0 )); then
      ulimit -v "$LAUNCH_MEMORY_LIMIT_KB"
    fi
    exec ionice -c 2 -n "$LAUNCH_IONICE_PRIORITY" \
      nice -n "$LAUNCH_NICE" \
      taskset -c "$LAUNCH_CPU_SET" \
      "$@"
  )
}

missing_ros_packages() {
  local missing=()
  local package_name

  for package_name in "$@"; do
    if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
      missing+=("$package_name")
    fi
  done

  if (( ${#missing[@]} > 0 )); then
    printf '%s\n' "${missing[@]}"
  fi
}

planning_console_pids_on_port() {
  local port="$1"
  local pid

  if ! command -v ss >/dev/null 2>&1; then
    return 0
  fi

  while IFS= read -r pid; do
    if [[ -r "/proc/$pid/cmdline" ]] &&
      tr '\0' ' ' <"/proc/$pid/cmdline" | grep -q "nav_planning_console"; then
      printf '%s\n' "$pid"
    fi
  done < <(
    ss -ltnp "sport = :$port" 2>/dev/null |
      sed -n 's/.*pid=\([0-9]\+\).*/\1/p' |
      sort -u
  )
}

zed_wrapper_candidate_pids() {
  local self_pid="$$"
  local parent_pid="${PPID:-}"
  local pid
  local args

  while read -r pid args; do
    [[ -n "$pid" ]] || continue
    if [[ "$pid" == "$self_pid" || "$pid" == "$parent_pid" ]]; then
      continue
    fi

    case "$args" in
      *"ros2 launch zed_wrapper zed_camera.launch.py"*|\
      *"/component_container_isolated "*"zed_container"*|\
      *"/robot_state_publisher "*"__node:=zed_state_publisher"*|\
      *"/zed_wrapper/"*"zed_camera.launch.py"*)
        printf '%s\n' "$pid"
        ;;
    esac
  done < <(ps -eo pid=,args=)
}

stop_stale_zed_wrapper_processes() {
  local pids=()
  local still_running=()
  local pid

  if ! bool_is_true "$KILL_STALE_ZED_WRAPPER"; then
    return 0
  fi

  mapfile -t pids < <(zed_wrapper_candidate_pids | sort -u)
  if (( ${#pids[@]} == 0 )); then
    return 0
  fi

  echo "Stopping stale ZED wrapper processes before launch: ${pids[*]}"
  kill "${pids[@]}" >/dev/null 2>&1 || true

  for _ in 1 2 3 4 5; do
    still_running=()
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" >/dev/null 2>&1; then
        still_running+=("$pid")
      fi
    done
    if (( ${#still_running[@]} == 0 )); then
      return 0
    fi
    sleep 1
  done

  echo "Force-stopping stale ZED wrapper processes: ${still_running[*]}"
  kill -9 "${still_running[@]}" >/dev/null 2>&1 || true
}

ensure_planning_console_port_available() {
  if ! bool_is_true "$ENABLE_PLANNING_CONSOLE"; then
    return 0
  fi

  local pids=()
  mapfile -t pids < <(planning_console_pids_on_port "$PLANNING_CONSOLE_PORT")

  if (( ${#pids[@]} > 0 )); then
    if bool_is_true "$KILL_STALE_PLANNING_CONSOLE"; then
      echo "Stopping stale nav_planning_console on port $PLANNING_CONSOLE_PORT: ${pids[*]}"
      kill "${pids[@]}" >/dev/null 2>&1 || true
      sleep 1
    else
      echo "nav_planning_console is already listening on port $PLANNING_CONSOLE_PORT: ${pids[*]}" >&2
      echo "Set PLANNING_CONSOLE_PORT to another port or KILL_STALE_PLANNING_CONSOLE=true." >&2
      exit 1
    fi
  fi

  if command -v ss >/dev/null 2>&1 &&
    ss -ltn "sport = :$PLANNING_CONSOLE_PORT" 2>/dev/null | grep -q "LISTEN"; then
    echo "Port $PLANNING_CONSOLE_PORT is already in use by a non-console process." >&2
    echo "Set PLANNING_CONSOLE_PORT to an open port before launching." >&2
    exit 1
  fi
}

configure_nav2_launch() {
  if ! bool_is_true "$ENABLE_NAV2"; then
    return 0
  fi

  local required_packages=(
    nav2_planner
    nav2_controller
    nav2_behaviors
    nav2_bt_navigator
    nav2_lifecycle_manager
    nav2_navfn_planner
    nav2_regulated_pure_pursuit_controller
    nav2_costmap_2d
  )
  local missing=()
  mapfile -t missing < <(missing_ros_packages "${required_packages[@]}")

  if (( ${#missing[@]} == 0 )); then
    return 0
  fi

  echo "Nav2 planning packages are missing:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  echo "Continuing with ENABLE_NAV2=false so mapping and the web console can still start." >&2
  ENABLE_NAV2="false"
}

configure_planner_only_launch() {
  if bool_is_true "$ENABLE_NAV2"; then
    ENABLE_PLANNER_ONLY="false"
    return 0
  fi
  if ! bool_is_true "$ENABLE_PLANNER_ONLY"; then
    return 0
  fi

  local required_packages=(
    nav2_planner
    nav2_lifecycle_manager
    nav2_navfn_planner
    nav2_costmap_2d
  )
  local missing=()
  mapfile -t missing < <(missing_ros_packages "${required_packages[@]}")

  if (( ${#missing[@]} == 0 )); then
    return 0
  fi

  echo "Nav2 planner-only packages are missing:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  echo "Continuing with ENABLE_PLANNER_ONLY=false." >&2
  ENABLE_PLANNER_ONLY="false"
}

missing_topics() {
  local available_topics="$1"
  local missing=()
  local topic

  for topic in \
    "$INPUT_POSE_TOPIC" \
    "$INPUT_ODOM_TOPIC" \
    "$CLOUD_TOPIC"; do
    if ! grep -Fxq "$topic" <<<"$available_topics"; then
      missing+=("$topic")
    fi
  done

  if bool_is_true "$REQUIRE_POSE_COV_TOPIC"; then
    if ! grep -Fxq "$INPUT_POSE_COV_TOPIC" <<<"$available_topics"; then
      missing+=("$INPUT_POSE_COV_TOPIC")
    fi
  fi

  if bool_is_true "$START_PERSON_TRACKING"; then
    if ! grep -Fxq "$PERSON_TRACKING_IMAGE_TOPIC" <<<"$available_topics"; then
      missing+=("$PERSON_TRACKING_IMAGE_TOPIC")
    fi
  fi

  if (( ${#missing[@]} > 0 )); then
    printf '%s\n' "${missing[@]}"
  fi
}

topic_visible() {
  local topic="$1"

  ros2 topic list 2>/dev/null | grep -Fxq "$topic"
}

report_topic_status() {
  local label="$1"
  shift
  local topic
  local seen=()
  local missing=()

  for topic in "$@"; do
    if topic_visible "$topic"; then
      seen+=("$topic")
    else
      missing+=("$topic")
    fi
  done

  echo "$label"
  if (( ${#seen[@]} > 0 )); then
    printf '  seen: %s\n' "${seen[*]}"
  fi
  if (( ${#missing[@]} > 0 )); then
    printf '  waiting: %s\n' "${missing[*]}"
  fi
}

topic_missing() {
  local topic="$1"

  ! topic_visible "$topic"
}

print_command_samples() {
  local topic
  local sample

  if ! bool_is_true "$COMMAND_MONITOR_ENABLED"; then
    return 0
  fi

  echo "Command monitor:"
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

print_command_topic_info() {
  local topic
  local info

  if ! bool_is_true "$COMMAND_MONITOR_ENABLED"; then
    return 0
  fi

  echo "Command topic endpoints:"
  for topic in $COMMAND_MONITOR_TOPICS; do
    if ! topic_visible "$topic"; then
      echo "  $topic: not visible"
      continue
    fi
    info="$(ros2 topic info "$topic" 2>/dev/null | tr '\n' '; ' || true)"
    echo "  $topic: $info"
  done
}

print_nav_diagnostics() {
  local nav_log="$ROS_LOG_DIR/zed_slam_nav.log"
  local nodes
  local actions
  local recent_signal

  echo "Navigation diagnostics:"
  nodes="$(ros2 node list 2>/dev/null | sort | tr '\n' ' ' || true)"
  if [[ -n "${nodes//[[:space:]]/}" ]]; then
    echo "  nodes: $nodes"
  else
    echo "  nodes: none visible"
  fi

  actions="$(ros2 action list 2>/dev/null | sort | tr '\n' ' ' || true)"
  if [[ -n "${actions//[[:space:]]/}" ]]; then
    echo "  actions: $actions"
  else
    echo "  actions: none visible"
  fi

  if [[ -f "$nav_log" ]]; then
    recent_signal="$(
      grep -E "ERROR|WARN|gate_open|localized=|Serving Nav2 planning console|process has died|lifecycle|Planner Server|Controller Server" "$nav_log" \
        | tail -12 || true
    )"
    if [[ -n "${recent_signal//[[:space:]]/}" ]]; then
      echo "  recent nav log signal:"
      sed 's/^/    /' <<<"$recent_signal"
    else
      echo "  recent nav log signal: no warnings/errors/readiness messages yet"
    fi
  else
    echo "  nav log not created yet: $nav_log"
  fi
}

print_zed_log_signal() {
  local wrapper_log="$ROS_LOG_DIR/zed_wrapper.log"
  local recent_signal

  if [[ ! -f "$wrapper_log" ]]; then
    echo "  zed wrapper log not created yet: $wrapper_log"
    return 0
  fi

  recent_signal="$(
    grep -E "Camera successfully opened|Advertised on topic|zed started|Starting Positional Tracking|ERROR|WARN|WARNING|FAILED|Failed|timeout|TIMEOUT" "$wrapper_log" \
      | tail -14 || true
  )"
  if [[ -n "${recent_signal//[[:space:]]/}" ]]; then
    echo "  recent ZED wrapper log signal:"
    sed 's/^/    /' <<<"$recent_signal"
  else
    echo "  recent ZED wrapper log signal: no camera/topic/errors messages yet"
  fi
}

print_visible_zed_topics() {
  local zed_topics

  zed_topics="$(ros2 topic list 2>/dev/null | grep -E '^/zed(/|$)' | sort || true)"
  if [[ -n "${zed_topics//[[:space:]]/}" ]]; then
    echo "Visible ZED topics:"
    sed 's/^/  /' <<<"$zed_topics"
  else
    echo "Visible ZED topics: none"
  fi
}

zed_wrapper_fatal_signal() {
  local wrapper_log="$ROS_LOG_DIR/zed_wrapper.log"

  [[ -f "$wrapper_log" ]] || return 1
  grep -Eq "Camera detection timeout|CAMERA STREAM FAILED TO START|process has died|Caught exception in launch|failed to create domain|does not match an available interface" "$wrapper_log"
}

print_next_blocker_hint() {
  if topic_missing "$SCAN_TOPIC"; then
    echo "Blocking readiness: $SCAN_TOPIC is missing. Check pointcloud_to_laserscan TF input from $CLOUD_TOPIC to $BASE_FRAME."
    return 0
  fi
  if topic_missing "/map"; then
    echo "Blocking readiness: /map is missing. Check slam_toolbox startup in $ROS_LOG_DIR/zed_slam_nav.log."
    return 0
  fi
  if topic_missing "/zed/is_localized"; then
    echo "Blocking readiness: /zed/is_localized is missing. Check zed_tracking subscriptions to ZED pose/odom topics."
    return 0
  fi
  if bool_is_true "$ENABLE_PLANNING_CONSOLE" && ! topic_visible "$MANUAL_CMD_TOPIC"; then
    echo "Waiting for manual command topic $MANUAL_CMD_TOPIC. It may appear after the web console starts."
    return 0
  fi
  if topic_missing "/balance/status"; then
    echo "Pi balance status is not visible yet. Check CycloneDDS peer config and b_cubed_pi.service."
    return 0
  fi

  echo "Core topics are visible; if the robot does not move, check /balance/status and hardware gating."
}

report_child_status() {
  local i
  local pid
  local name
  local log_file
  local state

  echo "Launch process status:"
  for i in "${!child_pids[@]}"; do
    pid="${child_pids[$i]}"
    name="${child_names[$i]}"
    log_file="${child_logs[$i]}"
    if kill -0 "$pid" >/dev/null 2>&1; then
      state="running"
    else
      state="stopped"
    fi
    echo "  $name: $state pid=$pid log=$log_file"
  done
}

tail_log_on_failure() {
  local log_file="$1"

  if [[ -f "$log_file" ]]; then
    echo "Last 80 lines from $log_file:" >&2
    tail -80 "$log_file" >&2 || true
  fi
}

wait_for_wrapper_topics() {
  local deadline=$((SECONDS + TOPIC_WAIT_TIMEOUT_SEC))
  local available_topics=""
  local missing=()
  local next_report=$SECONDS

  while (( SECONDS < deadline )); do
    if bool_is_true "$START_WRAPPER" && ! child_running_by_name "zed_wrapper"; then
      echo "ZED wrapper stopped before required topics appeared." >&2
      echo "  status: $(child_status_by_name "zed_wrapper" || true)" >&2
      tail_log_on_failure "$ROS_LOG_DIR/zed_wrapper.log"
      return 1
    fi

    if zed_wrapper_fatal_signal; then
      echo "ZED wrapper reported a fatal startup error before required topics appeared." >&2
      print_zed_log_signal >&2
      tail_log_on_failure "$ROS_LOG_DIR/zed_wrapper.log"
      return 1
    fi

    available_topics="$(ros2 topic list 2>/dev/null || true)"
    mapfile -t missing < <(missing_topics "$available_topics")

    if (( ${#missing[@]} == 0 )); then
      return 0
    fi

    if (( SECONDS >= next_report )); then
      echo "Still waiting for ZED wrapper topics:"
      printf '  %s\n' "${missing[@]}"
      print_visible_zed_topics
      print_zed_log_signal
      next_report=$((SECONDS + STATUS_INTERVAL_SEC))
    fi

    sleep 2
  done

  echo "Timed out waiting for required ZED wrapper topics." >&2
  echo "Still missing:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  print_visible_zed_topics >&2
  print_zed_log_signal >&2
  tail_log_on_failure "$ROS_LOG_DIR/zed_wrapper.log"
  return 1
}

while (($#)); do
  case "$1" in
    --mapping)
      SLAM_MODE="mapping"
      ;;
    --localization)
      SLAM_MODE="localization"
      ;;
    --map-file)
      shift
      MAP_FILE_NAME="${1:?--map-file requires a posegraph prefix}"
      ;;
    --no-wrapper)
      START_WRAPPER="false"
      ;;
    --hotspot)
      NETWORK_MODE="hotspot"
      ;;
    --auto)
      NETWORK_MODE="auto"
      ;;
    --tailscale|--wifi)
      NETWORK_MODE="${1#--}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "$SLAM_MODE" == "mapping" && -f "$LOCALIZATION_FLAG_FILE" ]]; then
  SLAM_MODE="localization"
  MAP_FILE_NAME="$(<"$LOCALIZATION_FLAG_FILE")"
  MAP_FILE_NAME="${MAP_FILE_NAME:-$MAP_PREFIX}"
fi

if [[ "$SLAM_MODE" != "mapping" && "$SLAM_MODE" != "localization" ]]; then
  echo "SLAM_MODE must be mapping or localization, got: $SLAM_MODE" >&2
  exit 2
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

mkdir -p "$ROS_LOG_DIR" "$MAP_OUTPUT_DIR"
export ROS_DOMAIN_ID ROS_LOG_DIR RMW_IMPLEMENTATION CYCLONEDDS_URI
trap cleanup EXIT INT TERM
configure_network_mode
PLANNING_CONSOLE_URL_HOST="$(planning_console_url_host)"

: >"$ROS_LOG_DIR/zed_wrapper.log"
: >"$ROS_LOG_DIR/zed_slam_nav.log"

source_setup_file "$ROS_SETUP"
cd "$WS_DIR"
configure_resource_limits

if bool_is_true "$BUILD_FIRST" || [[ ! -f "$INSTALL_SETUP" ]]; then
  run_limited colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    --parallel-workers "$BUILD_PARALLEL_WORKERS"
fi

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "Workspace setup file not found: $INSTALL_SETUP" >&2
  exit 1
fi
source_setup_file "$INSTALL_SETUP"
configure_nav2_launch
configure_planner_only_launch
ensure_planning_console_port_available

echo "B_Cubed headless launch"
echo "  mode=$SLAM_MODE"
echo "  ros_domain_id=$ROS_DOMAIN_ID"
echo "  rmw=${RMW_IMPLEMENTATION:-default}"
echo "  cyclonedds_uri=${CYCLONEDDS_URI:-default}"
echo "  network_mode=$NETWORK_MODE"
echo "  map_prefix=$MAP_PREFIX"
echo "  map_file_name=$MAP_FILE_NAME"
echo "  web_console=http://${PLANNING_CONSOLE_URL_HOST}:${PLANNING_CONSOLE_PORT}/"
echo "  wrapper_log=$ROS_LOG_DIR/zed_wrapper.log"
echo "  nav_log=$ROS_LOG_DIR/zed_slam_nav.log"
echo "  required_zed_topics=$INPUT_POSE_TOPIC $INPUT_ODOM_TOPIC $CLOUD_TOPIC"
echo "  topic_wait_timeout_sec=$TOPIC_WAIT_TIMEOUT_SEC"
echo "  command_monitor_enabled=$COMMAND_MONITOR_ENABLED"
echo "  command_monitor_topics=$COMMAND_MONITOR_TOPICS"

if bool_is_true "$START_WRAPPER"; then
  stop_stale_zed_wrapper_processes
  launch_background "zed_wrapper" bash -lc "$WRAPPER_LAUNCH"
  echo "Allowing ZED wrapper ${ZED_STARTUP_GRACE_SEC}s to initialize before topic checks..."
  sleep "$ZED_STARTUP_GRACE_SEC"
fi

echo "Waiting for ZED wrapper topics..."
wait_for_wrapper_topics
echo "Wrapper topics are available."

nav_args=(
  "slam_mode:=$SLAM_MODE"
  "enable_nav2:=$ENABLE_NAV2"
  "enable_planner_only:=$ENABLE_PLANNER_ONLY"
  "enable_planning_console:=$ENABLE_PLANNING_CONSOLE"
  "planning_console_host:=$PLANNING_CONSOLE_HOST"
  "planning_console_port:=$PLANNING_CONSOLE_PORT"
  "manual_cmd_topic:=$MANUAL_CMD_TOPIC"
  "person_tracking_control_enabled:=$START_PERSON_TRACKING"
  "person_tracking_image_topic:=$PERSON_TRACKING_IMAGE_TOPIC"
  "person_tracking_engine_path:=$PERSON_TRACKING_ENGINE_PATH"
  "person_tracking_show_window:=$SHOW_PERSON_TRACKING_WINDOW"
  "person_tracking_publish_annotated_image:=$PUBLISH_PERSON_TRACKING_IMAGE"
  "person_tracking_detection_topic:=$PERSON_TRACKING_TOPIC"
  "person_tracking_annotated_image_topic:=$PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC"
  "person_tracking_confidence_threshold:=$PERSON_TRACKING_CONFIDENCE_THRESHOLD"
  "person_tracking_nms_threshold:=$PERSON_TRACKING_NMS_THRESHOLD"
  "enable_tracking_node:=true"
  "base_frame:=$BASE_FRAME"
  "enable_base_adapter:=true"
  "base_to_camera_translation:=$BASE_TO_CAMERA_TRANSLATION"
  "base_to_camera_rpy:=$BASE_TO_CAMERA_RPY"
  "flatten_navigation_to_2d:=$FLATTEN_NAVIGATION_TO_2D"
  "navigation_plane_z:=$NAVIGATION_PLANE_Z"
  "input_pose_topic:=$INPUT_POSE_TOPIC"
  "input_pose_cov_topic:=$INPUT_POSE_COV_TOPIC"
  "input_odom_topic:=$INPUT_ODOM_TOPIC"
  "input_image_topic:=$INPUT_IMAGE_TOPIC"
  "input_image_is_compressed:=$INPUT_IMAGE_IS_COMPRESSED"
  "cloud_topic:=$CLOUD_TOPIC"
  "scan_topic:=$SCAN_TOPIC"
  "map_file_name:=$MAP_FILE_NAME"
  "enable_tracking_visualization:=$ENABLE_TRACKING_VISUALIZATION"
  "show_tracking_window:=$SHOW_TRACKING_WINDOW"
  "publish_tracking_image:=$PUBLISH_TRACKING_IMAGE"
)

launch_background "zed_slam_nav" ros2 launch depth_processing zed_slam_nav.launch.py "${nav_args[@]}"
echo "zed_slam_nav launch process started."
echo "The stack is now running in the background; this script will keep reporting readiness."
echo "Open the planning console at: http://${PLANNING_CONSOLE_URL_HOST}:${PLANNING_CONSOLE_PORT}/"

report_child_status
report_topic_status "Initial navigation topic status:" \
  "/map" \
  "$SCAN_TOPIC" \
  "/zed/is_localized" \
  "$MANUAL_CMD_TOPIC" \
  "/cmd_vel" \
  "/balance/status"
print_next_blocker_hint
print_nav_diagnostics
print_command_topic_info
print_command_samples

next_status_report=$((SECONDS + STATUS_INTERVAL_SEC))
while :; do
  for i in "${!child_pids[@]}"; do
    pid="${child_pids[$i]}"
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      if wait "$pid"; then
        status=0
      else
        status=$?
      fi
      echo "Launch process '${child_names[$i]}' exited with status $status." >&2
      tail_log_on_failure "${child_logs[$i]}"
      exit "$status"
    fi
  done

  if (( SECONDS >= next_status_report )); then
    report_child_status
    report_topic_status "Navigation topic status:" \
      "/map" \
      "$SCAN_TOPIC" \
      "/zed/is_localized" \
      "$MANUAL_CMD_TOPIC" \
      "/cmd_vel" \
      "/balance/status"
    print_next_blocker_hint
    print_nav_diagnostics
    print_command_topic_info
    print_command_samples
    next_status_report=$((SECONDS + STATUS_INTERVAL_SEC))
  fi

  sleep 2
done
