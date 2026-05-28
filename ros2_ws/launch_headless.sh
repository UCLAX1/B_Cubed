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
TOPIC_WAIT_TIMEOUT_SEC="${TOPIC_WAIT_TIMEOUT_SEC:-60}"
STATUS_INTERVAL_SEC="${STATUS_INTERVAL_SEC:-30}"

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

  trap - EXIT INT TERM
  for pid in "${child_pids[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
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
    available_topics="$(ros2 topic list 2>/dev/null || true)"
    mapfile -t missing < <(missing_topics "$available_topics")

    if (( ${#missing[@]} == 0 )); then
      return 0
    fi

    if (( SECONDS >= next_report )); then
      echo "Still waiting for ZED wrapper topics:"
      printf '  %s\n' "${missing[@]}"
      next_report=$((SECONDS + STATUS_INTERVAL_SEC))
    fi

    sleep 2
  done

  echo "Timed out waiting for required ZED wrapper topics." >&2
  echo "Still missing:" >&2
  printf '  %s\n' "${missing[@]}" >&2
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
export ROS_DOMAIN_ID ROS_LOG_DIR
trap cleanup EXIT INT TERM

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
echo "  map_prefix=$MAP_PREFIX"
echo "  map_file_name=$MAP_FILE_NAME"
echo "  web_console=http://${PLANNING_CONSOLE_HOST}:${PLANNING_CONSOLE_PORT}/"
echo "  wrapper_log=$ROS_LOG_DIR/zed_wrapper.log"
echo "  nav_log=$ROS_LOG_DIR/zed_slam_nav.log"
echo "  required_zed_topics=$INPUT_POSE_TOPIC $INPUT_ODOM_TOPIC $CLOUD_TOPIC"

if bool_is_true "$START_WRAPPER"; then
  launch_background "zed_wrapper" bash -lc "$WRAPPER_LAUNCH"
  sleep 5
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
echo "Open the planning console at: http://${PLANNING_CONSOLE_HOST}:${PLANNING_CONSOLE_PORT}/"

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
    next_status_report=$((SECONDS + STATUS_INTERVAL_SEC))
  fi

  sleep 2
done
