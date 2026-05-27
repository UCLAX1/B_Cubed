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

SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"
MAP_OUTPUT_DIR="${MAP_OUTPUT_DIR:-$WS_DIR/maps}"
MAP_SESSION_NAME="${MAP_SESSION_NAME:-drive_${SESSION_STAMP}}"
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
TOPIC_WAIT_TIMEOUT_SEC="${TOPIC_WAIT_TIMEOUT_SEC:-90}"

BASE_FRAME="${BASE_FRAME:-base_link}"
BASE_TO_CAMERA_TRANSLATION="${BASE_TO_CAMERA_TRANSLATION:-0.0,0.0,0.381}"
BASE_TO_CAMERA_RPY="${BASE_TO_CAMERA_RPY:-0.0,0.0,0.0}"
FLATTEN_NAVIGATION_TO_2D="${FLATTEN_NAVIGATION_TO_2D:-true}"
NAVIGATION_PLANE_Z="${NAVIGATION_PLANE_Z:-0.0}"

ENABLE_NAV2="${ENABLE_NAV2:-true}"
ENABLE_PLANNER_ONLY="${ENABLE_PLANNER_ONLY:-false}"
ENABLE_PLANNING_CONSOLE="${ENABLE_PLANNING_CONSOLE:-true}"
PLANNING_CONSOLE_HOST="${PLANNING_CONSOLE_HOST:-0.0.0.0}"
PLANNING_CONSOLE_PORT="${PLANNING_CONSOLE_PORT:-8080}"
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

  echo "Starting $name; log: $log_file"
  (
    cd "$WS_DIR"
    exec "$@"
  ) >>"$log_file" 2>&1 &
  child_pids+=("$!")
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + TOPIC_WAIT_TIMEOUT_SEC))

  echo "Waiting for $topic ..."
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "$topic is visible."
      return 0
    fi
    sleep 2
  done

  echo "Timed out waiting for $topic after ${TOPIC_WAIT_TIMEOUT_SEC}s." >&2
  return 1
}

wait_for_required_zed_topics() {
  wait_for_topic "$INPUT_POSE_TOPIC"
  if [[ -n "${WAIT_FOR_EXTRA_TOPICS:-}" ]]; then
    local topic
    IFS=',' read -r -a extra_topics <<<"$WAIT_FOR_EXTRA_TOPICS"
    for topic in "${extra_topics[@]}"; do
      topic="${topic//[[:space:]]/}"
      [[ -n "$topic" ]] && wait_for_topic "$topic"
    done
  fi
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

if bool_is_true "$BUILD_FIRST" || [[ ! -f "$INSTALL_SETUP" ]]; then
  colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release
fi

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "Workspace setup file not found: $INSTALL_SETUP" >&2
  exit 1
fi
source_setup_file "$INSTALL_SETUP"

echo "B_Cubed headless launch"
echo "  mode=$SLAM_MODE"
echo "  ros_domain_id=$ROS_DOMAIN_ID"
echo "  map_prefix=$MAP_PREFIX"
echo "  map_file_name=$MAP_FILE_NAME"
echo "  web_console=http://${PLANNING_CONSOLE_HOST}:${PLANNING_CONSOLE_PORT}/"

if bool_is_true "$START_WRAPPER"; then
  launch_background "zed_wrapper" bash -lc "$WRAPPER_LAUNCH"
fi

wait_for_required_zed_topics

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

while :; do
  for pid in "${child_pids[@]}"; do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      wait "$pid"
      exit $?
    fi
  done
  sleep 2
done
