#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"
INSTALL_SETUP="$WS_DIR/install/local_setup.bash"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is required for ros2_ws/launch.sh" >&2
  exit 1
fi

NPROC_VALUE="$(command -v nproc >/dev/null 2>&1 && nproc || echo 4)"
SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"

MAP_OUTPUT_DIR="${MAP_OUTPUT_DIR:-$WS_DIR/maps}"
MAP_SESSION_NAME="${MAP_SESSION_NAME:-handheld_${SESSION_STAMP}}"
MAP_PREFIX="${MAP_PREFIX:-$MAP_OUTPUT_DIR/$MAP_SESSION_NAME}"

BASE_TO_CAMERA_TRANSLATION="${BASE_TO_CAMERA_TRANSLATION:-0.0,0.0,0.0}"
BASE_TO_CAMERA_RPY="${BASE_TO_CAMERA_RPY:-0.0,0.0,0.0}"
BASE_FRAME="${BASE_FRAME:-zed_camera_link}"

CAMERA_MODEL="${CAMERA_MODEL:-zedm}"
START_WRAPPER="${START_WRAPPER:-true}"
WRAPPER_LAUNCH="${WRAPPER_LAUNCH:-ros2 launch zed_wrapper zed_camera.launch.py camera_model:=${CAMERA_MODEL} publish_tf:=false publish_map_tf:=false param_overrides:='pos_tracking.two_d_mode:=true;debug.use_pub_timestamps:=true'}"

INPUT_POSE_TOPIC="${INPUT_POSE_TOPIC:-/zed/zed_node/pose}"
INPUT_POSE_COV_TOPIC="${INPUT_POSE_COV_TOPIC:-/zed/zed_node/pose_with_covariance}"
INPUT_ODOM_TOPIC="${INPUT_ODOM_TOPIC:-/zed/zed_node/odom}"
INPUT_IMAGE_TOPIC="${INPUT_IMAGE_TOPIC:-/zed/zed_node/rgb/color/rect/image/compressed}"
INPUT_IMAGE_IS_COMPRESSED="${INPUT_IMAGE_IS_COMPRESSED:-true}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/zed/zed_node/point_cloud/cloud_registered}"
REQUIRE_POSE_COV_TOPIC="${REQUIRE_POSE_COV_TOPIC:-false}"

START_GESTURE_RECOGNITION="${START_GESTURE_RECOGNITION:-true}"
GESTURE_IMAGE_TOPIC="${GESTURE_IMAGE_TOPIC:-$INPUT_IMAGE_TOPIC}"
GESTURE_IMAGE_IS_COMPRESSED="${GESTURE_IMAGE_IS_COMPRESSED:-$INPUT_IMAGE_IS_COMPRESSED}"
GESTURE_MODEL_PATH="${GESTURE_MODEL_PATH:-}"
SHOW_GESTURE_WINDOW="${SHOW_GESTURE_WINDOW:-false}"
PUBLISH_GESTURE_ANNOTATED_IMAGE="${PUBLISH_GESTURE_ANNOTATED_IMAGE:-true}"
GESTURE_TOPIC="${GESTURE_TOPIC:-/gesture_recognition/result}"
GESTURE_ANNOTATED_IMAGE_TOPIC="${GESTURE_ANNOTATED_IMAGE_TOPIC:-/gesture_recognition/annotated_image/compressed}"

ENABLE_TRACKING_VISUALIZATION="${ENABLE_TRACKING_VISUALIZATION:-false}"
SHOW_TRACKING_WINDOW="${SHOW_TRACKING_WINDOW:-false}"
PUBLISH_TRACKING_IMAGE="${PUBLISH_TRACKING_IMAGE:-false}"

ENABLE_NAV2="${ENABLE_NAV2:-false}"
ENABLE_PLANNER_ONLY="${ENABLE_PLANNER_ONLY:-true}"
ENABLE_PLANNING_CONSOLE="${ENABLE_PLANNING_CONSOLE:-true}"
PLANNING_CONSOLE_HOST="${PLANNING_CONSOLE_HOST:-127.0.0.1}"
PLANNING_CONSOLE_PORT="${PLANNING_CONSOLE_PORT:-8080}"
PLANNING_CONSOLE_URL_HOST="$PLANNING_CONSOLE_HOST"
if [[ "$PLANNING_CONSOLE_URL_HOST" == "0.0.0.0" ]]; then
  PLANNING_CONSOLE_URL_HOST="127.0.0.1"
fi

START_RVIZ="${START_RVIZ:-false}"
RVIZ_COMMAND="${RVIZ_COMMAND:-rviz2}"

TOPIC_WAIT_TIMEOUT_SEC="${TOPIC_WAIT_TIMEOUT_SEC:-60}"

mkdir -p "$MAP_OUTPUT_DIR"

run_terminal() {
  local title="$1"
  local command="$2"

  gnome-terminal --title="$title" -- bash -lc "
    source '$INSTALL_SETUP'
    $command
    exec bash
  "
}

source_setup_file() {
  local setup_file="$1"

  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

bool_is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

launch_arg() {
  local name="$1"
  local value="$2"

  printf '%q' "${name}:=${value}"
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
  echo >&2
  echo "Continuing with ENABLE_NAV2=false so mapping and the web console can still start." >&2
  echo "Click-to-plan will work after Navigation2 is installed:" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup" >&2
  echo >&2

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
  echo >&2
  echo "Continuing with ENABLE_PLANNER_ONLY=false." >&2
  echo "The web console will still show the map, but click-to-plan needs Navigation2:" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup" >&2
  echo >&2

  ENABLE_PLANNER_ONLY="false"
}

print_instructions() {
  cat <<EOF

Handheld mapping session prefix:
  $MAP_PREFIX

This launcher is for camera-only mapping tests.

It assumes:
  base_frame=$BASE_FRAME
  base_to_camera_translation=$BASE_TO_CAMERA_TRANSLATION
  base_to_camera_rpy=$BASE_TO_CAMERA_RPY

Expected upstream wrapper topics:
  $INPUT_POSE_TOPIC
  $INPUT_ODOM_TOPIC
  $CLOUD_TOPIC

Optional upstream wrapper topics:
  $INPUT_POSE_COV_TOPIC
  $INPUT_IMAGE_TOPIC

MediaPipe gesture recognition:
  enabled=$START_GESTURE_RECOGNITION
  image_topic=$GESTURE_IMAGE_TOPIC
  result_topic=$GESTURE_TOPIC
  annotated_image_topic=$GESTURE_ANNOTATED_IMAGE_TOPIC

Web planning console:
  http://$PLANNING_CONSOLE_URL_HOST:$PLANNING_CONSOLE_PORT/
  Nav2 planner-only enabled: $ENABLE_PLANNER_ONLY
  Full Nav2 enabled: $ENABLE_NAV2
  Full Nav2 can be tested later with ENABLE_NAV2=true.

Save commands after the map looks good:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAP_PREFIX'}}"
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_PREFIX'}"

Helpful RViz displays:
  Map -> /map
  LaserScan -> /scan
  TF
  Path -> /zed/base_path or /zed/path
EOF
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

  if bool_is_true "$START_GESTURE_RECOGNITION"; then
    if ! grep -Fxq "$GESTURE_IMAGE_TOPIC" <<<"$available_topics"; then
      missing+=("$GESTURE_IMAGE_TOPIC")
    fi
  fi

  if (( ${#missing[@]} > 0 )); then
    printf '%s\n' "${missing[@]}"
  fi
}

wait_for_wrapper_topics() {
  local deadline=$((SECONDS + TOPIC_WAIT_TIMEOUT_SEC))
  local available_topics=""

  while (( SECONDS < deadline )); do
    available_topics="$(ros2 topic list 2>/dev/null || true)"
    mapfile -t missing < <(missing_topics "$available_topics")

    if (( ${#missing[@]} == 0 )); then
      return 0
    fi

    sleep 2
  done

  echo "Timed out waiting for required ZED wrapper topics." >&2
  echo "Still missing:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  return 1
}

cd "$WS_DIR"
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers "$NPROC_VALUE"
source_setup_file install/setup.bash

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "Expected setup file not found: $INSTALL_SETUP" >&2
  exit 1
fi

configure_nav2_launch
configure_planner_only_launch

if bool_is_true "$START_WRAPPER"; then
  run_terminal "zed wrapper" "$WRAPPER_LAUNCH"
  sleep 5
fi

echo "Waiting for ZED wrapper topics..."
wait_for_wrapper_topics
echo "Wrapper topics are available."

if bool_is_true "$START_GESTURE_RECOGNITION"; then
  gesture_launch_command="ros2 launch gesture_recognition gesture_recognition.launch.py"
  gesture_launch_command+=" $(launch_arg image_topic "$GESTURE_IMAGE_TOPIC")"
  gesture_launch_command+=" $(launch_arg image_is_compressed "$GESTURE_IMAGE_IS_COMPRESSED")"
  if [[ -n "${GESTURE_MODEL_PATH//[[:space:]]/}" ]]; then
    gesture_launch_command+=" $(launch_arg model_path "$GESTURE_MODEL_PATH")"
  fi
  gesture_launch_command+=" $(launch_arg show_window "$SHOW_GESTURE_WINDOW")"
  gesture_launch_command+=" $(launch_arg publish_annotated_image "$PUBLISH_GESTURE_ANNOTATED_IMAGE")"
  gesture_launch_command+=" $(launch_arg gesture_topic "$GESTURE_TOPIC")"
  gesture_launch_command+=" $(launch_arg annotated_image_topic "$GESTURE_ANNOTATED_IMAGE_TOPIC")"

  run_terminal \
    "mediapipe gesture" \
    "$gesture_launch_command"
fi

if bool_is_true "$START_RVIZ"; then
  run_terminal "rviz2" "$RVIZ_COMMAND"
fi

run_terminal \
  "handheld mapping" \
  "ros2 launch depth_processing zed_slam_nav.launch.py \
    slam_mode:='mapping' \
    enable_nav2:='${ENABLE_NAV2}' \
    enable_planner_only:='${ENABLE_PLANNER_ONLY}' \
    enable_planning_console:='${ENABLE_PLANNING_CONSOLE}' \
    planning_console_host:='${PLANNING_CONSOLE_HOST}' \
    planning_console_port:='${PLANNING_CONSOLE_PORT}' \
    enable_tracking_node:='false' \
    base_frame:='${BASE_FRAME}' \
    enable_base_adapter:='true' \
    base_to_camera_translation:='${BASE_TO_CAMERA_TRANSLATION}' \
    base_to_camera_rpy:='${BASE_TO_CAMERA_RPY}' \
    input_pose_topic:='${INPUT_POSE_TOPIC}' \
    input_pose_cov_topic:='${INPUT_POSE_COV_TOPIC}' \
    input_odom_topic:='${INPUT_ODOM_TOPIC}' \
    input_image_topic:='${INPUT_IMAGE_TOPIC}' \
    input_image_is_compressed:='${INPUT_IMAGE_IS_COMPRESSED}' \
    cloud_topic:='${CLOUD_TOPIC}' \
    enable_tracking_visualization:='${ENABLE_TRACKING_VISUALIZATION}' \
    show_tracking_window:='${SHOW_TRACKING_WINDOW}' \
    publish_tracking_image:='${PUBLISH_TRACKING_IMAGE}'"

run_terminal \
  "handheld mapping instructions" \
  "echo 'Handheld mapping session prefix:'; \
   echo '  $MAP_PREFIX'; \
   echo; \
   echo 'Web planning console:'; \
   echo '  http://$PLANNING_CONSOLE_URL_HOST:$PLANNING_CONSOLE_PORT/'; \
   echo '  Nav2 planner-only enabled: $ENABLE_PLANNER_ONLY'; \
   echo '  Full Nav2 enabled: $ENABLE_NAV2'; \
   echo '  Full Nav2 can be tested later with ENABLE_NAV2=true.'; \
   echo; \
   echo 'MediaPipe gesture recognition:'; \
   echo '  Result topic: $GESTURE_TOPIC'; \
   echo '  Annotated image topic: $GESTURE_ANNOTATED_IMAGE_TOPIC'; \
   echo; \
   echo 'Save commands:'; \
   echo \"ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\\"{name: {data: '$MAP_PREFIX'}}\\\"\"; \
   echo \"ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\\"{filename: '$MAP_PREFIX'}\\\"\"; \
   echo; \
   echo 'RViz displays to add:'; \
   echo '  Map -> /map'; \
   echo '  LaserScan -> /scan'; \
   echo '  TF'; \
   echo '  Path -> /zed/base_path or /zed/path'"

print_instructions
