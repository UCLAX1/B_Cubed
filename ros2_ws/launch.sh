#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"
REPO_DIR="$(cd -- "$WS_DIR/.." && pwd)"
INSTALL_SETUP="$WS_DIR/install/local_setup.bash"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is required for ros2_ws/launch.sh" >&2
  exit 1
fi

NPROC_VALUE="$(command -v nproc >/dev/null 2>&1 && nproc || echo 4)"
SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"

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
LAUNCH_MEMORY_LIMIT_DISPLAY="disabled"

MAP_OUTPUT_DIR="${MAP_OUTPUT_DIR:-$WS_DIR/maps}"
MAP_SESSION_NAME="${MAP_SESSION_NAME:-handheld_${SESSION_STAMP}}"
MAP_PREFIX="${MAP_PREFIX:-$MAP_OUTPUT_DIR/$MAP_SESSION_NAME}"

BASE_TO_CAMERA_TRANSLATION="${BASE_TO_CAMERA_TRANSLATION:-0.0,0.0,0.0}"
BASE_TO_CAMERA_RPY="${BASE_TO_CAMERA_RPY:-0.0,0.0,0.0}"
BASE_FRAME="${BASE_FRAME:-zed_camera_link}"

CAMERA_MODEL="${CAMERA_MODEL:-zedm}"
START_WRAPPER="${START_WRAPPER:-true}"
ZED_GRAB_RESOLUTION="${ZED_GRAB_RESOLUTION:-VGA}"
ZED_GRAB_FRAME_RATE="${ZED_GRAB_FRAME_RATE:-15}"
ZED_PARAM_OVERRIDES="${ZED_PARAM_OVERRIDES:-general.grab_resolution:=${ZED_GRAB_RESOLUTION};general.grab_frame_rate:=${ZED_GRAB_FRAME_RATE};pos_tracking.pos_tracking_enabled:=true;pos_tracking.area_memory:=true;pos_tracking.two_d_mode:=true;debug.use_pub_timestamps:=true}"
WRAPPER_LAUNCH="${WRAPPER_LAUNCH:-ros2 launch zed_wrapper zed_camera.launch.py camera_model:=${CAMERA_MODEL} publish_tf:=false publish_map_tf:=false param_overrides:='${ZED_PARAM_OVERRIDES}'}"

INPUT_POSE_TOPIC="${INPUT_POSE_TOPIC:-/zed/zed_node/pose}"
INPUT_POSE_COV_TOPIC="${INPUT_POSE_COV_TOPIC:-/zed/zed_node/pose_with_covariance}"
INPUT_ODOM_TOPIC="${INPUT_ODOM_TOPIC:-/zed/zed_node/odom}"
INPUT_IMAGE_TOPIC="${INPUT_IMAGE_TOPIC:-/zed/zed_node/rgb/color/rect/image/compressed}"
INPUT_IMAGE_IS_COMPRESSED="${INPUT_IMAGE_IS_COMPRESSED:-true}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/zed/zed_node/point_cloud/cloud_registered}"
REQUIRE_POSE_COV_TOPIC="${REQUIRE_POSE_COV_TOPIC:-false}"

START_GESTURE_RECOGNITION="${START_GESTURE_RECOGNITION:-false}"
GESTURE_IMAGE_TOPIC="${GESTURE_IMAGE_TOPIC:-$INPUT_IMAGE_TOPIC}"
GESTURE_IMAGE_IS_COMPRESSED="${GESTURE_IMAGE_IS_COMPRESSED:-$INPUT_IMAGE_IS_COMPRESSED}"
GESTURE_MODEL_PATH="${GESTURE_MODEL_PATH:-}"
SHOW_GESTURE_WINDOW="${SHOW_GESTURE_WINDOW:-false}"
PUBLISH_GESTURE_ANNOTATED_IMAGE="${PUBLISH_GESTURE_ANNOTATED_IMAGE:-false}"
GESTURE_TOPIC="${GESTURE_TOPIC:-/gesture_recognition/result}"
GESTURE_ANNOTATED_IMAGE_TOPIC="${GESTURE_ANNOTATED_IMAGE_TOPIC:-/gesture_recognition/annotated_image/compressed}"

START_PERSON_TRACKING="${START_PERSON_TRACKING:-false}"
PERSON_TRACKING_IMAGE_TOPIC="${PERSON_TRACKING_IMAGE_TOPIC:-$INPUT_IMAGE_TOPIC}"
PERSON_TRACKING_ENGINE_PATH="${PERSON_TRACKING_ENGINE_PATH:-$REPO_DIR/models/yolo11n.engine}"
SHOW_PERSON_TRACKING_WINDOW="${SHOW_PERSON_TRACKING_WINDOW:-false}"
PUBLISH_PERSON_TRACKING_IMAGE="${PUBLISH_PERSON_TRACKING_IMAGE:-false}"
PERSON_TRACKING_TOPIC="${PERSON_TRACKING_TOPIC:-/person_tracking/detections}"
PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC="${PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC:-/person_tracking/annotated_image/compressed}"
PERSON_TRACKING_CONFIDENCE_THRESHOLD="${PERSON_TRACKING_CONFIDENCE_THRESHOLD:-0.4}"
PERSON_TRACKING_NMS_THRESHOLD="${PERSON_TRACKING_NMS_THRESHOLD:-0.45}"

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
  local command_with_shell
  local quoted_command

  command_with_shell="${command}
exec bash"
  quoted_command="$(printf '%q' "$command_with_shell")"

  gnome-terminal --title="$title" -- bash -lc "
    source '$INSTALL_SETUP'
    export OMP_NUM_THREADS='$LAUNCH_THREAD_LIMIT'
    export OPENBLAS_NUM_THREADS='$LAUNCH_THREAD_LIMIT'
    export MKL_NUM_THREADS='$LAUNCH_THREAD_LIMIT'
    export NUMEXPR_NUM_THREADS='$LAUNCH_THREAD_LIMIT'
    export OPENCV_FOR_THREADS_NUM='$LAUNCH_THREAD_LIMIT'
    export VECLIB_MAXIMUM_THREADS='$LAUNCH_THREAD_LIMIT'
    export MALLOC_ARENA_MAX=2
    export CUDA_MODULE_LOADING=LAZY
    if [[ '$LAUNCH_MEMORY_LIMIT_KB' != '0' ]]; then
      ulimit -v '$LAUNCH_MEMORY_LIMIT_KB'
    fi
    exec ionice -c 2 -n '$LAUNCH_IONICE_PRIORITY' \
      nice -n '$LAUNCH_NICE' \
      taskset -c '$LAUNCH_CPU_SET' \
      bash -lc $quoted_command
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
  if (( LAUNCH_MEMORY_LIMIT_MB > 0 )); then
    LAUNCH_MEMORY_LIMIT_DISPLAY="${LAUNCH_MEMORY_LIMIT_MB} MB"
  else
    LAUNCH_MEMORY_LIMIT_DISPLAY="disabled"
  fi
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

ZED wrapper:
  camera_model=$CAMERA_MODEL
  publish_tf=false
  publish_map_tf=false
  param_overrides=$ZED_PARAM_OVERRIDES

Resource limits:
  build_parallel_workers=$BUILD_PARALLEL_WORKERS
  launch_cpu_set=$LAUNCH_CPU_SET
  launch_thread_limit=$LAUNCH_THREAD_LIMIT
  launch_nice=$LAUNCH_NICE
  launch_ionice_priority=$LAUNCH_IONICE_PRIORITY
  launch_memory_limit=$LAUNCH_MEMORY_LIMIT_DISPLAY

Optional upstream wrapper topics:
  $INPUT_POSE_COV_TOPIC
  $INPUT_IMAGE_TOPIC

MediaPipe gesture recognition:
  enabled=$START_GESTURE_RECOGNITION
  image_topic=$GESTURE_IMAGE_TOPIC
  result_topic=$GESTURE_TOPIC
  annotated_image_topic=$GESTURE_ANNOTATED_IMAGE_TOPIC

Person tracking:
  enabled=$START_PERSON_TRACKING
  image_topic=$PERSON_TRACKING_IMAGE_TOPIC
  result_topic=$PERSON_TRACKING_TOPIC
  annotated_image_topic=$PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC

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

  if bool_is_true "$START_PERSON_TRACKING"; then
    if ! grep -Fxq "$PERSON_TRACKING_IMAGE_TOPIC" <<<"$available_topics"; then
      missing+=("$PERSON_TRACKING_IMAGE_TOPIC")
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

configure_resource_limits

cd "$WS_DIR"
run_limited colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers "$BUILD_PARALLEL_WORKERS"
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

if bool_is_true "$START_PERSON_TRACKING"; then
  person_tracking_launch_command="ros2 launch person_tracking person_tracking.launch.py"
  person_tracking_launch_command+=" $(launch_arg image_topic "$PERSON_TRACKING_IMAGE_TOPIC")"
  person_tracking_launch_command+=" $(launch_arg engine_path "$PERSON_TRACKING_ENGINE_PATH")"
  person_tracking_launch_command+=" $(launch_arg show_window "$SHOW_PERSON_TRACKING_WINDOW")"
  person_tracking_launch_command+=" $(launch_arg publish_annotated_image "$PUBLISH_PERSON_TRACKING_IMAGE")"
  person_tracking_launch_command+=" $(launch_arg detection_topic "$PERSON_TRACKING_TOPIC")"
  person_tracking_launch_command+=" $(launch_arg annotated_image_topic "$PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC")"
  person_tracking_launch_command+=" $(launch_arg confidence_threshold "$PERSON_TRACKING_CONFIDENCE_THRESHOLD")"
  person_tracking_launch_command+=" $(launch_arg nms_threshold "$PERSON_TRACKING_NMS_THRESHOLD")"

  run_terminal \
    "person tracking" \
    "$person_tracking_launch_command"
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
    enable_tracking_node:='true' \
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
   echo 'Person tracking:'; \
   echo '  Result topic: $PERSON_TRACKING_TOPIC'; \
   echo '  Annotated image topic: $PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC'; \
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
