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
MAP_SESSION_NAME="${MAP_SESSION_NAME:-mapping_${SESSION_STAMP}}"
MAP_PREFIX="${MAP_PREFIX:-$MAP_OUTPUT_DIR/$MAP_SESSION_NAME}"
MAP_YAML_FILE="${MAP_PREFIX}.yaml"
MAP_POSEGRAPH_FILE="${MAP_PREFIX}.posegraph"
MAP_DATA_FILE="${MAP_PREFIX}.data"

BASE_TO_CAMERA_TRANSLATION="${BASE_TO_CAMERA_TRANSLATION:-0.0,0.0,0.0}"
BASE_TO_CAMERA_RPY="${BASE_TO_CAMERA_RPY:-0.0,0.0,0.0}"
BASE_FRAME="${BASE_FRAME:-base_link}"

CAMERA_MODEL="${CAMERA_MODEL:-zedm}"
WRAPPER_LAUNCH="${WRAPPER_LAUNCH:-ros2 launch zed_wrapper zed_camera.launch.py camera_model:=${CAMERA_MODEL}}"
NAVIGATION_AUTOSTART_COMMAND="${NAVIGATION_AUTOSTART_COMMAND:-}"

START_HAND_TRACKING="${START_HAND_TRACKING:-true}"
START_PERSON_TRACKING="${START_PERSON_TRACKING:-true}"
START_DEPTH_VIEWER="${START_DEPTH_VIEWER:-false}"
START_ZED_TRACKING_VIEWER="${START_ZED_TRACKING_VIEWER:-true}"

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

bool_is_true() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

file_is_fresh() {
  local file_path="$1"
  [[ -f "$file_path" ]] && [[ "$(stat -c %Y "$file_path")" -ge "$LAUNCH_EPOCH" ]]
}

stop_mapping_stack() {
  pkill -f "zed_mapping_pass.launch.py" >/dev/null 2>&1 || true
  pkill -f "async_slam_toolbox_node" >/dev/null 2>&1 || true
}

launch_navigation_mode() {
  echo
  echo "Mapping artifacts detected for $MAP_PREFIX"
  echo "Stopping mapping pass and switching into post-mapping mode..."

  stop_mapping_stack
  sleep 3

  run_terminal \
    "localization mode" \
    "ros2 launch depth_processing zed_localization_mode.launch.py \
      map_file_name:='$MAP_PREFIX' \
      base_frame:='${BASE_FRAME}' \
      base_to_camera_translation:='${BASE_TO_CAMERA_TRANSLATION}' \
      base_to_camera_rpy:='${BASE_TO_CAMERA_RPY}'"

  if [[ -n "$NAVIGATION_AUTOSTART_COMMAND" ]]; then
    run_terminal "navigation stack" "$NAVIGATION_AUTOSTART_COMMAND"
  else
    run_terminal \
      "navigation handoff" \
      "echo 'Localization mode is running with serialized map prefix:'; \
       echo '  $MAP_PREFIX'; \
       echo; \
       echo 'Set NAVIGATION_AUTOSTART_COMMAND in ros2_ws/launch.sh or your shell to auto-run the full navigation stack here.'"
  fi

  echo "Post-mapping mode launched."
}

print_mapping_instructions() {
  cat <<EOF

Mapping session prefix:
  $MAP_PREFIX

This launcher will automatically switch to post-mapping mode when both of these files
from the current run exist:
  $MAP_POSEGRAPH_FILE
  $MAP_DATA_FILE

Recommended save commands after the square mapping pass is complete:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAP_PREFIX'}}"
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_PREFIX'}"

The saved occupancy map (.yaml/.pgm) is useful for inspection.
The serialized slam session (.posegraph/.data) is what the localization mode reloads.
EOF
}

trap 'echo; echo "Stopping mapping stack monitor."; stop_mapping_stack' INT TERM

cd "$WS_DIR"
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers "$NPROC_VALUE"
source install/setup.bash

if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "Expected setup file not found: $INSTALL_SETUP" >&2
  exit 1
fi

LAUNCH_EPOCH="$(date +%s)"

run_terminal "zed wrapper" "$WRAPPER_LAUNCH"
sleep 5

if bool_is_true "$START_ZED_TRACKING_VIEWER"; then
  run_terminal "zed tracking" "ros2 run depth_processing zed_tracking"
fi

if bool_is_true "$START_DEPTH_VIEWER"; then
  run_terminal "depth processing" "ros2 run depth_processing depth"
fi

if bool_is_true "$START_HAND_TRACKING"; then
  run_terminal "hand tracking" "ros2 run person_tracking hands"
fi

if bool_is_true "$START_PERSON_TRACKING"; then
  run_terminal "person tracking" "ros2 run person_tracking person"
fi

run_terminal \
  "mapping pass" \
  "ros2 launch depth_processing zed_mapping_pass.launch.py \
    base_frame:='${BASE_FRAME}' \
    base_to_camera_translation:='${BASE_TO_CAMERA_TRANSLATION}' \
    base_to_camera_rpy:='${BASE_TO_CAMERA_RPY}'"

run_terminal \
  "mapping instructions" \
  "echo 'Save map prefix:'; \
   echo '  $MAP_PREFIX'; \
   echo; \
   echo 'Run these after the mapping pass:'; \
   echo \"ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\\"{name: {data: '$MAP_PREFIX'}}\\\"\"; \
   echo \"ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\\"{filename: '$MAP_PREFIX'}\\\"\""

print_mapping_instructions
echo
echo "Waiting for mapping artifacts from this run..."

while true; do
  if file_is_fresh "$MAP_POSEGRAPH_FILE" && file_is_fresh "$MAP_DATA_FILE"; then
    launch_navigation_mode
    break
  fi
  sleep 2
done
