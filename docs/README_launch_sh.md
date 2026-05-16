# Using `ros2_ws/launch.sh`

## Purpose

`ros2_ws/launch.sh` is the current one-stop handheld ZED mapping launcher.

The previous automatic mapping-to-localization launcher has been preserved as
`ros2_ws/launch_old.sh`.

The current launcher is designed around this sequence:

1. Build the workspace.
2. Start the ZED wrapper.
3. Wait for the required ZED topics.
4. Launch the handheld mapping stack.
5. Open an instructions terminal with the save commands for the current run.

Gesture recognition and TensorRT person tracking are available as opt-in launch
features. They are disabled by default to keep the Jetson from running out of
CPU or memory during mapping.

## What it launches

By default the script launches these pieces in separate terminals:

- the ZED wrapper
- `depth_processing zed_slam_nav.launch.py` in `mapping` mode
- planner-only Nav2 inside `depth_processing zed_slam_nav.launch.py`
- the standalone `nav_planning_console` web planning console
- an instructions terminal

Optional terminals can also be enabled for:

- `gesture_recognition gesture_recognition.launch.py`
- `person_tracking person_tracking.launch.py`

The old TensorRT hand tracker is no longer started by this launcher.

## What the script waits for

The script waits until these ZED wrapper topics are available:

- `/zed/zed_node/pose`
- `/zed/zed_node/odom`
- `/zed/zed_node/point_cloud/cloud_registered`

When `START_GESTURE_RECOGNITION=true`, it also waits for:

- `/zed/zed_node/rgb/color/rect/image/compressed`

When `START_PERSON_TRACKING=true`, it waits for:

- `/zed/zed_node/rgb/color/rect/image/compressed`

This keeps MediaPipe gesture recognition from launching before the ZED image
feed exists, and does the same for the person detector.

## Basic usage

From the Jetson:

```bash
cd ~/Documents/B_Cubed/ros2_ws
./launch.sh
```

The script prints a per-run map prefix. When the map looks good, run the printed
save commands in a sourced ROS terminal.

## Important environment variables

You can adjust launcher behavior by exporting environment variables before
running it.

### Mapping output

- `MAP_OUTPUT_DIR`
  Example: `/home/jetson-nano-x1/Documents/B_Cubed/ros2_ws/maps`
- `MAP_SESSION_NAME`
  Example: `lab_a`
- `MAP_PREFIX`
  If set directly, this overrides the default prefix logic.

### Handheld geometry

- `BASE_FRAME`
  Default: `zed_camera_link`
- `BASE_TO_CAMERA_TRANSLATION`
  Default: `0.0,0.0,0.0`
- `BASE_TO_CAMERA_RPY`
  Default: `0.0,0.0,0.0`

### Wrapper settings

- `START_WRAPPER=false`
  Use this if the ZED wrapper is already running.
- `CAMERA_MODEL`
  Default: `zedm`
- `ZED_PARAM_OVERRIDES`
  Default:
  `general.grab_resolution:=VGA;general.grab_frame_rate:=15;pos_tracking.pos_tracking_enabled:=true;pos_tracking.area_memory:=true;pos_tracking.two_d_mode:=true;debug.use_pub_timestamps:=true`
- `WRAPPER_LAUNCH`
  Full override for the wrapper launch command.
- `TOPIC_WAIT_TIMEOUT_SEC`
  Default: `60`

### Resource limits

- `BUILD_PARALLEL_WORKERS`
  Default: up to `4`, capped by detected CPU count.
- `LAUNCH_MAX_CORES`
  Default: up to `4`, capped by detected CPU count.
- `LAUNCH_CPU_SET`
  Default: derived from `LAUNCH_MAX_CORES`, for example `0-3`
- `LAUNCH_THREAD_LIMIT`
  Default: `2`
- `LAUNCH_NICE`
  Default: `10`
- `LAUNCH_IONICE_PRIORITY`
  Default: `7`
- `LAUNCH_MEMORY_LIMIT_MB`
  Default: `0`, disabled. Use with care because CUDA/ZED/TensorRT can reserve
  large virtual address ranges.

### MediaPipe gesture recognition

- `START_GESTURE_RECOGNITION`
  Default: `false`
- `GESTURE_IMAGE_TOPIC`
  Default: same as `INPUT_IMAGE_TOPIC`
- `GESTURE_IMAGE_IS_COMPRESSED`
  Default: same as `INPUT_IMAGE_IS_COMPRESSED`
- `GESTURE_MODEL_PATH`
  Optional explicit `.task` model path.
- `SHOW_GESTURE_WINDOW`
  Default: `false`
- `PUBLISH_GESTURE_ANNOTATED_IMAGE`
  Default: `false`
- `GESTURE_TOPIC`
  Default: `/gesture_recognition/result`
- `GESTURE_ANNOTATED_IMAGE_TOPIC`
  Default: `/gesture_recognition/annotated_image/compressed`

### Person tracking

- `START_PERSON_TRACKING`
  Default: `false`
- `PERSON_TRACKING_IMAGE_TOPIC`
  Default: same as `INPUT_IMAGE_TOPIC`
- `PERSON_TRACKING_ENGINE_PATH`
  Default: `../models/yolo11n.engine` relative to `ros2_ws`
- `SHOW_PERSON_TRACKING_WINDOW`
  Default: `false`
- `PUBLISH_PERSON_TRACKING_IMAGE`
  Default: `false`
- `PERSON_TRACKING_TOPIC`
  Default: `/person_tracking/detections`
- `PERSON_TRACKING_ANNOTATED_IMAGE_TOPIC`
  Default: `/person_tracking/annotated_image/compressed`

### Planning and visualization

- `ENABLE_PLANNER_ONLY`
  Default: `true`
- `ENABLE_PLANNING_CONSOLE`
  Default: `true`
- `MANUAL_CMD_TOPIC`
  Default: `cmd_vel_manual`
- `PLANNING_CONSOLE_HOST`
  Default: `127.0.0.1`
- `PLANNING_CONSOLE_PORT`
  Default: `8080`

### Kiwi drive controller

- `START_KIWI_DRIVE_CONTROLLER`
  Default: `false`
- `KIWI_HARDWARE_ENABLED`
  Default: `true`
- `KIWI_NAV_CMD_TOPIC`
  Default: `cmd_vel`

The console can also be launched independently:

```bash
ros2 launch nav_planning_console nav_planning_console.launch.py
```
- `START_RVIZ=true`
  Opens `rviz2` in another terminal.

## Old Launcher

Use `./launch_old.sh` if you need the older robot-mounted workflow that starts
the old hand/person tracking terminals, waits for saved `.posegraph` and `.data`
files, and then switches into localization mode.
