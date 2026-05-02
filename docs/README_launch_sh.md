# Using `ros2_ws/launch.sh`

## Purpose

`ros2_ws/launch.sh` is the current one-stop handheld ZED mapping launcher.

The previous automatic mapping-to-localization launcher has been preserved as
`ros2_ws/launch_old.sh`.

The current launcher is designed around this sequence:

1. Build the workspace.
2. Start the ZED wrapper.
3. Wait for the required ZED topics.
4. Start the MediaPipe gesture recognition node.
5. Launch the handheld mapping stack.
6. Open an instructions terminal with the save commands for the current run.

## What it launches

By default the script launches these pieces in separate terminals:

- the ZED wrapper
- `gesture_recognition gesture_recognition.launch.py`
- `depth_processing zed_slam_nav.launch.py` in `mapping` mode
- the web planning console
- an instructions terminal

The old TensorRT hand tracker is no longer started by this launcher.

## What the script waits for

The script waits until these ZED wrapper topics are available:

- `/zed/zed_node/pose`
- `/zed/zed_node/odom`
- `/zed/zed_node/point_cloud/cloud_registered`

When `START_GESTURE_RECOGNITION=true`, it also waits for:

- `/zed/zed_node/rgb/color/rect/image/compressed`

This keeps MediaPipe gesture recognition from launching before the ZED image
feed exists.

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
- `WRAPPER_LAUNCH`
  Full override for the wrapper launch command.
- `TOPIC_WAIT_TIMEOUT_SEC`
  Default: `60`

### MediaPipe gesture recognition

- `START_GESTURE_RECOGNITION`
  Default: `true`
- `GESTURE_IMAGE_TOPIC`
  Default: same as `INPUT_IMAGE_TOPIC`
- `GESTURE_IMAGE_IS_COMPRESSED`
  Default: same as `INPUT_IMAGE_IS_COMPRESSED`
- `GESTURE_MODEL_PATH`
  Optional explicit `.task` model path.
- `SHOW_GESTURE_WINDOW`
  Default: `false`
- `PUBLISH_GESTURE_ANNOTATED_IMAGE`
  Default: `true`
- `GESTURE_TOPIC`
  Default: `/gesture_recognition/result`
- `GESTURE_ANNOTATED_IMAGE_TOPIC`
  Default: `/gesture_recognition/annotated_image/compressed`

### Planning and visualization

- `ENABLE_PLANNER_ONLY`
  Default: `true`
- `ENABLE_PLANNING_CONSOLE`
  Default: `true`
- `PLANNING_CONSOLE_HOST`
  Default: `127.0.0.1`
- `PLANNING_CONSOLE_PORT`
  Default: `8080`
- `START_RVIZ=true`
  Opens `rviz2` in another terminal.

## Old Launcher

Use `./launch_old.sh` if you need the older robot-mounted workflow that starts
the old hand/person tracking terminals, waits for saved `.posegraph` and `.data`
files, and then switches into localization mode.
