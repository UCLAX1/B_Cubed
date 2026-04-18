# Using `ros2_ws/launch.sh`

## Purpose

[ros2_ws/launch.sh](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/launch.sh) is the current one-stop launcher for the Jetson workflow.

It is designed around this sequence:

1. Build the workspace.
2. Start the ZED wrapper and helper nodes.
3. Launch the mapping pass.
4. Wait for the current run's saved mapping artifacts.
5. Automatically switch into localization mode.
6. Optionally start a navigation stack command if one is provided.

## What it launches

By default the script can launch these pieces in separate terminals:

- the ZED wrapper
- `depth_processing zed_tracking`
- `person_tracking hands`
- `person_tracking person`
- the mapping pass launch
- an instructions terminal that prints the exact save commands to run after mapping

The depth viewer is currently optional and controlled by an environment variable.

## What the script watches for

The script creates a unique per-run map prefix, then waits for:

- `<prefix>.posegraph`
- `<prefix>.data`

Once both files exist and are newer than the launcher start time, the script:

1. stops the mapping pass
2. launches localization mode using that prefix
3. optionally launches whatever command is stored in `NAVIGATION_AUTOSTART_COMMAND`

This avoids a false handoff caused by stale files from older sessions.

## Basic usage

From the Jetson:

```bash
cd ~/Documents/B_Cubed/ros2_ws
./launch.sh
```

During the mapping pass, the script prints a per-run map prefix. After the robot completes its square or loop pass, run the printed save commands in a sourced ROS terminal.

Example:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/path/to/prefix'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/path/to/prefix'}"
```

Once the serialized files are written, `launch.sh` automatically advances to localization mode.

## Important environment variables

You can adjust the launcher behavior without editing the script by exporting environment variables before running it.

### Mapping output

- `MAP_OUTPUT_DIR`
  Example: `/home/jetson-nano-x1/Documents/B_Cubed/maps`
- `MAP_SESSION_NAME`
  Example: `lab_a`
- `MAP_PREFIX`
  If you set this directly, it overrides the default prefix logic.

### Robot geometry

- `BASE_FRAME`
  Default: `base_link`
- `BASE_TO_CAMERA_TRANSLATION`
  Default: `0.0,0.0,0.381`
  Format: `x,y,z`
- `BASE_TO_CAMERA_RPY`
  Format: `roll,pitch,yaw`

These are passed through to the mapping and localization launches so the robot-base pose can be derived from the ZED camera pose.

### Wrapper settings

- `CAMERA_MODEL`
  Default: `zedm`
- `WRAPPER_LAUNCH`
  Full override for the wrapper launch command

### Optional helper terminals

- `START_HAND_TRACKING`
- `START_PERSON_TRACKING`
- `START_DEPTH_VIEWER`
- `START_ZED_TRACKING_VIEWER`

These accept values like `true`, `false`, `1`, `0`, `yes`, or `on`.

### Navigation handoff

- `NAVIGATION_AUTOSTART_COMMAND`

If this is set, `launch.sh` will start that command in a new terminal after localization mode is launched.

Example:

```bash
export NAVIGATION_AUTOSTART_COMMAND="ros2 launch depth_processing zed_nav2_bringup.launch.py"
./launch.sh
```

If it is not set, the script opens a reminder terminal instead of guessing how your navigation stack should start.

## Example setup

```bash
export MAP_SESSION_NAME="warehouse_bay_1"
export MAP_OUTPUT_DIR="/home/jetson-nano-x1/Documents/B_Cubed/maps"
export BASE_TO_CAMERA_TRANSLATION="0.0,0.0,0.381"
export BASE_TO_CAMERA_RPY="0.0,0.0,0.0"
export NAVIGATION_AUTOSTART_COMMAND="ros2 launch depth_processing zed_nav2_bringup.launch.py"

cd ~/Documents/B_Cubed/ros2_ws
./launch.sh
```

## What "automatic launch" means right now

The script does not currently decide on its own when the mapping pass is good enough.

The automatic transition happens only after you explicitly save the mapping outputs. That means the workflow is still operator-controlled at the moment:

- you decide when the mapping pass is complete
- you run the save calls
- the script sees the new serialized files and performs the handoff

## Common failure modes

### The script never switches out of mapping mode

Most likely causes:

- the save services were not called
- the save prefix used in the service calls does not match the prefix printed by `launch.sh`
- only the occupancy map was saved, but the serialized pose graph was not saved

The handoff requires the serialized `.posegraph` and `.data` files for the current run.

### Localization mode starts but fails to localize well

Most likely causes:

- the square pass did not create a clean map
- the base-to-camera transform is wrong
- the point-cloud-to-scan parameters need tuning for camera height and robot geometry

### The full navigation stack does not start

Most likely cause:

- `NAVIGATION_AUTOSTART_COMMAND` has not been set yet

Recommended value:

- `ros2 launch depth_processing zed_nav2_bringup.launch.py`
