# Handheld Mapping Test

This is the fastest way to test the mapping stack before the full robot is
ready.

The idea is simple:

- run the ZED wrapper
- treat the camera as the whole robot
- keep the base-to-camera offset at zero
- walk the camera slowly through a loop
- watch the occupancy map update

## Helper launcher

Use:

```bash
cd ~/Documents/B_Cubed/ros2_ws
./launch_handheld_mapping.sh
```

That script lives at
[launch_handheld_mapping.sh](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/launch_handheld_mapping.sh)
and it does four things:

1. builds the workspace
2. starts the ZED wrapper in a new terminal by default
3. waits until the required wrapper topics appear
4. launches `zed_slam_nav.launch.py` in `mapping` mode with:
   - `enable_nav2:=false`
   - `base_to_camera_translation:=0.0,0.0,0.0`
   - `base_to_camera_rpy:=0.0,0.0,0.0`

It also opens a small instructions terminal with the exact save commands for the
current run.

## Why the zero offset matters

The robot stack now defaults to a `0.381 m` ZED height because that matches the
planned robot mount.

For a handheld test, that is the wrong assumption. The camera is the thing being
moved, so the helper launcher overrides the stack back to:

- `base_to_camera_translation=0.0,0.0,0.0`

That makes `base_link` effectively sit on the camera for the test.

## Required upstream wrapper topics

The helper script waits for these topics before launching mapping:

- `/zed/zed_node/pose`
- `/zed/zed_node/odom`
- `/zed/zed_node/point_cloud/cloud_registered`

It does not require `pose_with_covariance` by default, because
`zed_tracking` can synthesize a covariance message from the plain pose stream
when needed.

The compressed image topic is also treated as optional for launch readiness.
It is still useful for the localization overlay, but the mapping stack itself
does not need it to start.

If your wrapper uses different topic names, export overrides before running the
script. Example:

```bash
export INPUT_POSE_TOPIC="/my_zed/pose"
export INPUT_POSE_COV_TOPIC="/my_zed/pose_with_covariance"
export INPUT_ODOM_TOPIC="/my_zed/odom"
export INPUT_IMAGE_TOPIC="/my_zed/image/compressed"
export CLOUD_TOPIC="/my_zed/cloud"
./launch_handheld_mapping.sh
```

## Optional environment overrides

- `START_WRAPPER=false`
  Use this if the wrapper is already running.
- `TOPIC_WAIT_TIMEOUT_SEC=120`
  Increase this if your wrapper takes a while to come up.
- `REQUIRE_POSE_COV_TOPIC=true`
  Only use this if you specifically want the launcher to wait for a real
  covariance topic instead of allowing `zed_tracking` to synthesize one.
- `START_RVIZ=true`
  Opens a plain `rviz2` window in another terminal.
- `RVIZ_COMMAND="rviz2"`
  Lets you replace the RViz command if you prefer a custom one.

## What to look at in RViz

Set `Fixed Frame` to `map` and add:

- `Map` on `/map`
- `LaserScan` on `/scan`
- `TF`
- `Path` on `/zed/base_path` or `/zed/path`

## How to move the camera

Move it like a very careful ground robot:

- keep the camera roughly level
- keep the height roughly constant
- walk slowly
- make a loop, square, or rectangle
- revisit part of the path before saving
- avoid large pitch and roll changes

The more robot-like the motion is, the better the 2D map will look.

## Saving the map

When the map looks good, run the printed save commands. They will write:

- an occupancy map (`.yaml` and usually `.pgm`)
- a serialized SLAM pose graph (`.posegraph` and `.data`)

That saved pose graph can then be used for a follow-up localization test.
