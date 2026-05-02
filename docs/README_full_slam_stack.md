# Full SLAM Stack

This document explains the new end-to-end SLAM and navigation pieces that sit on
top of the existing ZED and `slam_toolbox` groundwork.

## What is now in the repo

There are two new launch paths for the autonomous stack:

- [zed_nav2_bringup.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_nav2_bringup.launch.py)
- [zed_slam_nav.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_slam_nav.launch.py)

There are also two new config files:

- [nav2_zed_slam.yaml](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/config/nav2_zed_slam.yaml)
- [twist_safety_gate.yaml](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/config/twist_safety_gate.yaml)

And one new helper node:

- [twist_safety_gate.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/depth_processing/twist_safety_gate.py)

## Default robot assumption

The stack now assumes the ZED is rigidly mounted `0.381 m` above the chassis
origin. That is the 15-inch vertical offset you asked for.

That default is now baked into:

- `zed_base_adapter`
- `zed_nav_bringup.launch.py`
- `zed_mapping_pass.launch.py`
- `zed_localization_mode.launch.py`
- `ros2_ws/launch_old.sh`

If the real measured offset is slightly different, update
`base_to_camera_translation`.

The current handheld `ros2_ws/launch.sh` overrides this to
`0.0,0.0,0.0`, because the camera is treated as the whole moving body during a
handheld test.

## TF ownership rule

When `slam_toolbox` mapping or localization is running, it should be the only
node publishing `map -> odom`.

That means:

- do not leave the ZED wrapper publishing a competing global map transform
- leave `zed_base_adapter` in its default `publish_map_to_odom_tf:=false` mode
- let `slam_toolbox` own the global frame during mapping or localization

## Launch 1: `zed_nav2_bringup.launch.py`

Use this when the SLAM or localization layer is already running and you only
need the navigation stack on top.

It starts:

- `planner_server`
- `controller_server`
- `behavior_server`
- `bt_navigator`
- `lifecycle_manager_navigation`
- `twist_safety_gate`

What it expects to already exist:

- `map`
- `scan`
- `map -> odom -> base_link`
- `zed/is_localized`

This is the best companion launch for `ros2_ws/launch_old.sh`, because that
preserved launcher brings up mapping and localization mode.

Recommended use after `launch_old.sh` switches into localization mode:

```bash
export NAVIGATION_AUTOSTART_COMMAND="ros2 launch depth_processing zed_nav2_bringup.launch.py"
./launch_old.sh
```

## Launch 2: `zed_slam_nav.launch.py`

Use this when you want one command that starts:

- `zed_tracking`
- the SLAM layer
- Nav2
- the `cmd_vel` safety gate

This launch does not start the ZED wrapper itself.

You still need the wrapper running first so these upstream topics exist:

- `/zed/zed_node/pose`
- `/zed/zed_node/pose_with_covariance`
- `/zed/zed_node/odom`
- `/zed/zed_node/rgb/color/rect/image/compressed`
- `/zed/zed_node/point_cloud/cloud_registered`

If your wrapper publishes under different topic names, `zed_slam_nav.launch.py`
now exposes launch arguments for the tracking input topics.

It supports two modes.

### Mapping mode

```bash
ros2 launch depth_processing zed_slam_nav.launch.py slam_mode:=mapping
```

This mode includes `zed_mapping_pass.launch.py`.

That means:

- the occupancy map keeps updating
- `slam_toolbox` owns the live map
- Nav2 plans against the evolving map

This is the mode to use if you specifically want a continuously updating global
occupancy map while the full stack is running.

### Localization mode

```bash
ros2 launch depth_processing zed_slam_nav.launch.py \
  slam_mode:=localization \
  map_file_name:=/absolute/path/to/posegraph_prefix
```

This mode includes `zed_localization_mode.launch.py`.

That means:

- `slam_toolbox` reloads a saved pose graph
- the robot localizes in the saved environment
- the global map is reused instead of being rebuilt the same way

This is the more stable mode once an environment has already been mapped.

## What the safety gate does

`twist_safety_gate` sits between Nav2 and the base drive topic.

By default it:

- listens for navigation output on `cmd_vel_nav`
- publishes the gated command on `cmd_vel`
- requires `zed/is_localized` to be fresh and true
- requires `/scan` to be fresh
- publishes zero velocity when the gate closes

It also publishes a plain-text status topic:

- `nav/cmd_vel_gate_status`

This is intentionally simple: the robot should not keep moving if localization
or scan generation has gone stale.

## How Nav2 is configured

[nav2_zed_slam.yaml](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/config/nav2_zed_slam.yaml)
contains the initial navigation tuning for this robot.

Current decisions in that file:

- `NavfnPlanner` for global planning
- `RegulatedPurePursuitController` for path following
- a conservative `robot_radius` of `0.35 m`
- `scan` as the obstacle source for local and global costmaps
- the live `map` topic as the global static layer input

Those values are intended to be safe starting points, not final field tuning.

## Important boundary

The repo now brings the full software stack up to `/cmd_vel`.

What it still does not contain is the robot-specific hardware drive node that
consumes `/cmd_vel` and turns it into motor motion. If your base controller
already subscribes to `/cmd_vel`, this stack can sit directly on top of it. If
not, that final hardware interface still has to exist outside this repo.

## Camera-only testing

For a handheld camera test before the full robot is ready, use
`ros2_ws/launch.sh`
and the companion notes in
[README_handheld_mapping_test.md](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/docs/README_handheld_mapping_test.md).
