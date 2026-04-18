# Mapping To Navigation Workflow

## Purpose

This document explains the intended runtime flow for the robot in a new environment.

The robot does not assume that a map already exists. Instead, it starts in a mapping mode, builds a map of the current environment, saves the mapping artifacts, and then switches into a localization and navigation mode that reuses what it just learned.

This is the intended high-level sequence:

1. Start the robot in mapping mode.
2. Drive a deliberate square or loop through the environment.
3. Save the occupancy map and the serialized SLAM pose graph.
4. Stop mapping.
5. Start localization mode using the saved pose graph.
6. Start the navigation stack once localization is stable.

## Why there are two saved outputs

At the end of the mapping pass, two different save operations matter:

- `save_map` writes the normal 2D occupancy map output, typically the `.yaml` and `.pgm` pair.
- `serialize_map` writes the serialized `slam_toolbox` session, typically the `.posegraph` and `.data` pair.

These serve different purposes:

- The occupancy map is the human-friendly map artifact. It is useful for inspection, visualization, and later use with map-aware tooling.
- The serialized pose graph is what `slam_toolbox` localization mode reloads so it can localize against the map that was just built.

The current automatic mode switch is driven by the serialized SLAM files, not the `.yaml` map, because the localization mode depends on the pose graph.

## What happens during mapping mode

Mapping mode is launched with [zed_mapping_pass.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_mapping_pass.launch.py).

That launch file does three main things:

1. Starts the ZED-to-base groundwork through [zed_nav_bringup.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_nav_bringup.launch.py).
2. Converts the ZED point cloud into a 2D `/scan`.
3. Starts `slam_toolbox` in online async mapping mode.

During this phase:

- the ZED wrapper provides camera localization and depth data
- `zed_base_adapter` converts camera-centric motion into robot-base motion
- `pointcloud_to_laserscan` projects the 3D ZED point cloud into a 2D scan
- `slam_toolbox` uses that scan and robot motion estimate to build a 2D map

In this mode, `slam_toolbox` should own `map -> odom`.

## Recommended mapping pass behavior

The mapping pass should be structured and repeatable.

Recommended operator behavior:

- start with the robot reasonably centered in the local area
- drive a square, rectangle, or loop with clear overlap
- keep motion smooth enough that scan matching stays stable
- avoid spinning in place for long periods unless you are intentionally improving loop closure
- revisit part of the route before saving so the map gets overlap and consistency

The goal is not just to cover space, but to create enough repeated geometry for `slam_toolbox` to close loops and stabilize the map.

## What happens when mapping is complete

Once the mapping pass looks good in RViz or another map viewer, save both outputs:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '<map_prefix>'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '<map_prefix>'}"
```

After that:

- the serialized `.posegraph` and `.data` files become the handoff point
- the mapping stack can be stopped
- localization mode can be launched against the saved pose graph

## What happens during localization mode

Localization mode is launched with [zed_localization_mode.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_localization_mode.launch.py).

That launch file:

1. Starts the same ZED base-frame and scan-generation pipeline.
2. Starts `slam_toolbox` in localization mode instead of mapping mode.
3. Reloads the saved pose graph through the `map_file_name` parameter.

In this phase:

- the map is no longer being actively built in the same way
- `slam_toolbox` uses the current scan stream to estimate where the robot is within the saved map
- the resulting localization can be used by a navigation stack

This is the correct phase to start full navigation behavior.

The repo now includes a dedicated Nav2 bringup launch for this phase:

- [zed_nav2_bringup.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_nav2_bringup.launch.py)

If you want a single command for the full stack instead, use:

- [zed_slam_nav.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_slam_nav.launch.py)

## Why this approach fits changing environments

The robot is expected to work in many different environments, so a prebuilt static map is not enough.

This workflow makes the robot adaptable:

- the first pass learns the current environment
- the second phase reuses that learned environment for repeatable navigation
- each run can save its own environment-specific outputs

## Current boundaries

The repo now covers the mapping and post-mapping localization stages, but there are still some system-level assumptions:

- `zed_base_adapter` assumes the ZED camera is rigidly mounted relative to the robot body
- the 2D scan depends on point cloud projection parameters that may need tuning per robot geometry
- the robot still needs a real hardware node that consumes `/cmd_vel`
- Nav2 tuning still needs to be validated on the physical robot

## Practical mental model

The easiest way to think about the whole flow is:

- `zed_tracking` helps you see what the ZED thinks is happening
- `zed_base_adapter` turns camera motion into robot motion
- `zed_mapping_pass.launch.py` learns the environment
- `zed_localization_mode.launch.py` reuses that learned environment
- `zed_nav2_bringup.launch.py` runs the autonomous stack on top of the map and scan topics
- `zed_slam_nav.launch.py` can run SLAM plus Nav2 in one command
