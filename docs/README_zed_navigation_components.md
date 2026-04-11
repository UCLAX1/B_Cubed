# `zed_base_adapter` And The Three ZED Navigation Launch Files

## Purpose

This document explains:

- what `zed_base_adapter` is for
- why it exists at all
- what each of the three ZED/navigation launch files is meant to do

Those three launch files are:

- [zed_nav_bringup.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_nav_bringup.launch.py)
- [zed_mapping_pass.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_mapping_pass.launch.py)
- [zed_localization_mode.launch.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/launch/zed_localization_mode.launch.py)

## What problem `zed_base_adapter` solves

The ZED wrapper gives you localization for the camera.

That is useful, but navigation software usually cares about the robot body frame, not the camera frame. For example, planners, costmaps, and controllers typically reason about `base_link` or a similar body-centered frame.

This mismatch matters because:

- the camera may be mounted forward of the robot center
- the camera may sit above the robot body
- the camera orientation may not match the robot forward axis exactly

If we pass the raw camera pose straight into navigation, the robot can appear offset from where its physical footprint actually is.

`zed_base_adapter` exists to fix that.

## What `zed_base_adapter` does

[zed_base_adapter.py](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/ros2_ws/src/depth_processing/depth_processing/zed_base_adapter.py) subscribes to:

- `zed/pose`
- `zed/pose_with_covariance`
- `zed/odom`

It then applies a rigid transform from the robot base to the camera, using:

- `base_to_camera_translation`
- `base_to_camera_rpy`

From that, it publishes robot-centric outputs:

- `zed/base_pose`
- `zed/base_pose_with_covariance`
- `zed/base_odom`
- `zed/base_path`

It can also publish TF:

- `odom -> base_link` by default
- `map -> odom` optionally

The optional `map -> odom` publication is off by default because another node, especially the ZED wrapper or `slam_toolbox`, may already be responsible for that transform.

## What `zed_base_adapter` does not do

It is important to be clear about its limits.

`zed_base_adapter` does not:

- perform SLAM
- build a map
- do path planning
- drive the robot
- infer a moving camera mount

It assumes the ZED camera is rigidly mounted to the robot body. If the camera can move independently of the chassis, this adapter alone is not enough for trustworthy body localization.

## Launch file 1: `zed_nav_bringup.launch.py`

This is the base-layer navigation bringup.

Use it when you want the minimum ZED-to-navigation groundwork without choosing mapping mode or localization mode yet.

It starts:

- `zed_base_adapter`
- `pointcloud_to_laserscan`

Its job is to create the core topics that mapping and navigation need:

- robot-base pose and odometry instead of camera-only pose
- a 2D `/scan` topic derived from the ZED point cloud

Mental model:

- `zed_nav_bringup.launch.py` is the shared foundation
- the other two launch files build on top of it

## Launch file 2: `zed_mapping_pass.launch.py`

This is the first-mode launch for a new environment.

It includes:

- `zed_nav_bringup.launch.py`
- `slam_toolbox` in online async mapping mode

Use it when:

- the robot is in a new environment
- no usable map for that environment exists yet
- the robot needs to perform its square or loop mapping pass

Its job is to:

- convert ZED data into robot and scan topics
- let `slam_toolbox` build a live map from those scans
- publish the map frame during mapping

This is the launch file that should be active while the operator is collecting the environment.

## Launch file 3: `zed_localization_mode.launch.py`

This is the second-mode launch after mapping has been completed and saved.

It includes:

- `zed_nav_bringup.launch.py`
- `slam_toolbox` in localization mode

Use it when:

- the mapping pass is complete
- the serialized map artifacts already exist
- the robot should localize within that map instead of continuing to build it

Its job is to:

- reload the saved `slam_toolbox` pose graph
- use the live scan stream to localize the robot in that environment
- provide the localization layer that a navigation stack can sit on top of

## How the three launch files fit together

The relationship is:

1. `zed_nav_bringup.launch.py` is the shared groundwork.
2. `zed_mapping_pass.launch.py` is groundwork plus live mapping.
3. `zed_localization_mode.launch.py` is groundwork plus post-mapping localization.

Another way to say it:

- if you only need frame conversion and `/scan`, use `zed_nav_bringup.launch.py`
- if you are learning a new environment, use `zed_mapping_pass.launch.py`
- if you already finished the mapping pass and want to reuse it, use `zed_localization_mode.launch.py`

## Recommended usage sequence

For a new environment:

1. Start `zed_mapping_pass.launch.py`.
2. Drive the mapping pass.
3. Save the occupancy map and serialized pose graph.
4. Stop mapping mode.
5. Start `zed_localization_mode.launch.py`.
6. Start the navigation stack on top of that localization layer.

For debugging or tuning:

1. Start `zed_nav_bringup.launch.py`.
2. Verify base-frame odometry and `/scan`.
3. Tune transforms and scan-generation parameters.
4. Only then move up to mapping or localization mode.
