# Navigation Plan

## Current state

The repo now has two useful localization layers:

- `zed_tracking` republishes the ZED wrapper's pose, pose-with-covariance, odometry, and localization status while showing a live localization overlay.
- `zed_base_adapter` converts that camera-centric localization into robot-centric navigation frames and topics:
  - `zed/base_pose`
  - `zed/base_pose_with_covariance`
  - `zed/base_odom`
  - `zed/base_path`
  - `odom -> base_link` TF
  - optional `map -> odom` TF

This is the minimum frame setup needed for downstream navigation software.

## Key assumption

The current adapter assumes the camera is rigidly mounted to the robot body.

If the ZED is mounted on a head or mast that can move relative to the chassis, the base pose will only be as good as that assumption. In that case we will need one of these before trusting autonomous navigation:

- a fixed camera mount
- a joint-state-aware transform from base to camera
- an additional state estimator that fuses body motion with camera motion

## Phase 1: Groundwork now in repo

Goal: make the ZED outputs usable by mapping and Nav2.

What is implemented:

- `ros2 run depth_processing zed_base_adapter`
- `ros2 launch depth_processing zed_nav_bringup.launch.py`
- conversion from ZED point cloud to a 2D `scan` topic

Jetson validation checklist:

1. Confirm `zed/base_odom` looks stable while driving.
2. Confirm TF contains `map -> odom -> base_link`.
3. Make sure only one node is publishing `map -> odom`.
4. Confirm `/scan` is centered on the robot body, not visibly offset by the camera mount.
5. Tune `base_to_camera_translation` and `base_to_camera_rpy`.
6. Tune `pointcloud_to_laserscan.yaml` height and range limits for the robot's actual camera height.

## Phase 2: Mapping pass in each environment

Recommended first mapping path: start every run in a short online SLAM phase, drive a deliberate square or loop through the local environment, then switch out of mapping once the map is good enough.

Candidate stack:

- `pointcloud_to_laserscan` for scan generation
- `slam_toolbox` for 2D mapping and map saving

What is now in the repo:

- `ros2 launch depth_processing zed_mapping_pass.launch.py`
- `config/slam_toolbox_mapping.yaml`

Expected behavior:

- `slam_toolbox` owns `map -> odom`
- the robot generates a live occupancy map from `/scan`
- after the square pass, save both:
  - a normal 2D map for visualization and Nav2 consumption
  - a serialized pose graph for slam_toolbox localization mode

Decision to validate on the Jetson:

- During mapping, either let `slam_toolbox` own `map -> odom`, or keep ZED as the global localizer and isolate SLAM in a separate frame while we compare results.

Why this matters:

- We should avoid two nodes publishing different `map -> odom` transforms at the same time.

## Phase 3: Post-mapping localization and navigation

Recommended first navigation path:

- stop the mapping node after the environment scan is complete
- relaunch in slam_toolbox localization mode against the just-saved pose graph
- feed `scan` into local and global costmaps
- plug Nav2 into the robot's `cmd_vel` interface

What is now in the repo:

- `ros2 launch depth_processing zed_localization_mode.launch.py map_file_name:=/absolute/path/to/posegraph_prefix`
- `ros2 launch depth_processing zed_nav2_bringup.launch.py`
- `ros2 launch depth_processing zed_slam_nav.launch.py slam_mode:=mapping|localization`
- `config/nav2_zed_slam.yaml`
- `config/twist_safety_gate.yaml`

What is now wired:

- Nav2 planner, controller, behavior server, and BT navigator
- a `cmd_vel` safety gate that blocks motion when localization or `/scan` goes stale
- one-command bringup for continuous mapping plus navigation in the same run

What still needs field validation:

- the real robot drive node that consumes `/cmd_vel`
- final footprint, velocity, and costmap tuning on hardware
- the operator workflow for deciding when to stay in mapping mode versus switching to localization mode

## Suggested order

1. Validate `zed_base_adapter` and `/scan` on the Jetson.
2. Run `zed_mapping_pass.launch.py` and tune the mapping pass until the square route produces a clean occupancy map.
3. Save a first map and serialized pose graph, then relaunch with `zed_localization_mode.launch.py`.
4. Bring up `zed_nav2_bringup.launch.py` against the robot's real drive interface.
5. Tune controller and costmap behavior in place.
6. Use `zed_slam_nav.launch.py slam_mode:=mapping` when you want the map to keep updating while Nav2 is active.
