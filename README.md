# B_Cubed
UCLA X1 Robotics 2025-26 B^3 Project

1. Install ROS2 Humble
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. Install the ZED SDK:
https://www.stereolabs.com/developers/release/5.0#82af3640d775

3. Install the ROS2 Wrapper for the ZED SDK
https://www.stereolabs.com/docs/ros2

4. Enable positional tracking in the ZED wrapper configuration.

Relevant wrapper parameters:
- `pos_tracking.pos_tracking_enabled: true`
- `pos_tracking.publish_tf: true`
- `pos_tracking.publish_map_tf: true`
- `pos_tracking.area_memory: true`
- `pos_tracking.two_d_mode: true` for a ground robot

5. Run the ZED positional tracking bridge node after the wrapper is running:
`ros2 run depth_processing zed_tracking`

Useful options:
- `--ros-args -p input_pose_topic:=/zed/zed_node/pose`
- `--ros-args -p input_pose_cov_topic:=/zed/zed_node/pose_with_covariance`
- `--ros-args -p input_odom_topic:=/zed/zed_node/odom`
- `--ros-args -p input_image_topic:=/zed/zed_node/rgb/color/rect/image/compressed`
- `--ros-args -p input_image_is_compressed:=true`
- `--ros-args -p require_odom_for_localized:=true`
- `--ros-args -p show_visualization_window:=true`

Notes:
- `zed_tracking` does not estimate localization itself. The ZED ROS 2 wrapper does that work; this node republishes the wrapper outputs in simpler topics for the rest of the robot.
- The wrapper pose is the camera pose in the map frame. For your BB-8 style robot, that means this is the head-mounted camera pose unless you add a separate transform/fusion step to estimate the body center.
- The node now also subscribes to the ZED color stream so it can show a live camera view with localization status, pose text, and a top-down trajectory inset.
- It publishes the recent camera trajectory on `zed/path`, which is useful in RViz.
- It publishes an annotated image on `zed/localization_view/compressed`, so you can inspect the localization overlay with `rqt_image_view` if the OpenCV window is inconvenient.

6. Navigation groundwork

To make the ZED localization usable for mapping and Nav2, there is now a base-frame adapter and a point-cloud-to-scan bringup launch.

Base-frame adapter:
- `ros2 run depth_processing zed_base_adapter`
- Converts camera-centric ZED topics into:
  - `zed/base_pose`
  - `zed/base_pose_with_covariance`
  - `zed/base_odom`
  - `zed/base_path`
- Publishes `odom -> base_link` by default.
- It can also publish `map -> odom`, but that is off by default so it does not fight the ZED wrapper if `pos_tracking.publish_map_tf: true` is already enabled.

Navigation groundwork launch:
- `ros2 launch depth_processing zed_nav_bringup.launch.py`
- Starts:
  - `zed_base_adapter`
  - `pointcloud_to_laserscan` to generate a 2D `scan` from `/zed/zed_node/point_cloud/cloud_registered`

Mapping pass launch:
- `ros2 launch depth_processing zed_mapping_pass.launch.py`
- Starts:
  - `zed_base_adapter`
  - `pointcloud_to_laserscan`
  - `slam_toolbox` in online async mapping mode
- This is the intended first mode for a new environment.

Localization-after-mapping launch:
- `ros2 launch depth_processing zed_localization_mode.launch.py map_file_name:=/absolute/path/to/posegraph_prefix`
- Starts:
  - `zed_base_adapter`
  - `pointcloud_to_laserscan`
  - `slam_toolbox` in localization mode using a saved pose graph
- This is the intended second mode after the robot finishes its mapping pass in that environment.

Useful options:
- `base_to_camera_translation:=x,y,z`
- `base_to_camera_rpy:=roll,pitch,yaw`
- `base_frame:=base_link`
- `publish_map_to_odom_tf:=true` if the wrapper is not already publishing `map -> odom`

See these docs for more detail:
- [Navigation plan](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/docs/navigation_plan.md)
- [Mapping to navigation workflow](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/docs/README_mapping_to_navigation.md)
- [Using launch.sh](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/docs/README_launch_sh.md)
- [ZED adapter and launch files](/Users/sara/Documents/My-Documents/X1 Robotics/B_Cubed/docs/README_zed_navigation_components.md)

7. One-stop Jetson launcher

`ros2_ws/launch.sh` now treats each run as a fresh mapping session:
- launches the ZED wrapper and the existing viewer / tracking nodes
- starts `zed_mapping_pass.launch.py`
- prints the per-run map prefix to save against
- waits for the current run's serialized slam files (`.posegraph` and `.data`)
- automatically stops the mapping pass and launches `zed_localization_mode.launch.py`

Useful environment overrides before running `launch.sh`:
- `MAP_SESSION_NAME=my_env`
- `MAP_OUTPUT_DIR=/home/jetson-nano-x1/Documents/B_Cubed/maps`
- `BASE_TO_CAMERA_TRANSLATION=x,y,z`
- `BASE_TO_CAMERA_RPY=roll,pitch,yaw`
- `NAVIGATION_AUTOSTART_COMMAND='ros2 launch your_nav_package your_nav_launch.py'`
