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
- `--ros-args -p require_odom_for_localized:=true`

Notes:
- `zed_tracking` does not access the camera directly. It only subscribes to the ZED ROS 2 wrapper topics.
- The wrapper pose is the camera pose in the map frame. For your BB-8 style robot, that means this is the head-mounted camera pose unless you add a separate transform/fusion step to estimate the body center.
