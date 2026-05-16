#!/usr/bin/env python3
"""Launch the local Nav2 web planning console."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for the web planning console."""
    package_share = FindPackageShare("nav_planning_console")
    default_params = PathJoinSubstitution(
        [package_share, "config", "nav_planning_console.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    map_topic = LaunchConfiguration("map_topic")
    global_frame = LaunchConfiguration("global_frame")
    base_frame = LaunchConfiguration("base_frame")
    planner_action_name = LaunchConfiguration("planner_action_name")
    navigator_action_name = LaunchConfiguration("navigator_action_name")
    manual_cmd_topic = LaunchConfiguration("manual_cmd_topic")
    navigation_plane_z = LaunchConfiguration("navigation_plane_z")
    person_tracking_control_enabled = LaunchConfiguration(
        "person_tracking_control_enabled"
    )
    person_tracking_image_topic = LaunchConfiguration("person_tracking_image_topic")
    person_tracking_engine_path = LaunchConfiguration("person_tracking_engine_path")
    person_tracking_show_window = LaunchConfiguration("person_tracking_show_window")
    person_tracking_publish_annotated_image = LaunchConfiguration(
        "person_tracking_publish_annotated_image"
    )
    person_tracking_detection_topic = LaunchConfiguration(
        "person_tracking_detection_topic"
    )
    person_tracking_annotated_image_topic = LaunchConfiguration(
        "person_tracking_annotated_image_topic"
    )
    person_tracking_confidence_threshold = LaunchConfiguration(
        "person_tracking_confidence_threshold"
    )
    person_tracking_nms_threshold = LaunchConfiguration(
        "person_tracking_nms_threshold"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("web_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "planner_action_name",
                default_value="compute_path_to_pose",
            ),
            DeclareLaunchArgument(
                "navigator_action_name",
                default_value="navigate_to_pose",
            ),
            DeclareLaunchArgument("manual_cmd_topic", default_value="cmd_vel_manual"),
            DeclareLaunchArgument("navigation_plane_z", default_value="0.0"),
            DeclareLaunchArgument(
                "person_tracking_control_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "person_tracking_image_topic",
                default_value="/zed/zed_node/rgb/color/rect/image/compressed",
            ),
            DeclareLaunchArgument(
                "person_tracking_engine_path",
                default_value=(
                    "/home/jetson-nano-x1/Documents/B_Cubed/models/yolo11n.engine"
                ),
            ),
            DeclareLaunchArgument("person_tracking_show_window", default_value="false"),
            DeclareLaunchArgument(
                "person_tracking_publish_annotated_image",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "person_tracking_detection_topic",
                default_value="/person_tracking/detections",
            ),
            DeclareLaunchArgument(
                "person_tracking_annotated_image_topic",
                default_value="/person_tracking/annotated_image/compressed",
            ),
            DeclareLaunchArgument(
                "person_tracking_confidence_threshold",
                default_value="0.4",
            ),
            DeclareLaunchArgument("person_tracking_nms_threshold", default_value="0.45"),
            Node(
                package="nav_planning_console",
                executable="nav_planning_console",
                name="nav_planning_console",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "web_host": web_host,
                        "web_port": ParameterValue(web_port, value_type=int),
                        "map_topic": map_topic,
                        "global_frame": global_frame,
                        "base_frame": base_frame,
                        "planner_action_name": planner_action_name,
                        "navigator_action_name": navigator_action_name,
                        "manual_cmd_topic": manual_cmd_topic,
                        "navigation_plane_z": ParameterValue(
                            navigation_plane_z,
                            value_type=float,
                        ),
                        "person_tracking_control_enabled": ParameterValue(
                            person_tracking_control_enabled,
                            value_type=bool,
                        ),
                        "person_tracking_image_topic": (
                            person_tracking_image_topic
                        ),
                        "person_tracking_engine_path": (
                            person_tracking_engine_path
                        ),
                        "person_tracking_show_window": ParameterValue(
                            person_tracking_show_window,
                            value_type=bool,
                        ),
                        "person_tracking_publish_annotated_image": ParameterValue(
                            person_tracking_publish_annotated_image,
                            value_type=bool,
                        ),
                        "person_tracking_detection_topic": (
                            person_tracking_detection_topic
                        ),
                        "person_tracking_annotated_image_topic": (
                            person_tracking_annotated_image_topic
                        ),
                        "person_tracking_confidence_threshold": ParameterValue(
                            person_tracking_confidence_threshold,
                            value_type=float,
                        ),
                        "person_tracking_nms_threshold": ParameterValue(
                            person_tracking_nms_threshold,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
