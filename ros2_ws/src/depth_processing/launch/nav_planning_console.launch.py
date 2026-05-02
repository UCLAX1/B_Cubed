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
    package_share = FindPackageShare("depth_processing")
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
    navigation_plane_z = LaunchConfiguration("navigation_plane_z")

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
            DeclareLaunchArgument("navigation_plane_z", default_value="0.0"),
            Node(
                package="depth_processing",
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
                        "navigation_plane_z": ParameterValue(
                            navigation_plane_z,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
