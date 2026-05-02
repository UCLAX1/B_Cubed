#!/usr/bin/env python3
"""Bring up only the Nav2 planner server for click-to-plan demos."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for planner-only Nav2."""
    package_share = FindPackageShare("depth_processing")
    default_params = PathJoinSubstitution(
        [package_share, "config", "nav2_handheld_planner.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    planner_params_file = LaunchConfiguration("planner_params_file")

    common_parameters = [
        planner_params_file,
        {
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("planner_params_file", default_value=default_params),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_planner",
                output="screen",
                parameters=[
                    planner_params_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": ParameterValue(autostart, value_type=bool),
                        "node_names": ["planner_server"],
                    },
                ],
            ),
        ]
    )
