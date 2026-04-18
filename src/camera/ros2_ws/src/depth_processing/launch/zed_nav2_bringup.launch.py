#!/usr/bin/env python3
"""Bring up Nav2 on top of the repo's ZED + slam_toolbox topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for the Nav2 and safety-gate stack."""
    package_share = FindPackageShare("depth_processing")
    nav2_params = PathJoinSubstitution([package_share, "config", "nav2_zed_slam.yaml"])
    gate_params = PathJoinSubstitution(
        [package_share, "config", "twist_safety_gate.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    safety_gate_params_file = LaunchConfiguration("safety_gate_params_file")

    common_parameters = [
        nav2_params_file,
        {
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("nav2_params_file", default_value=nav2_params),
            DeclareLaunchArgument(
                "safety_gate_params_file",
                default_value=gate_params,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common_parameters,
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common_parameters,
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    nav2_params_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": ParameterValue(autostart, value_type=bool),
                    },
                ],
            ),
            Node(
                package="depth_processing",
                executable="twist_safety_gate",
                name="twist_safety_gate",
                output="screen",
                parameters=[
                    safety_gate_params_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]
    )
