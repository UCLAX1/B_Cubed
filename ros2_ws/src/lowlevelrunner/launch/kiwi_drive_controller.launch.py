#!/usr/bin/env python3
"""Launch the Kiwi drive controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for the Kiwi drive controller."""
    package_share = FindPackageShare("lowlevelrunner")
    default_params = PathJoinSubstitution(
        [package_share, "config", "kiwi_drive_controller.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    nav_cmd_topic = LaunchConfiguration("nav_cmd_topic")
    manual_cmd_topic = LaunchConfiguration("manual_cmd_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("hardware_enabled", default_value="true"),
            DeclareLaunchArgument("nav_cmd_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("manual_cmd_topic", default_value="cmd_vel_manual"),
            Node(
                package="lowlevelrunner",
                executable="kiwi_drive_controller",
                name="kiwi_drive_controller",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "hardware_enabled": ParameterValue(
                            hardware_enabled,
                            value_type=bool,
                        ),
                        "nav_cmd_topic": nav_cmd_topic,
                        "manual_cmd_topic": manual_cmd_topic,
                    },
                ],
            ),
        ]
    )
