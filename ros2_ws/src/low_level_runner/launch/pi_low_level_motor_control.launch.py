#!/usr/bin/env python3
"""Launch the low-level motor control node for Raspberry Pi deployment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for the Pi motor controller."""
    package_share = FindPackageShare("low_level_runner")
    default_params = PathJoinSubstitution(
        [package_share, "config", "pi_low_level_motor_control.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    nav_cmd_topic = LaunchConfiguration("nav_cmd_topic")
    manual_cmd_topic = LaunchConfiguration("manual_cmd_topic")
    can_channel = LaunchConfiguration("can_channel")
    can_interface = LaunchConfiguration("can_interface")
    can_bitrate = LaunchConfiguration("can_bitrate")
    init_pos_path = LaunchConfiguration("init_pos_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("hardware_enabled", default_value="true"),
            DeclareLaunchArgument("nav_cmd_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("manual_cmd_topic", default_value="cmd_vel_manual"),
            DeclareLaunchArgument("can_channel", default_value="can0"),
            DeclareLaunchArgument("can_interface", default_value="socketcan"),
            DeclareLaunchArgument("can_bitrate", default_value="1000000"),
            DeclareLaunchArgument(
                "init_pos_path",
                default_value="~/.ros/b_cubed_motor_init_pos.json",
            ),
            Node(
                package="low_level_runner",
                executable="low_level_motor_control",
                name="low_level_motor_control",
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
                        "can_channel": can_channel,
                        "can_interface": can_interface,
                        "can_bitrate": ParameterValue(
                            can_bitrate,
                            value_type=int,
                        ),
                        "init_pos_path": init_pos_path,
                    },
                ],
            ),
        ]
    )
