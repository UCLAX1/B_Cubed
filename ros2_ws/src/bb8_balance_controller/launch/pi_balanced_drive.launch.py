#!/usr/bin/env python3
"""Bring up Sense HAT balance control and low-level motor output on the Pi."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for balanced Pi drivetrain control."""
    balance_share = FindPackageShare("bb8_balance_controller")
    low_level_share = FindPackageShare("low_level_runner")

    default_balance_params = PathJoinSubstitution(
        [balance_share, "config", "bb8_balance_controller.yaml"]
    )
    default_imu_params = PathJoinSubstitution(
        [balance_share, "config", "sense_hat_imu.yaml"]
    )
    default_policy = PathJoinSubstitution(
        [balance_share, "policies", "bb8_balance.npz"]
    )
    default_motor_params = PathJoinSubstitution(
        [low_level_share, "config", "pi_low_level_motor_control.yaml"]
    )

    start_sense_hat = LaunchConfiguration("start_sense_hat")
    start_motor_control = LaunchConfiguration("start_motor_control")
    policy_path = LaunchConfiguration("policy_path")
    allow_pd_fallback = LaunchConfiguration("allow_pd_fallback")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    nav_cmd_topic = LaunchConfiguration("nav_cmd_topic")
    manual_cmd_topic = LaunchConfiguration("manual_cmd_topic")
    balanced_cmd_topic = LaunchConfiguration("balanced_cmd_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    roll_offset_rad = LaunchConfiguration("roll_offset_rad")
    pitch_offset_rad = LaunchConfiguration("pitch_offset_rad")
    can_channel = LaunchConfiguration("can_channel")
    can_interface = LaunchConfiguration("can_interface")
    can_bitrate = LaunchConfiguration("can_bitrate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_sense_hat", default_value="true"),
            DeclareLaunchArgument("start_motor_control", default_value="true"),
            DeclareLaunchArgument(
                "balance_params_file",
                default_value=default_balance_params,
            ),
            DeclareLaunchArgument("sense_hat_params_file", default_value=default_imu_params),
            DeclareLaunchArgument("motor_params_file", default_value=default_motor_params),
            DeclareLaunchArgument("policy_path", default_value=default_policy),
            DeclareLaunchArgument("allow_pd_fallback", default_value="false"),
            DeclareLaunchArgument("hardware_enabled", default_value="true"),
            DeclareLaunchArgument("nav_cmd_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("manual_cmd_topic", default_value="cmd_vel_manual"),
            DeclareLaunchArgument("balanced_cmd_topic", default_value="cmd_vel_balanced"),
            DeclareLaunchArgument("imu_topic", default_value="imu/data"),
            DeclareLaunchArgument("roll_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("pitch_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("can_channel", default_value="can0"),
            DeclareLaunchArgument("can_interface", default_value="socketcan"),
            DeclareLaunchArgument("can_bitrate", default_value="1000000"),
            Node(
                package="bb8_balance_controller",
                executable="sense_hat_imu",
                name="sense_hat_imu",
                output="screen",
                condition=IfCondition(start_sense_hat),
                parameters=[
                    LaunchConfiguration("sense_hat_params_file"),
                    {"imu_topic": imu_topic},
                ],
            ),
            Node(
                package="bb8_balance_controller",
                executable="balance_controller",
                name="bb8_balance_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("balance_params_file"),
                    {
                        "policy_path": policy_path,
                        "allow_pd_fallback": ParameterValue(
                            allow_pd_fallback,
                            value_type=bool,
                        ),
                        "nav_cmd_topic": nav_cmd_topic,
                        "manual_cmd_topic": manual_cmd_topic,
                        "output_twist_topic": balanced_cmd_topic,
                        "imu_topic": imu_topic,
                        "roll_offset_rad": ParameterValue(
                            roll_offset_rad,
                            value_type=float,
                        ),
                        "pitch_offset_rad": ParameterValue(
                            pitch_offset_rad,
                            value_type=float,
                        ),
                        "tilt_topic": "",
                    },
                ],
            ),
            Node(
                package="low_level_runner",
                executable="low_level_motor_control",
                name="low_level_motor_control",
                output="screen",
                condition=IfCondition(start_motor_control),
                parameters=[
                    LaunchConfiguration("motor_params_file"),
                    {
                        "hardware_enabled": ParameterValue(
                            hardware_enabled,
                            value_type=bool,
                        ),
                        "nav_cmd_topic": balanced_cmd_topic,
                        "manual_cmd_topic": "cmd_vel_manual_balance_bypass_disabled",
                        "can_channel": can_channel,
                        "can_interface": can_interface,
                        "can_bitrate": ParameterValue(can_bitrate, value_type=int),
                    },
                ],
            ),
        ]
    )
