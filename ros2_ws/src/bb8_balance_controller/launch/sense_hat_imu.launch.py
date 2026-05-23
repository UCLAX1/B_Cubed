#!/usr/bin/env python3
"""Launch the Sense HAT IMU publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for Sense HAT IMU publishing."""
    package_share = FindPackageShare("bb8_balance_controller")
    default_params = PathJoinSubstitution(
        [package_share, "config", "sense_hat_imu.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("imu_topic", default_value="imu/data"),
            Node(
                package="bb8_balance_controller",
                executable="sense_hat_imu",
                name="sense_hat_imu",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"imu_topic": LaunchConfiguration("imu_topic")},
                ],
            ),
        ]
    )
