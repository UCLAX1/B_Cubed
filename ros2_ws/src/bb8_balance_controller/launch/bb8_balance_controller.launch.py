import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("bb8_balance_controller")
    config_path = os.path.join(package_share, "config", "bb8_balance_controller.yaml")
    default_policy_path = os.path.join(package_share, "policies", "bb8_balance.npz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_path",
                default_value=default_policy_path,
                description="Path to the exported NumPy balance policy.",
            ),
            Node(
                package="bb8_balance_controller",
                executable="balance_controller",
                name="bb8_balance_controller",
                output="screen",
                parameters=[
                    config_path,
                    {"policy_path": LaunchConfiguration("policy_path")},
                ],
            )
        ]
    )
