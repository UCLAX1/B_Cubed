from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("bb8_balance_controller")
    config_path = os.path.join(package_share, "config", "bb8_balance_controller.yaml")

    return LaunchDescription(
        [
            Node(
                package="bb8_balance_controller",
                executable="balance_controller",
                name="bb8_balance_controller",
                output="screen",
                parameters=[config_path],
            )
        ]
    )

