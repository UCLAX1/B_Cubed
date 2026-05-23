import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            DeclareLaunchArgument(
                "allow_pd_fallback",
                default_value="false",
                description="Allow PD fallback if the learned policy cannot load.",
            ),
            DeclareLaunchArgument("nav_cmd_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("manual_cmd_topic", default_value="cmd_vel_manual"),
            DeclareLaunchArgument("imu_topic", default_value="imu/data"),
            Node(
                package="bb8_balance_controller",
                executable="balance_controller",
                name="bb8_balance_controller",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "allow_pd_fallback": ParameterValue(
                            LaunchConfiguration("allow_pd_fallback"),
                            value_type=bool,
                        ),
                        "nav_cmd_topic": LaunchConfiguration("nav_cmd_topic"),
                        "manual_cmd_topic": LaunchConfiguration("manual_cmd_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                    },
                ],
            )
        ]
    )
