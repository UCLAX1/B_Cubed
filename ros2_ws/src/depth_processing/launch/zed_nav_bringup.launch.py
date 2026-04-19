#!/usr/bin/env python3
"""Bring up base-frame navigation topics and a 2D scan from the ZED camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for the ZED navigation groundwork nodes."""
    package_share = FindPackageShare("depth_processing")
    scan_params = PathJoinSubstitution(
        [package_share, "config", "pointcloud_to_laserscan.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/zed/zed_node/pose",
            ),
            DeclareLaunchArgument(
                "pose_cov_topic",
                default_value="/zed/zed_node/pose_with_covariance",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/zed/zed_node/odom",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/zed/zed_node/point_cloud/cloud_registered",
            ),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "base_to_camera_translation",
                default_value="0.0,0.0,0.381",
            ),
            DeclareLaunchArgument(
                "base_to_camera_rpy",
                default_value="0.0,0.0,0.0",
            ),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="false"),
            DeclareLaunchArgument("enable_base_adapter", default_value="true"),
            Node(
                package="depth_processing",
                executable="zed_base_adapter",
                name="zed_base_adapter",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_base_adapter")),
                parameters=[
                    {
                        "input_pose_topic": LaunchConfiguration("pose_topic"),
                        "input_pose_cov_topic": LaunchConfiguration("pose_cov_topic"),
                        "input_odom_topic": LaunchConfiguration("odom_topic"),
                        "base_frame_id": LaunchConfiguration("base_frame"),
                        "base_to_camera_translation": LaunchConfiguration(
                            "base_to_camera_translation"
                        ),
                        "base_to_camera_rpy": LaunchConfiguration("base_to_camera_rpy"),
                        "publish_odom_tf": ParameterValue(
                            LaunchConfiguration("publish_odom_tf"),
                            value_type=bool,
                        ),
                        "publish_map_to_odom_tf": ParameterValue(
                            LaunchConfiguration("publish_map_to_odom_tf"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="zed_scan_from_cloud",
                output="screen",
                parameters=[
                    scan_params,
                    {
                        "target_frame": LaunchConfiguration("base_frame"),
                    },
                ],
                remappings=[
                    ("cloud_in", LaunchConfiguration("cloud_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
        ]
    )
