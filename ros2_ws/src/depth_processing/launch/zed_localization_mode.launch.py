#!/usr/bin/env python3
"""Run ZED-based localization against a saved slam_toolbox pose graph."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for localization after a mapping pass."""
    depth_processing_share = FindPackageShare("depth_processing")
    localization_params = PathJoinSubstitution(
        [depth_processing_share, "config", "slam_toolbox_localization.yaml"]
    )
    nav_groundwork_launch = PathJoinSubstitution(
        [depth_processing_share, "launch", "zed_nav_bringup.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/zed/zed_node/point_cloud/cloud_registered",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("base_to_camera_translation", default_value="0.0,0.0,0.381"),
            DeclareLaunchArgument("base_to_camera_rpy", default_value="0.0,0.0,0.0"),
            DeclareLaunchArgument("flatten_navigation_to_2d", default_value="true"),
            DeclareLaunchArgument("navigation_plane_z", default_value="0.0"),
            DeclareLaunchArgument("enable_base_adapter", default_value="true"),
            DeclareLaunchArgument("localization_params_file", default_value=localization_params),
            DeclareLaunchArgument("map_file_name", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_groundwork_launch),
                launch_arguments={
                    "cloud_topic": LaunchConfiguration("cloud_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "base_to_camera_translation": LaunchConfiguration(
                        "base_to_camera_translation"
                    ),
                    "base_to_camera_rpy": LaunchConfiguration("base_to_camera_rpy"),
                    "flatten_navigation_to_2d": LaunchConfiguration(
                        "flatten_navigation_to_2d"
                    ),
                    "navigation_plane_z": LaunchConfiguration("navigation_plane_z"),
                    "publish_map_to_odom_tf": "false",
                    "enable_base_adapter": LaunchConfiguration("enable_base_adapter"),
                }.items(),
            ),
            Node(
                package="slam_toolbox",
                executable="localization_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    LaunchConfiguration("localization_params_file"),
                    {
                        "base_frame": LaunchConfiguration("base_frame"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "map_file_name": LaunchConfiguration("map_file_name"),
                    },
                ],
            ),
        ]
    )
