#!/usr/bin/env python3
"""Bring up ZED tracking, SLAM, Nav2, and cmd_vel safety gating."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch actions for mapping/localization plus Nav2."""
    package_share = FindPackageShare("depth_processing")
    mapping_launch = PathJoinSubstitution(
        [package_share, "launch", "zed_mapping_pass.launch.py"]
    )
    localization_launch = PathJoinSubstitution(
        [package_share, "launch", "zed_localization_mode.launch.py"]
    )
    nav2_launch = PathJoinSubstitution(
        [package_share, "launch", "zed_nav2_bringup.launch.py"]
    )
    planner_only_launch = PathJoinSubstitution(
        [package_share, "launch", "zed_planner_only.launch.py"]
    )
    nav2_params = PathJoinSubstitution([package_share, "config", "nav2_zed_slam.yaml"])
    planner_only_params = PathJoinSubstitution(
        [package_share, "config", "nav2_handheld_planner.yaml"]
    )
    gate_params = PathJoinSubstitution(
        [package_share, "config", "twist_safety_gate.yaml"]
    )

    slam_mode = LaunchConfiguration("slam_mode")

    mapping_condition = IfCondition(
        PythonExpression(["'", slam_mode, "' == 'mapping'"])
    )
    localization_condition = IfCondition(
        PythonExpression(["'", slam_mode, "' == 'localization'"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("slam_mode", default_value="mapping"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_nav2", default_value="true"),
            DeclareLaunchArgument("enable_planner_only", default_value="false"),
            DeclareLaunchArgument("enable_planning_console", default_value="false"),
            DeclareLaunchArgument("planning_console_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("planning_console_port", default_value="8080"),
            DeclareLaunchArgument("enable_tracking_node", default_value="true"),
            DeclareLaunchArgument("autostart_nav2", default_value="true"),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/zed/zed_node/point_cloud/cloud_registered",
            ),
            DeclareLaunchArgument("input_pose_topic", default_value="/zed/zed_node/pose"),
            DeclareLaunchArgument(
                "input_pose_cov_topic",
                default_value="/zed/zed_node/pose_with_covariance",
            ),
            DeclareLaunchArgument("input_odom_topic", default_value="/zed/zed_node/odom"),
            DeclareLaunchArgument(
                "input_image_topic",
                default_value="/zed/zed_node/rgb/color/rect/image/compressed",
            ),
            DeclareLaunchArgument("input_image_is_compressed", default_value="true"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("enable_base_adapter", default_value="true"),
            DeclareLaunchArgument("flatten_navigation_to_2d", default_value="true"),
            DeclareLaunchArgument("navigation_plane_z", default_value="0.0"),
            DeclareLaunchArgument(
                "base_to_camera_translation",
                default_value="0.0,0.0,0.381",
            ),
            DeclareLaunchArgument("base_to_camera_rpy", default_value="0.0,0.0,0.0"),
            DeclareLaunchArgument("map_file_name", default_value=""),
            DeclareLaunchArgument("nav2_params_file", default_value=nav2_params),
            DeclareLaunchArgument(
                "planner_only_params_file",
                default_value=planner_only_params,
            ),
            DeclareLaunchArgument(
                "safety_gate_params_file",
                default_value=gate_params,
            ),
            DeclareLaunchArgument("enable_tracking_visualization", default_value="false"),
            DeclareLaunchArgument("show_tracking_window", default_value="false"),
            DeclareLaunchArgument("publish_tracking_image", default_value="false"),
            Node(
                package="depth_processing",
                executable="zed_tracking",
                name="zed_tracking",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_tracking_node")),
                parameters=[
                    {
                        "input_pose_topic": LaunchConfiguration("input_pose_topic"),
                        "input_pose_cov_topic": LaunchConfiguration(
                            "input_pose_cov_topic"
                        ),
                        "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                        "input_image_topic": LaunchConfiguration("input_image_topic"),
                        "input_image_is_compressed": ParameterValue(
                            LaunchConfiguration("input_image_is_compressed"),
                            value_type=bool,
                        ),
                        "require_odom_for_localized": True,
                        "enable_visualization": ParameterValue(
                            LaunchConfiguration("enable_tracking_visualization"),
                            value_type=bool,
                        ),
                        "show_visualization_window": ParameterValue(
                            LaunchConfiguration("show_tracking_window"),
                            value_type=bool,
                        ),
                        "publish_visualization_image": ParameterValue(
                            LaunchConfiguration("publish_tracking_image"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                condition=mapping_condition,
                launch_arguments={
                    "cloud_topic": LaunchConfiguration("cloud_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "enable_base_adapter": LaunchConfiguration("enable_base_adapter"),
                    "base_to_camera_translation": LaunchConfiguration(
                        "base_to_camera_translation"
                    ),
                    "base_to_camera_rpy": LaunchConfiguration("base_to_camera_rpy"),
                    "flatten_navigation_to_2d": LaunchConfiguration(
                        "flatten_navigation_to_2d"
                    ),
                    "navigation_plane_z": LaunchConfiguration("navigation_plane_z"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                condition=localization_condition,
                launch_arguments={
                    "cloud_topic": LaunchConfiguration("cloud_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "enable_base_adapter": LaunchConfiguration("enable_base_adapter"),
                    "base_to_camera_translation": LaunchConfiguration(
                        "base_to_camera_translation"
                    ),
                    "base_to_camera_rpy": LaunchConfiguration("base_to_camera_rpy"),
                    "flatten_navigation_to_2d": LaunchConfiguration(
                        "flatten_navigation_to_2d"
                    ),
                    "navigation_plane_z": LaunchConfiguration("navigation_plane_z"),
                    "map_file_name": LaunchConfiguration("map_file_name"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(LaunchConfiguration("enable_nav2")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": LaunchConfiguration("autostart_nav2"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "safety_gate_params_file": LaunchConfiguration(
                        "safety_gate_params_file"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planner_only_launch),
                condition=IfCondition(LaunchConfiguration("enable_planner_only")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": "true",
                    "planner_params_file": LaunchConfiguration(
                        "planner_only_params_file"
                    ),
                }.items(),
            ),
            Node(
                package="depth_processing",
                executable="nav_planning_console",
                name="nav_planning_console",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("enable_planning_console")
                ),
                parameters=[
                    {
                        "web_host": LaunchConfiguration("planning_console_host"),
                        "web_port": ParameterValue(
                            LaunchConfiguration("planning_console_port"),
                            value_type=int,
                        ),
                        "map_topic": "/map",
                        "global_frame": "map",
                        "base_frame": LaunchConfiguration("base_frame"),
                        "planner_action_name": "compute_path_to_pose",
                        "navigator_action_name": "navigate_to_pose",
                        "navigation_plane_z": ParameterValue(
                            LaunchConfiguration("navigation_plane_z"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
