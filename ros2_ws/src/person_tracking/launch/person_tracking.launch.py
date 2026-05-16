from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/zed/zed_node/rgb/color/rect/image/compressed",
                description="Compressed ZED color image topic to subscribe to.",
            ),
            DeclareLaunchArgument(
                "engine_path",
                default_value="/home/jetson-nano-x1/Documents/B_Cubed/models/yolo11n.engine",
                description="TensorRT engine path for the YOLO person detector.",
            ),
            DeclareLaunchArgument(
                "show_window",
                default_value="false",
                description="Show an OpenCV person-tracking window.",
            ),
            DeclareLaunchArgument(
                "publish_annotated_image",
                default_value="true",
                description="Publish annotated person-tracking images.",
            ),
            DeclareLaunchArgument(
                "annotated_image_topic",
                default_value="/person_tracking/annotated_image/compressed",
                description="Annotated compressed image output topic.",
            ),
            DeclareLaunchArgument(
                "detection_topic",
                default_value="/person_tracking/detections",
                description="std_msgs/String JSON person detection topic.",
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.4",
                description="Minimum person confidence.",
            ),
            DeclareLaunchArgument(
                "nms_threshold",
                default_value="0.45",
                description="IoU threshold for non-maximum suppression.",
            ),
            Node(
                package="person_tracking",
                executable="person",
                name="person_tracking",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "engine_path": LaunchConfiguration("engine_path"),
                        "show_window": ParameterValue(
                            LaunchConfiguration("show_window"),
                            value_type=bool,
                        ),
                        "publish_annotated_image": ParameterValue(
                            LaunchConfiguration("publish_annotated_image"),
                            value_type=bool,
                        ),
                        "annotated_image_topic": LaunchConfiguration(
                            "annotated_image_topic"
                        ),
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "nms_threshold": ParameterValue(
                            LaunchConfiguration("nms_threshold"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
