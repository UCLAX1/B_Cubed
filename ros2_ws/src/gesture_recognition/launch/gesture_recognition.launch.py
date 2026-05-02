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
                description="ZED color image topic to subscribe to.",
            ),
            DeclareLaunchArgument(
                "image_is_compressed",
                default_value="true",
                description="Whether image_topic publishes sensor_msgs/CompressedImage.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Optional explicit MediaPipe gesture_recognizer.task path.",
            ),
            DeclareLaunchArgument(
                "show_window",
                default_value="false",
                description="Show an OpenCV annotation window.",
            ),
            DeclareLaunchArgument(
                "publish_annotated_image",
                default_value="true",
                description="Publish the annotated image as a compressed image topic.",
            ),
            DeclareLaunchArgument(
                "gesture_topic",
                default_value="/gesture_recognition/result",
                description="std_msgs/String topic for JSON gesture results.",
            ),
            DeclareLaunchArgument(
                "annotated_image_topic",
                default_value="/gesture_recognition/annotated_image/compressed",
                description="sensor_msgs/CompressedImage topic for annotated output frames.",
            ),
            Node(
                package="gesture_recognition",
                executable="gesture_recognition",
                name="gesture_recognition",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "image_is_compressed": ParameterValue(
                            LaunchConfiguration("image_is_compressed"),
                            value_type=bool,
                        ),
                        "model_path": LaunchConfiguration("model_path"),
                        "show_window": ParameterValue(
                            LaunchConfiguration("show_window"),
                            value_type=bool,
                        ),
                        "publish_annotated_image": ParameterValue(
                            LaunchConfiguration("publish_annotated_image"),
                            value_type=bool,
                        ),
                        "gesture_topic": LaunchConfiguration("gesture_topic"),
                        "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
                    }
                ],
            ),
        ]
    )
