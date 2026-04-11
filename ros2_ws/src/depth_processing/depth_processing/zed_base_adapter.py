#!/usr/bin/env python3
"""Adapt ZED camera localization topics into robot-base navigation frames."""

from collections import deque
from copy import deepcopy
import math
from typing import Sequence

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def _normalize_quaternion(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
    """Return a unit quaternion, or identity if the input norm is invalid."""
    x_value, y_value, z_value, w_value = [float(component) for component in quaternion[:4]]
    norm = math.sqrt(
        x_value * x_value
        + y_value * y_value
        + z_value * z_value
        + w_value * w_value
    )
    if norm < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return (
        x_value / norm,
        y_value / norm,
        z_value / norm,
        w_value / norm,
    )


def _quaternion_multiply(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float, float]:
    """Hamilton product of two quaternions in x/y/z/w order."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_inverse(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
    """Inverse of a unit quaternion in x/y/z/w order."""
    x_value, y_value, z_value, w_value = _normalize_quaternion(quaternion)
    return (-x_value, -y_value, -z_value, w_value)


def _rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    """Rotate a 3D vector by a quaternion."""
    qx, qy, qz, qw = _normalize_quaternion(quaternion)
    vx, vy, vz = [float(component) for component in vector[:3]]

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _cross(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    """Compute a 3D cross product."""
    lx, ly, lz = [float(component) for component in left[:3]]
    rx, ry, rz = [float(component) for component in right[:3]]
    return (
        ly * rz - lz * ry,
        lz * rx - lx * rz,
        lx * ry - ly * rx,
    )


def _quaternion_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Convert roll/pitch/yaw to x/y/z/w quaternion order."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    sin_r = math.sin(half_roll)
    cos_r = math.cos(half_roll)
    sin_p = math.sin(half_pitch)
    cos_p = math.cos(half_pitch)
    sin_y = math.sin(half_yaw)
    cos_y = math.cos(half_yaw)

    return _normalize_quaternion(
        (
            sin_r * cos_p * cos_y - cos_r * sin_p * sin_y,
            cos_r * sin_p * cos_y + sin_r * cos_p * sin_y,
            cos_r * cos_p * sin_y - sin_r * sin_p * cos_y,
            cos_r * cos_p * cos_y + sin_r * sin_p * sin_y,
        )
    )


class ZedBaseAdapterNode(Node):
    """Convert ZED camera-centric localization into base_link-centric navigation topics."""

    def __init__(self) -> None:
        super().__init__("zed_base_adapter")

        self._declare_parameters()

        self.input_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self.input_pose_cov_topic = str(self.get_parameter("input_pose_cov_topic").value)
        self.input_odom_topic = str(self.get_parameter("input_odom_topic").value)

        self.output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        self.output_pose_cov_topic = str(
            self.get_parameter("output_pose_cov_topic").value
        )
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.output_path_topic = str(self.get_parameter("output_path_topic").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)
        self.publish_map_to_odom_tf = bool(
            self.get_parameter("publish_map_to_odom_tf").value
        )
        self.path_history_length = max(
            2,
            int(self.get_parameter("path_history_length").value),
        )

        self.base_to_camera_translation = self._read_vector_parameter(
            "base_to_camera_translation",
            3,
            [0.0, 0.0, 0.0],
        )
        self.base_to_camera_rpy = self._read_vector_parameter(
            "base_to_camera_rpy",
            3,
            [0.0, 0.0, 0.0],
        )

        self.base_to_camera_rotation = _quaternion_from_rpy(
            self.base_to_camera_rpy[0],
            self.base_to_camera_rpy[1],
            self.base_to_camera_rpy[2],
        )
        self.camera_to_base_rotation = _quaternion_inverse(self.base_to_camera_rotation)
        self.camera_to_base_translation = _rotate_vector(
            self.camera_to_base_rotation,
            (
                -self.base_to_camera_translation[0],
                -self.base_to_camera_translation[1],
                -self.base_to_camera_translation[2],
            ),
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_pose_cov_topic,
            10,
        )
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.output_path_topic, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self._pose_callback,
            qos,
        )
        self.pose_cov_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_pose_cov_topic,
            self._pose_cov_callback,
            qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self._odom_callback,
            qos,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.latest_map_base_pose = None
        self.latest_odom_base = None
        self.pose_history = deque(maxlen=self.path_history_length)
        self.warned_map_odom_same_frame = False
        self.warned_missing_odom_frame = False
        self.warned_missing_map_or_odom_frame = False

        self.get_logger().info(
            "Adapting ZED camera localization to the robot base frame with "
            f"base_frame_id={self.base_frame_id}."
        )
        self.get_logger().info(
            "Assuming a rigid mount from base to camera with translation="
            f"{self.base_to_camera_translation} m and rpy="
            f"{self.base_to_camera_rpy} rad."
        )
        self.get_logger().info(
            "Publishing base-centric navigation topics to "
            f"{self.output_pose_topic}, {self.output_pose_cov_topic}, "
            f"{self.output_odom_topic}, and {self.output_path_topic}."
        )

    def _declare_parameters(self) -> None:
        """Declare runtime-configurable parameters."""
        self.declare_parameter("input_pose_topic", "zed/pose")
        self.declare_parameter("input_pose_cov_topic", "zed/pose_with_covariance")
        self.declare_parameter("input_odom_topic", "zed/odom")

        self.declare_parameter("output_pose_topic", "zed/base_pose")
        self.declare_parameter("output_pose_cov_topic", "zed/base_pose_with_covariance")
        self.declare_parameter("output_odom_topic", "zed/base_odom")
        self.declare_parameter("output_path_topic", "zed/base_path")

        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("base_to_camera_translation", "0.0,0.0,0.0")
        self.declare_parameter("base_to_camera_rpy", "0.0,0.0,0.0")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("publish_map_to_odom_tf", False)
        self.declare_parameter("path_history_length", 400)

    def _read_vector_parameter(
        self,
        parameter_name: str,
        expected_length: int,
        fallback: Sequence[float],
    ) -> list[float]:
        """Read a numeric vector parameter from either YAML arrays or comma-separated text."""
        raw_value = self.get_parameter(parameter_name).value

        if isinstance(raw_value, str):
            parts = [part.strip() for part in raw_value.split(",") if part.strip()]
            parsed = [float(part) for part in parts]
        else:
            parsed = [float(part) for part in list(raw_value)]

        if len(parsed) != expected_length:
            self.get_logger().warning(
                f"Parameter '{parameter_name}' expected {expected_length} values. "
                f"Falling back to {list(fallback)}."
            )
            return [float(value) for value in fallback]

        return parsed

    def _pose_callback(self, msg: PoseStamped) -> None:
        """Convert the map-frame camera pose into a base-frame pose."""
        base_pose = self._transform_pose(msg)
        self.latest_map_base_pose = deepcopy(base_pose)
        self.pose_pub.publish(base_pose)
        self._record_path_pose(base_pose)
        self._publish_map_to_odom_tf(base_pose.header.stamp)

    def _pose_cov_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Convert the map-frame camera pose with covariance into base-frame outputs."""
        base_pose_cov = self._transform_pose_covariance(msg)
        self.pose_cov_pub.publish(base_pose_cov)

        base_pose = PoseStamped()
        base_pose.header = base_pose_cov.header
        base_pose.pose = base_pose_cov.pose.pose
        self.latest_map_base_pose = deepcopy(base_pose)
        self.pose_pub.publish(base_pose)
        self._record_path_pose(base_pose)
        self._publish_map_to_odom_tf(base_pose.header.stamp)

    def _odom_callback(self, msg: Odometry) -> None:
        """Convert camera odometry into base odometry and publish TF."""
        base_odom = self._transform_odometry(msg)
        self.latest_odom_base = deepcopy(base_odom)
        self.odom_pub.publish(base_odom)

        if self.publish_odom_tf:
            self._publish_odom_tf(base_odom)

        self._publish_map_to_odom_tf(base_odom.header.stamp)

    def _transform_pose(self, pose_msg: PoseStamped) -> PoseStamped:
        """Transform a camera pose into the robot base pose."""
        output = PoseStamped()
        output.header = pose_msg.header
        output.pose = deepcopy(pose_msg.pose)

        world_to_camera_rotation = _normalize_quaternion(
            (
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            )
        )
        rotated_offset = _rotate_vector(
            world_to_camera_rotation,
            self.camera_to_base_translation,
        )
        output.pose.position.x = pose_msg.pose.position.x + rotated_offset[0]
        output.pose.position.y = pose_msg.pose.position.y + rotated_offset[1]
        output.pose.position.z = pose_msg.pose.position.z + rotated_offset[2]

        world_to_base_rotation = _quaternion_multiply(
            world_to_camera_rotation,
            self.camera_to_base_rotation,
        )
        output.pose.orientation.x = world_to_base_rotation[0]
        output.pose.orientation.y = world_to_base_rotation[1]
        output.pose.orientation.z = world_to_base_rotation[2]
        output.pose.orientation.w = world_to_base_rotation[3]
        return output

    def _transform_pose_covariance(
        self,
        pose_cov_msg: PoseWithCovarianceStamped,
    ) -> PoseWithCovarianceStamped:
        """Transform a camera pose-with-covariance into a base pose-with-covariance."""
        output = PoseWithCovarianceStamped()
        output.header = pose_cov_msg.header
        output.pose = deepcopy(pose_cov_msg.pose)

        pose_view = PoseStamped()
        pose_view.header = pose_cov_msg.header
        pose_view.pose = pose_cov_msg.pose.pose
        transformed_pose = self._transform_pose(pose_view)
        output.pose.pose = transformed_pose.pose
        return output

    def _transform_odometry(self, odom_msg: Odometry) -> Odometry:
        """Transform camera odometry into base_link-centric odometry."""
        output = Odometry()
        output.header = odom_msg.header
        output.child_frame_id = self.base_frame_id
        output.pose = deepcopy(odom_msg.pose)
        output.twist = deepcopy(odom_msg.twist)

        pose_view = PoseStamped()
        pose_view.header = odom_msg.header
        pose_view.pose = odom_msg.pose.pose
        transformed_pose = self._transform_pose(pose_view)
        output.pose.pose = transformed_pose.pose

        linear_velocity_camera = (
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z,
        )
        angular_velocity_camera = (
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z,
        )

        angular_velocity_base = _rotate_vector(
            self.camera_to_base_rotation,
            angular_velocity_camera,
        )
        linear_velocity_base_at_camera = _rotate_vector(
            self.camera_to_base_rotation,
            linear_velocity_camera,
        )
        correction = _cross(angular_velocity_base, self.base_to_camera_translation)
        linear_velocity_base = (
            linear_velocity_base_at_camera[0] - correction[0],
            linear_velocity_base_at_camera[1] - correction[1],
            linear_velocity_base_at_camera[2] - correction[2],
        )

        output.twist.twist.linear.x = linear_velocity_base[0]
        output.twist.twist.linear.y = linear_velocity_base[1]
        output.twist.twist.linear.z = linear_velocity_base[2]
        output.twist.twist.angular.x = angular_velocity_base[0]
        output.twist.twist.angular.y = angular_velocity_base[1]
        output.twist.twist.angular.z = angular_velocity_base[2]
        return output

    def _record_path_pose(self, pose_msg: PoseStamped) -> None:
        """Append the newest base pose to the path history."""
        if self.pose_history and self.pose_history[-1].header.frame_id != pose_msg.header.frame_id:
            self.get_logger().warning(
                "Base pose frame changed from "
                f"{self.pose_history[-1].header.frame_id} to {pose_msg.header.frame_id}. "
                "Clearing path history."
            )
            self.pose_history.clear()

        self.pose_history.append(deepcopy(pose_msg))

        path_msg = Path()
        path_msg.header = pose_msg.header
        path_msg.poses = [deepcopy(path_pose) for path_pose in self.pose_history]
        self.path_pub.publish(path_msg)

    def _publish_odom_tf(self, base_odom: Odometry) -> None:
        """Publish the odom -> base_link transform."""
        if not base_odom.header.frame_id:
            if not self.warned_missing_odom_frame:
                self.get_logger().warning(
                    "Input odometry has no header.frame_id; cannot publish odom TF."
                )
                self.warned_missing_odom_frame = True
            return

        transform = TransformStamped()
        transform.header = base_odom.header
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = base_odom.pose.pose.position.x
        transform.transform.translation.y = base_odom.pose.pose.position.y
        transform.transform.translation.z = base_odom.pose.pose.position.z
        transform.transform.rotation = base_odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_map_to_odom_tf(self, stamp) -> None:
        """Publish the map -> odom transform derived from pose and odometry."""
        if not self.publish_map_to_odom_tf:
            return
        if self.latest_map_base_pose is None or self.latest_odom_base is None:
            return

        map_frame_id = self.latest_map_base_pose.header.frame_id
        odom_frame_id = self.latest_odom_base.header.frame_id
        if not map_frame_id or not odom_frame_id:
            if not self.warned_missing_map_or_odom_frame:
                self.get_logger().warning(
                    "Map pose or base odometry is missing a frame_id; "
                    "cannot publish map->odom TF."
                )
                self.warned_missing_map_or_odom_frame = True
            return
        if map_frame_id == odom_frame_id:
            if not self.warned_map_odom_same_frame:
                self.get_logger().warning(
                    f"Map frame and odom frame are both '{map_frame_id}'. "
                    "Skipping map->odom TF publication."
                )
                self.warned_map_odom_same_frame = True
            return

        transform = self._compose_map_to_odom_transform(
            self.latest_map_base_pose,
            self.latest_odom_base,
        )
        transform.header.stamp = stamp
        self.tf_broadcaster.sendTransform(transform)

    def _compose_map_to_odom_transform(
        self,
        map_to_base_pose: PoseStamped,
        odom_to_base: Odometry,
    ) -> TransformStamped:
        """Compose map -> odom from map -> base and odom -> base."""
        map_rotation = _normalize_quaternion(
            (
                map_to_base_pose.pose.orientation.x,
                map_to_base_pose.pose.orientation.y,
                map_to_base_pose.pose.orientation.z,
                map_to_base_pose.pose.orientation.w,
            )
        )
        odom_rotation = _normalize_quaternion(
            (
                odom_to_base.pose.pose.orientation.x,
                odom_to_base.pose.pose.orientation.y,
                odom_to_base.pose.pose.orientation.z,
                odom_to_base.pose.pose.orientation.w,
            )
        )
        odom_inverse_rotation = _quaternion_inverse(odom_rotation)

        odom_translation = (
            odom_to_base.pose.pose.position.x,
            odom_to_base.pose.pose.position.y,
            odom_to_base.pose.pose.position.z,
        )
        base_to_odom_translation = _rotate_vector(
            odom_inverse_rotation,
            (-odom_translation[0], -odom_translation[1], -odom_translation[2]),
        )

        map_to_odom_rotation = _quaternion_multiply(map_rotation, odom_inverse_rotation)
        rotated_base_to_odom = _rotate_vector(map_rotation, base_to_odom_translation)

        transform = TransformStamped()
        transform.header.frame_id = map_to_base_pose.header.frame_id
        transform.child_frame_id = odom_to_base.header.frame_id
        transform.transform.translation.x = (
            map_to_base_pose.pose.position.x + rotated_base_to_odom[0]
        )
        transform.transform.translation.y = (
            map_to_base_pose.pose.position.y + rotated_base_to_odom[1]
        )
        transform.transform.translation.z = (
            map_to_base_pose.pose.position.z + rotated_base_to_odom[2]
        )
        transform.transform.rotation.x = map_to_odom_rotation[0]
        transform.transform.rotation.y = map_to_odom_rotation[1]
        transform.transform.rotation.z = map_to_odom_rotation[2]
        transform.transform.rotation.w = map_to_odom_rotation[3]
        return transform


def main(args=None) -> None:
    """Run the ZED base adapter node."""
    rclpy.init(args=args)
    node = ZedBaseAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
