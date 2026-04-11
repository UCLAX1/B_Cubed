#!/usr/bin/env python3
"""ROS 2 node that bridges ZED wrapper positional-tracking topics."""

from copy import deepcopy
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


def _diagonal_covariance(diagonal: Sequence[float]) -> list[float]:
    """Expand a 6-value covariance diagonal into ROS's row-major 6x6 layout."""
    covariance = [0.0] * 36
    for index, value in enumerate(diagonal[:6]):
        covariance[index * 6 + index] = float(value)
    return covariance


class ZedPositionalTrackingNode(Node):
    """Subscribe to ZED wrapper localization topics and republish robot-friendly outputs."""

    def __init__(self) -> None:
        super().__init__("zed_positional_tracking")

        self._declare_parameters()

        self.input_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self.input_pose_cov_topic = str(self.get_parameter("input_pose_cov_topic").value)
        self.input_odom_topic = str(self.get_parameter("input_odom_topic").value)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.pose_cov_topic = str(self.get_parameter("pose_cov_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.localized_topic = str(self.get_parameter("localized_topic").value)

        self.stale_timeout = Duration(
            seconds=float(self.get_parameter("stale_timeout_sec").value)
        )
        self.status_publish_rate_hz = float(
            self.get_parameter("status_publish_rate_hz").value
        )
        self.require_odom_for_localized = bool(
            self.get_parameter("require_odom_for_localized").value
        )

        self.pose_covariance = self._read_covariance_parameter(
            "pose_covariance_diagonal",
            [0.05, 0.05, 0.10, 0.08, 0.08, 0.08],
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.pose_cov_topic,
            10,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.localized_pub = self.create_publisher(Bool, self.localized_topic, 10)

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

        self.latest_pose = None
        self.latest_pose_cov = None
        self.latest_odom = None

        self.last_pose_rx = None
        self.last_pose_cov_rx = None
        self.last_odom_rx = None
        self.last_status_text = None

        timer_period = 1.0 / max(self.status_publish_rate_hz, 1.0)
        self.status_timer = self.create_timer(timer_period, self._publish_status)

        self.get_logger().info(
            "Listening to ZED wrapper positional-tracking topics:"
            f" pose={self.input_pose_topic},"
            f" pose_with_covariance={self.input_pose_cov_topic},"
            f" odom={self.input_odom_topic}"
        )
        self.get_logger().info(
            "Republishing localization outputs to "
            f"{self.pose_topic}, {self.pose_cov_topic}, {self.odom_topic}, "
            f"{self.status_topic}, and {self.localized_topic}."
        )
        self.get_logger().info(
            "This node expects the ZED ROS 2 wrapper to have positional tracking enabled."
        )

    def _declare_parameters(self) -> None:
        """Declare runtime-tunable parameters."""
        self.declare_parameter("input_pose_topic", "/zed/zed_node/pose")
        self.declare_parameter(
            "input_pose_cov_topic",
            "/zed/zed_node/pose_with_covariance",
        )
        self.declare_parameter("input_odom_topic", "/zed/zed_node/odom")

        self.declare_parameter("pose_topic", "zed/pose")
        self.declare_parameter("pose_cov_topic", "zed/pose_with_covariance")
        self.declare_parameter("odom_topic", "zed/odom")
        self.declare_parameter("status_topic", "zed/tracking_status")
        self.declare_parameter("localized_topic", "zed/is_localized")

        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("status_publish_rate_hz", 2.0)
        self.declare_parameter("require_odom_for_localized", False)
        self.declare_parameter(
            "pose_covariance_diagonal",
            [0.05, 0.05, 0.10, 0.08, 0.08, 0.08],
        )

    def _read_covariance_parameter(
        self,
        parameter_name: str,
        fallback: Sequence[float],
    ) -> list[float]:
        """Validate a 6-value diagonal covariance parameter."""
        raw_value = list(self.get_parameter(parameter_name).value)
        if len(raw_value) != 6:
            self.get_logger().warning(
                f"Parameter '{parameter_name}' must contain 6 values. Falling back to "
                f"{list(fallback)}."
            )
            raw_value = list(fallback)
        return _diagonal_covariance(raw_value)

    def _pose_callback(self, msg: PoseStamped) -> None:
        """Forward pose messages from the wrapper and synthesize covariance if needed."""
        self.latest_pose = deepcopy(msg)
        self.last_pose_rx = self.get_clock().now()
        self.pose_pub.publish(msg)

        if not self._has_fresh_pose_covariance():
            self.pose_cov_pub.publish(self._build_pose_covariance(msg))

    def _pose_cov_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Forward wrapper pose-with-covariance and extract a plain pose view."""
        self.latest_pose_cov = deepcopy(msg)
        self.last_pose_cov_rx = self.get_clock().now()
        self.pose_cov_pub.publish(msg)
        if not self._is_fresh(self.last_pose_rx):
            self.pose_pub.publish(self._pose_from_covariance(msg))

    def _odom_callback(self, msg: Odometry) -> None:
        """Forward wrapper odometry."""
        self.latest_odom = deepcopy(msg)
        self.last_odom_rx = self.get_clock().now()
        self.odom_pub.publish(msg)

    def _build_pose_covariance(
        self,
        pose_msg: PoseStamped,
    ) -> PoseWithCovarianceStamped:
        """Create a PoseWithCovarianceStamped when the wrapper covariance topic is absent."""
        output = PoseWithCovarianceStamped()
        output.header = pose_msg.header
        output.pose.pose = pose_msg.pose
        output.pose.covariance = self.pose_covariance
        return output

    def _pose_from_covariance(
        self,
        pose_cov_msg: PoseWithCovarianceStamped,
    ) -> PoseStamped:
        """Extract a PoseStamped view from a PoseWithCovarianceStamped."""
        pose_msg = PoseStamped()
        pose_msg.header = pose_cov_msg.header
        pose_msg.pose = pose_cov_msg.pose.pose
        return pose_msg

    def _has_fresh_pose_covariance(self) -> bool:
        """Check whether the wrapper covariance topic has been seen recently."""
        return self._is_fresh(self.last_pose_cov_rx)

    def _is_fresh(self, stamp) -> bool:
        """Return true if a received message timestamp is still within the freshness window."""
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) <= self.stale_timeout

    def _age_text(self, stamp) -> str:
        """Report how old the last received message is."""
        if stamp is None:
            return "never"

        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return f"{age:.2f}s"

    def _publish_status(self) -> None:
        """Publish a concise health summary of the wrapper tracking inputs."""
        pose_ok = self._is_fresh(self.last_pose_rx) or self._is_fresh(self.last_pose_cov_rx)
        odom_ok = self._is_fresh(self.last_odom_rx)

        localized = pose_ok and (odom_ok or not self.require_odom_for_localized)

        status_text = (
            f"localized={localized}, "
            f"pose_fresh={pose_ok}, "
            f"odom_fresh={odom_ok}, "
            f"pose_age={self._age_text(self.last_pose_rx)}, "
            f"pose_cov_age={self._age_text(self.last_pose_cov_rx)}, "
            f"odom_age={self._age_text(self.last_odom_rx)}"
        )

        self.status_pub.publish(String(data=status_text))
        self.localized_pub.publish(Bool(data=localized))

        if status_text != self.last_status_text:
            self.get_logger().info(status_text)
            self.last_status_text = status_text


def main(args=None) -> None:
    """Run the ZED wrapper positional tracking bridge node."""
    rclpy.init(args=args)
    node = ZedPositionalTrackingNode()
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
