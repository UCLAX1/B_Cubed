#!/usr/bin/env python3
"""Gate navigation velocity commands on localization and scan health."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def _zero_twist() -> Twist:
    """Build a zero-velocity Twist message."""
    return Twist()


def _is_nonzero(msg: Twist) -> bool:
    """Return true when any Twist component is meaningfully nonzero."""
    linear = msg.linear
    angular = msg.angular
    return any(
        abs(component) > 1e-4
        for component in (
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
        )
    )


class TwistSafetyGateNode(Node):
    """Publish `cmd_vel` only when localization and scan data are healthy."""

    def __init__(self) -> None:
        super().__init__("twist_safety_gate")

        self._declare_parameters()

        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.localized_topic = str(self.get_parameter("localized_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.require_localized = bool(self.get_parameter("require_localized").value)
        self.block_on_unknown_localization = bool(
            self.get_parameter("block_on_unknown_localization").value
        )
        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.publish_zero_when_blocked = bool(
            self.get_parameter("publish_zero_when_blocked").value
        )
        self.zero_if_cmd_times_out = bool(
            self.get_parameter("zero_if_cmd_times_out").value
        )

        self.localized_timeout = Duration(
            seconds=float(self.get_parameter("localized_timeout_sec").value)
        )
        self.scan_timeout = Duration(
            seconds=float(self.get_parameter("scan_timeout_sec").value)
        )
        self.cmd_timeout = Duration(
            seconds=float(self.get_parameter("cmd_timeout_sec").value)
        )
        self.status_publish_rate_hz = max(
            1.0,
            float(self.get_parameter("status_publish_rate_hz").value),
        )
        self.zero_publish_rate_hz = max(
            1.0,
            float(self.get_parameter("zero_publish_rate_hz").value),
        )

        self.localized_state = None
        self.last_localized_rx = None
        self.last_scan_rx = None
        self.last_cmd_rx = None
        self.last_cmd_nonzero = False
        self.zero_sent_since_timeout = False
        self.last_status_text = None
        self.last_status_summary = None

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(
            Twist,
            self.input_cmd_topic,
            self._cmd_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.localized_topic,
            self._localized_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self.create_timer(
            1.0 / self.status_publish_rate_hz,
            self._publish_status,
        )
        self.create_timer(
            1.0 / self.zero_publish_rate_hz,
            self._enforce_stop,
        )

        self.get_logger().info(
            "Gating navigation velocity commands from "
            f"{self.input_cmd_topic} to {self.output_cmd_topic}."
        )
        self.get_logger().info(
            "Safety requirements: "
            f"localized={self.require_localized}, "
            f"scan={self.require_scan}, "
            f"block_on_unknown_localization={self.block_on_unknown_localization}."
        )

    def _declare_parameters(self) -> None:
        """Declare runtime-configurable parameters."""
        self.declare_parameter("input_cmd_topic", "cmd_vel_nav")
        self.declare_parameter("output_cmd_topic", "cmd_vel")
        self.declare_parameter("localized_topic", "zed/is_localized")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("status_topic", "nav/cmd_vel_gate_status")

        self.declare_parameter("require_localized", True)
        self.declare_parameter("block_on_unknown_localization", True)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("publish_zero_when_blocked", True)
        self.declare_parameter("zero_if_cmd_times_out", True)

        self.declare_parameter("localized_timeout_sec", 1.0)
        self.declare_parameter("scan_timeout_sec", 1.0)
        self.declare_parameter("cmd_timeout_sec", 0.75)
        self.declare_parameter("status_publish_rate_hz", 2.0)
        self.declare_parameter("zero_publish_rate_hz", 5.0)

    def _localized_callback(self, msg: Bool) -> None:
        """Track whether upstream localization is currently healthy."""
        self.localized_state = bool(msg.data)
        self.last_localized_rx = self.get_clock().now()

    def _scan_callback(self, _msg: LaserScan) -> None:
        """Track freshness of the projected 2D scan."""
        self.last_scan_rx = self.get_clock().now()

    def _cmd_callback(self, msg: Twist) -> None:
        """Pass through velocity commands only when the robot is ready."""
        self.last_cmd_rx = self.get_clock().now()
        self.last_cmd_nonzero = _is_nonzero(msg)
        self.zero_sent_since_timeout = False

        gate_open, _ = self._gate_state()
        if gate_open:
            self.cmd_pub.publish(msg)
            return

        if self.publish_zero_when_blocked:
            self.cmd_pub.publish(_zero_twist())

    def _publish_status(self) -> None:
        """Publish a concise summary of the gate state."""
        gate_open, reason = self._gate_state()
        status_summary = (
            f"gate_open={gate_open}, "
            f"reason={reason}, "
            f"localized_state={self._localized_text()}"
        )
        status_text = (
            f"{status_summary}, "
            f"localized_age={self._age_text(self.last_localized_rx)}, "
            f"scan_age={self._age_text(self.last_scan_rx)}, "
            f"cmd_age={self._age_text(self.last_cmd_rx)}"
        )

        self.status_pub.publish(String(data=status_text))
        if status_summary != self.last_status_summary:
            self.get_logger().info(status_text)
            self.last_status_summary = status_summary
        self.last_status_text = status_text

    def _enforce_stop(self) -> None:
        """Continuously hold the robot stopped whenever the gate is closed."""
        gate_open, _ = self._gate_state()

        if not gate_open:
            if self.publish_zero_when_blocked:
                self.cmd_pub.publish(_zero_twist())
            self.zero_sent_since_timeout = False
            return

        if not self.zero_if_cmd_times_out or self.last_cmd_rx is None:
            return

        if not self.last_cmd_nonzero or self.zero_sent_since_timeout:
            return

        if self._is_fresh(self.last_cmd_rx, self.cmd_timeout):
            return

        self.cmd_pub.publish(_zero_twist())
        self.zero_sent_since_timeout = True

    def _gate_state(self) -> tuple[bool, str]:
        """Return whether the gate is open, plus the main blocking reason."""
        localization_ok, localization_reason = self._localized_ok()
        if not localization_ok:
            return False, localization_reason

        scan_ok, scan_reason = self._scan_ok()
        if not scan_ok:
            return False, scan_reason

        return True, "ready"

    def _localized_ok(self) -> tuple[bool, str]:
        """Validate localization freshness and state."""
        if not self.require_localized:
            return True, "localization_not_required"

        if self.last_localized_rx is None:
            if self.block_on_unknown_localization:
                return False, "waiting_for_localization_status"
            return True, "localization_status_unknown"

        if not self._is_fresh(self.last_localized_rx, self.localized_timeout):
            return False, "localization_status_stale"

        if not self.localized_state:
            return False, "localization_unhealthy"

        return True, "localized"

    def _scan_ok(self) -> tuple[bool, str]:
        """Validate scan freshness."""
        if not self.require_scan:
            return True, "scan_not_required"

        if self.last_scan_rx is None:
            return False, "waiting_for_scan"

        if not self._is_fresh(self.last_scan_rx, self.scan_timeout):
            return False, "scan_stale"

        return True, "scan_ready"

    def _is_fresh(self, stamp, timeout: Duration) -> bool:
        """Return true when the last message arrived within the timeout."""
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) <= timeout

    def _age_text(self, stamp) -> str:
        """Format the age of the last received message."""
        if stamp is None:
            return "never"
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return f"{age:.2f}s"

    def _localized_text(self) -> str:
        """Format the latest localization boolean for status output."""
        if self.localized_state is None:
            return "unknown"
        return "true" if self.localized_state else "false"


def main(args=None) -> None:
    """Run the twist safety gate node."""
    rclpy.init(args=args)
    node = TwistSafetyGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
