#!/usr/bin/env python3
"""ROS 2 node that bridges ZED wrapper positional-tracking topics."""

from collections import deque
from copy import deepcopy
import math
import os
from typing import Sequence

import cv2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


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
        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.input_image_is_compressed = bool(
            self.get_parameter("input_image_is_compressed").value
        )

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.pose_cov_topic = str(self.get_parameter("pose_cov_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.localized_topic = str(self.get_parameter("localized_topic").value)
        self.visualization_image_topic = str(
            self.get_parameter("visualization_image_topic").value
        )
        self.visualization_window_name = str(
            self.get_parameter("visualization_window_name").value
        )

        self.stale_timeout = Duration(
            seconds=float(self.get_parameter("stale_timeout_sec").value)
        )
        self.status_publish_rate_hz = float(
            self.get_parameter("status_publish_rate_hz").value
        )
        self.require_odom_for_localized = bool(
            self.get_parameter("require_odom_for_localized").value
        )
        self.enable_visualization = bool(
            self.get_parameter("enable_visualization").value
        )
        self.show_visualization_window = bool(
            self.get_parameter("show_visualization_window").value
        )
        self.publish_visualization_image = bool(
            self.get_parameter("publish_visualization_image").value
        )
        self.visualization_rate_hz = float(
            self.get_parameter("visualization_rate_hz").value
        )
        self.path_history_length = max(
            2,
            int(self.get_parameter("path_history_length").value),
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
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.pose_cov_topic,
            10,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.localized_pub = self.create_publisher(Bool, self.localized_topic, 10)
        self.visualization_pub = None
        if self.publish_visualization_image:
            self.visualization_pub = self.create_publisher(
                CompressedImage,
                self.visualization_image_topic,
                1,
            )

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

        self.bridge = CvBridge() if CvBridge is not None else None
        self.image_sub = None
        if self.enable_visualization:
            if self.input_image_is_compressed:
                self.image_sub = self.create_subscription(
                    CompressedImage,
                    self.input_image_topic,
                    self._compressed_image_callback,
                    image_qos,
                )
            else:
                self.image_sub = self.create_subscription(
                    Image,
                    self.input_image_topic,
                    self._image_callback,
                    image_qos,
                )

        self.latest_pose = None
        self.latest_pose_cov = None
        self.latest_odom = None
        self.latest_frame = None
        self.latest_image_header = None

        self.last_pose_rx = None
        self.last_pose_cov_rx = None
        self.last_odom_rx = None
        self.last_image_rx = None
        self.last_status_text = None

        self.pose_history = deque(maxlen=self.path_history_length)
        self.warned_image_encodings = set()
        self.window_ready = False

        timer_period = 1.0 / max(self.status_publish_rate_hz, 1.0)
        self.status_timer = self.create_timer(timer_period, self._publish_status)

        self.visualization_timer = None
        if self.enable_visualization and (
            self.show_visualization_window or self.publish_visualization_image
        ):
            if self.show_visualization_window:
                self._maybe_enable_visualization_window()
            viz_period = 1.0 / max(self.visualization_rate_hz, 1.0)
            self.visualization_timer = self.create_timer(
                viz_period,
                self._render_visualization,
            )

        self.get_logger().info(
            "Listening to ZED wrapper positional-tracking topics:"
            f" pose={self.input_pose_topic},"
            f" pose_with_covariance={self.input_pose_cov_topic},"
            f" odom={self.input_odom_topic}"
        )
        self.get_logger().info(
            "Republishing localization outputs to "
            f"{self.pose_topic}, {self.pose_cov_topic}, {self.odom_topic}, "
            f"{self.path_topic}, {self.status_topic}, and {self.localized_topic}."
        )
        if self.enable_visualization:
            self.get_logger().info(
                "Visualization enabled with camera topic "
                f"{self.input_image_topic} "
                f"(compressed={self.input_image_is_compressed})."
            )
            if self.publish_visualization_image:
                self.get_logger().info(
                    "Publishing annotated localization view to "
                    f"{self.visualization_image_topic}."
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
        self.declare_parameter(
            "input_image_topic",
            "/zed/zed_node/rgb/color/rect/image/compressed",
        )
        self.declare_parameter("input_image_is_compressed", True)

        self.declare_parameter("pose_topic", "zed/pose")
        self.declare_parameter("pose_cov_topic", "zed/pose_with_covariance")
        self.declare_parameter("odom_topic", "zed/odom")
        self.declare_parameter("path_topic", "zed/path")
        self.declare_parameter("status_topic", "zed/tracking_status")
        self.declare_parameter("localized_topic", "zed/is_localized")
        self.declare_parameter(
            "visualization_image_topic",
            "zed/localization_view/compressed",
        )
        self.declare_parameter("visualization_window_name", "ZED Localization")

        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("status_publish_rate_hz", 2.0)
        self.declare_parameter("require_odom_for_localized", False)
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("show_visualization_window", True)
        self.declare_parameter("publish_visualization_image", True)
        self.declare_parameter("visualization_rate_hz", 10.0)
        self.declare_parameter("path_history_length", 200)
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
        self._record_pose(msg)

        if not self._has_fresh_pose_covariance():
            self.pose_cov_pub.publish(self._build_pose_covariance(msg))

    def _pose_cov_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Forward wrapper pose-with-covariance and extract a plain pose view."""
        self.latest_pose_cov = deepcopy(msg)
        self.last_pose_cov_rx = self.get_clock().now()
        self.pose_cov_pub.publish(msg)
        if not self._is_fresh(self.last_pose_rx):
            pose_msg = self._pose_from_covariance(msg)
            self.pose_pub.publish(pose_msg)
            self._record_pose(pose_msg)

    def _odom_callback(self, msg: Odometry) -> None:
        """Forward wrapper odometry."""
        self.latest_odom = deepcopy(msg)
        self.last_odom_rx = self.get_clock().now()
        self.odom_pub.publish(msg)

    def _compressed_image_callback(self, msg: CompressedImage) -> None:
        """Decode a compressed camera image for localization visualization."""
        buffer = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode compressed ZED image frame.")
            return
        self.latest_frame = frame
        self.latest_image_header = msg.header
        self.last_image_rx = self.get_clock().now()

    def _image_callback(self, msg: Image) -> None:
        """Decode a raw camera image for localization visualization."""
        frame = self._image_msg_to_bgr(msg)
        if frame is None:
            return
        self.latest_frame = frame
        self.latest_image_header = msg.header
        self.last_image_rx = self.get_clock().now()

    def _image_msg_to_bgr(self, msg: Image) -> np.ndarray | None:
        """Convert a raw ROS image message to a BGR OpenCV frame."""
        if self.bridge is not None:
            return np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))

        encoding = (msg.encoding or "").lower()
        if encoding not in {"bgr8", "rgb8", "bgra8", "rgba8", "mono8"}:
            if encoding not in self.warned_image_encodings:
                self.get_logger().warning(
                    f"Unsupported raw image encoding '{msg.encoding}'. "
                    "Install cv_bridge or switch to a compressed image topic."
                )
                self.warned_image_encodings.add(encoding)
            return None

        row = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))

        if encoding in {"bgr8", "rgb8"}:
            channels = 3
        elif encoding in {"bgra8", "rgba8"}:
            channels = 4
        else:
            channels = 1

        frame = row[:, : msg.width * channels].reshape((msg.height, msg.width, channels))

        if encoding == "bgr8":
            return frame.copy()
        if encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)

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

    def _record_pose(self, pose_msg: PoseStamped) -> None:
        """Append a pose sample to the trajectory history and publish a Path view."""
        if self.pose_history and self.pose_history[-1].header.frame_id != pose_msg.header.frame_id:
            self.get_logger().warning(
                "Pose frame changed from "
                f"{self.pose_history[-1].header.frame_id} to {pose_msg.header.frame_id}. "
                "Clearing trajectory history."
            )
            self.pose_history.clear()

        if self.pose_history:
            previous = self.pose_history[-1].pose.position
            current = pose_msg.pose.position
            dx = current.x - previous.x
            dy = current.y - previous.y
            dz = current.z - previous.z
            if (dx * dx + dy * dy + dz * dz) < 1e-6:
                return

        self.pose_history.append(deepcopy(pose_msg))
        self._publish_path()

    def _publish_path(self) -> None:
        """Publish the recent localization trail as a nav_msgs/Path."""
        if not self.pose_history:
            return

        path_msg = Path()
        path_msg.header = self.pose_history[-1].header
        path_msg.poses = [deepcopy(pose) for pose in self.pose_history]
        self.path_pub.publish(path_msg)

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

    def _localization_state(self) -> tuple[bool, bool, bool]:
        """Compute whether the wrapper data currently indicates a localized camera."""
        pose_ok = self._is_fresh(self.last_pose_rx) or self._is_fresh(self.last_pose_cov_rx)
        odom_ok = self._is_fresh(self.last_odom_rx)
        localized = pose_ok and (odom_ok or not self.require_odom_for_localized)
        return localized, pose_ok, odom_ok

    def _publish_status(self) -> None:
        """Publish a concise health summary of the wrapper tracking inputs."""
        localized, pose_ok, odom_ok = self._localization_state()

        status_text = (
            f"localized={localized}, "
            f"pose_fresh={pose_ok}, "
            f"odom_fresh={odom_ok}, "
            f"pose_age={self._age_text(self.last_pose_rx)}, "
            f"pose_cov_age={self._age_text(self.last_pose_cov_rx)}, "
            f"odom_age={self._age_text(self.last_odom_rx)}, "
            f"image_age={self._age_text(self.last_image_rx)}"
        )

        self.status_pub.publish(String(data=status_text))
        self.localized_pub.publish(Bool(data=localized))

        if status_text != self.last_status_text:
            self.get_logger().info(status_text)
            self.last_status_text = status_text

    def _maybe_enable_visualization_window(self) -> None:
        """Create an OpenCV visualization window when a GUI display is available."""
        if not self.show_visualization_window:
            return

        if not os.environ.get("DISPLAY"):
            self.get_logger().warning(
                "DISPLAY is not set, so the localization window is disabled. "
                f"Use the published image topic {self.visualization_image_topic} instead."
            )
            self.show_visualization_window = False
            return

        try:
            cv2.namedWindow(self.visualization_window_name, cv2.WINDOW_NORMAL)
            self.window_ready = True
        except cv2.error as exc:
            self.get_logger().warning(
                "Failed to open the localization window. "
                f"Continuing without it: {exc}"
            )
            self.show_visualization_window = False

    def _render_visualization(self) -> None:
        """Render a live camera view with localization overlays."""
        frame = self._base_visualization_frame()
        annotated = self._annotate_frame(frame)

        if self.visualization_pub is not None:
            self._publish_visualization_image(annotated)

        if self.show_visualization_window and self.window_ready:
            cv2.imshow(self.visualization_window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                rclpy.shutdown()

    def _base_visualization_frame(self) -> np.ndarray:
        """Return the latest camera frame, or a placeholder when no image has arrived."""
        if self.latest_frame is not None:
            return self.latest_frame.copy()

        placeholder = np.full((720, 1280, 3), 18, dtype=np.uint8)
        cv2.putText(
            placeholder,
            "Waiting for ZED camera image...",
            (40, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            placeholder,
            f"Topic: {self.input_image_topic}",
            (40, 130),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )
        return placeholder

    def _annotate_frame(self, frame: np.ndarray) -> np.ndarray:
        """Overlay localization state and trajectory on top of the camera frame."""
        localized, pose_ok, odom_ok = self._localization_state()
        overlay = frame.copy()

        panel_width = min(
            430,
            max(220, frame.shape[1] // 3),
            max(120, frame.shape[1] - 24),
        )
        panel_bottom = min(frame.shape[0] - 12, 240)
        cv2.rectangle(overlay, (12, 12), (12 + panel_width, panel_bottom), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.35, frame, 0.65, 0.0, frame)

        pose_view = self._latest_pose_view()
        header_frame = "n/a"
        position_text = "position: waiting for pose"
        yaw_text = "yaw: waiting for pose"
        if pose_view is not None:
            header_frame = pose_view.header.frame_id or "n/a"
            pos = pose_view.pose.position
            yaw_deg = math.degrees(self._yaw_from_quaternion(pose_view.pose.orientation))
            position_text = f"x={pos.x:+.2f}  y={pos.y:+.2f}  z={pos.z:+.2f} m"
            yaw_text = f"yaw={yaw_deg:+.1f} deg"

        velocity_text = "velocity: waiting for odom"
        if self.latest_odom is not None:
            twist = self.latest_odom.twist.twist
            velocity_text = (
                f"vx={twist.linear.x:+.2f}  vy={twist.linear.y:+.2f}  "
                f"wz={twist.angular.z:+.2f}"
            )

        status_label = "LOCALIZED" if localized else "TRACKING LOST"
        status_color = (40, 200, 80) if localized else (70, 90, 255)

        lines = [
            (status_label, 1.0, status_color),
            (f"frame: {header_frame}", 0.7, (255, 255, 255)),
            (position_text, 0.7, (255, 255, 255)),
            (yaw_text, 0.7, (255, 255, 255)),
            (velocity_text, 0.7, (255, 255, 255)),
            (
                f"pose_fresh={pose_ok}  odom_fresh={odom_ok}",
                0.7,
                (255, 255, 255),
            ),
            (
                f"ages pose={self._age_text(self.last_pose_rx)}  "
                f"cov={self._age_text(self.last_pose_cov_rx)}",
                0.65,
                (215, 215, 215),
            ),
            (
                f"odom={self._age_text(self.last_odom_rx)}  "
                f"image={self._age_text(self.last_image_rx)}",
                0.65,
                (215, 215, 215),
            ),
        ]

        y = 48
        max_text_y = max(40, frame.shape[0] - 48)
        for text, scale, color in lines:
            if y > max_text_y:
                break
            cv2.putText(
                frame,
                text,
                (28, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                color,
                2,
                cv2.LINE_AA,
            )
            y += 28 if scale <= 0.7 else 34

        inset_size = int(
            min(
                300,
                max(160, min(frame.shape[0], frame.shape[1]) // 3),
                max(120, frame.shape[1] - 24),
                max(120, frame.shape[0] - 24),
            )
        )
        trajectory_inset = self._trajectory_inset(inset_size, inset_size)
        inset_h, inset_w = trajectory_inset.shape[:2]
        top = 12
        left = max(12, frame.shape[1] - inset_w - 12)
        frame[top : top + inset_h, left : left + inset_w] = trajectory_inset

        cv2.putText(
            frame,
            "Camera view with localization overlay",
            (28, frame.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return frame

    def _trajectory_inset(self, width: int, height: int) -> np.ndarray:
        """Create a top-down trajectory view using the recent camera poses."""
        inset = np.full((height, width, 3), 28, dtype=np.uint8)
        cv2.rectangle(inset, (0, 0), (width - 1, height - 1), (160, 160, 160), 1)
        cv2.putText(
            inset,
            "Top-down trajectory (x/y)",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if not self.pose_history:
            cv2.putText(
                inset,
                "Waiting for pose data...",
                (24, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (220, 220, 220),
                2,
                cv2.LINE_AA,
            )
            return inset

        margin = 28
        points = np.array(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in self.pose_history
            ],
            dtype=np.float32,
        )

        min_xy = points.min(axis=0)
        max_xy = points.max(axis=0)
        center_xy = (min_xy + max_xy) * 0.5
        span_xy = np.maximum(max_xy - min_xy, np.array([1.0, 1.0], dtype=np.float32))
        span_xy *= 1.15

        plot_width = max(1.0, float(width - 2 * margin))
        plot_height = max(1.0, float(height - 2 * margin - 20))

        pixels = []
        for x_pos, y_pos in points:
            px = int(
                margin
                + ((x_pos - center_xy[0]) / span_xy[0] + 0.5) * plot_width
            )
            py = int(
                height
                - margin
                - ((y_pos - center_xy[1]) / span_xy[1] + 0.5) * plot_height
            )
            px = int(np.clip(px, 0, width - 1))
            py = int(np.clip(py, 0, height - 1))
            pixels.append((px, py))

        for px in range(margin, width - margin, 40):
            cv2.line(inset, (px, margin), (px, height - margin), (55, 55, 55), 1)
        for py in range(margin, height - margin, 40):
            cv2.line(inset, (margin, py), (width - margin, py), (55, 55, 55), 1)

        for start, end in zip(pixels, pixels[1:]):
            cv2.line(inset, start, end, (85, 210, 255), 2, cv2.LINE_AA)

        latest_pose = self.pose_history[-1]
        latest_point = pixels[-1]
        yaw = self._yaw_from_quaternion(latest_pose.pose.orientation)
        arrow_length = 18
        arrow_tip = (
            int(latest_point[0] + arrow_length * math.cos(yaw)),
            int(latest_point[1] - arrow_length * math.sin(yaw)),
        )
        cv2.circle(inset, latest_point, 5, (40, 200, 80), -1)
        cv2.arrowedLine(
            inset,
            latest_point,
            arrow_tip,
            (40, 200, 80),
            2,
            cv2.LINE_AA,
            tipLength=0.3,
        )

        pos = latest_pose.pose.position
        cv2.putText(
            inset,
            f"now x={pos.x:+.2f} y={pos.y:+.2f}",
            (12, height - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return inset

    def _latest_pose_view(self) -> PoseStamped | None:
        """Return the newest pose view currently available for visualization."""
        if self.latest_pose is not None and self._is_fresh(self.last_pose_rx):
            return self.latest_pose
        if self.latest_pose_cov is not None and self._is_fresh(self.last_pose_cov_rx):
            return self._pose_from_covariance(self.latest_pose_cov)
        return None

    def _yaw_from_quaternion(self, orientation) -> float:
        """Compute yaw from a geometry_msgs quaternion."""
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_visualization_image(self, frame: np.ndarray) -> None:
        """Publish the annotated visualization frame as a compressed image topic."""
        success, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not success:
            self.get_logger().warning("Failed to encode localization visualization frame.")
            return

        msg = CompressedImage()
        if self.latest_image_header is not None:
            msg.header = self.latest_image_header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.visualization_pub.publish(msg)


def main(args=None) -> None:
    """Run the ZED wrapper positional tracking bridge node."""
    rclpy.init(args=args)
    node = ZedPositionalTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
