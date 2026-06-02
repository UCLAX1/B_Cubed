#!/usr/bin/env python3
"""Serve a local web console for viewing a map and requesting Nav2 plans."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import threading
import time
from typing import Any, Callable
import zlib

from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from nav_planning_console.errors import PlanningConsoleError
from nav_planning_console.web_server import PlanningConsoleHttpServer


def _yaw_from_quaternion(quaternion: Any) -> float:
    """Return yaw from a geometry_msgs-style quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return x/y/z/w quaternion values for a planar yaw."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a float between minimum and maximum."""
    return max(minimum, min(float(value), maximum))


def _wrap_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _rotate_start_frame_to_body_frame(
    x_value: float,
    y_value: float,
    relative_yaw: float,
) -> tuple[float, float]:
    """Rotate a startup-frame XY vector into the robot body frame."""
    cos_yaw = math.cos(relative_yaw)
    sin_yaw = math.sin(relative_yaw)
    return (
        cos_yaw * float(x_value) + sin_yaw * float(y_value),
        -sin_yaw * float(x_value) + cos_yaw * float(y_value),
    )


def _coerce_bool(value: Any) -> bool:
    """Convert common HTTP/launch truthy values to bool."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _content_type_from_compressed_format(format_text: str) -> str:
    """Return an HTTP content type for a sensor_msgs/CompressedImage format."""
    normalized = format_text.lower()
    if "png" in normalized:
        return "image/png"
    if "jpg" in normalized or "jpeg" in normalized:
        return "image/jpeg"
    return "application/octet-stream"


def _duration_to_float(duration_msg: Any) -> float:
    """Convert a ROS duration message into seconds."""
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


def _time_to_float(stamp: Any) -> float:
    """Convert a ROS time message into seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _goal_status_text(status: int) -> str:
    """Return a readable action goal status."""
    names = {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "active",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "failed",
    }
    return names.get(status, f"status {status}")


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    """Create a PNG chunk."""
    crc_input = chunk_type + payload
    return (
        struct.pack(">I", len(payload))
        + chunk_type
        + payload
        + struct.pack(">I", zlib.crc32(crc_input) & 0xFFFFFFFF)
    )


def _encode_rgba_png(width: int, height: int, rgba_data: bytes) -> bytes:
    """Encode RGBA bytes as a PNG image using only the standard library."""
    row_stride = width * 4
    filtered_rows = bytearray()
    for row_index in range(height):
        start = row_index * row_stride
        filtered_rows.append(0)
        filtered_rows.extend(rgba_data[start:start + row_stride])

    header = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(bytes(filtered_rows), level=6))
        + _png_chunk(b"IEND", b"")
    )


def _render_map_png(map_msg: OccupancyGrid) -> bytes:
    """Render an OccupancyGrid as a browser-friendly PNG snapshot."""
    width = int(map_msg.info.width)
    height = int(map_msg.info.height)
    if width <= 0 or height <= 0:
        return _encode_rgba_png(1, 1, b"\x00\x00\x00\x00")

    data = map_msg.data
    rgba = bytearray(width * height * 4)

    for image_y in range(height):
        map_y = height - 1 - image_y
        source_row = map_y * width
        target_row = image_y * width * 4
        for x_value in range(width):
            occupancy = int(data[source_row + x_value])
            target = target_row + x_value * 4

            if occupancy < 0:
                color = (205, 211, 218)
            elif occupancy >= 65:
                shade = max(24, 96 - int((occupancy - 65) * 1.5))
                color = (shade, shade + 6, shade + 14)
            else:
                shade = max(232, 252 - int(occupancy * 0.3))
                color = (shade, shade, shade - 1)

            rgba[target] = color[0]
            rgba[target + 1] = color[1]
            rgba[target + 2] = color[2]
            rgba[target + 3] = 255

    return _encode_rgba_png(width, height, bytes(rgba))


def _path_to_points(path_msg: Any) -> list[dict[str, float]]:
    """Convert a nav_msgs/Path into browser overlay points."""
    points = []
    for pose_msg in path_msg.poses:
        pose = pose_msg.pose
        points.append(
            {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw": _yaw_from_quaternion(pose.orientation),
            }
        )
    return points


def _pose_to_payload(pose_msg: Any) -> dict[str, float]:
    """Convert a geometry_msgs/Pose into a planar payload."""
    return {
        "x": float(pose_msg.position.x),
        "y": float(pose_msg.position.y),
        "z": float(pose_msg.position.z),
        "yaw": _yaw_from_quaternion(pose_msg.orientation),
    }


def _world_to_map_grid(
    map_msg: OccupancyGrid,
    x_value: float,
    y_value: float,
) -> tuple[float, float]:
    """Convert world coordinates into map grid coordinates."""
    origin = map_msg.info.origin
    yaw = _yaw_from_quaternion(origin.orientation)
    dx = x_value - float(origin.position.x)
    dy = y_value - float(origin.position.y)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    resolution = float(map_msg.info.resolution)
    return (
        (cos_yaw * dx + sin_yaw * dy) / resolution,
        (-sin_yaw * dx + cos_yaw * dy) / resolution,
    )


def _point_in_map_bounds(map_msg: OccupancyGrid, x_value: float, y_value: float) -> bool:
    """Return whether a world point is inside the current OccupancyGrid bounds."""
    if map_msg.info.resolution <= 0.0:
        return False
    grid_x, grid_y = _world_to_map_grid(map_msg, x_value, y_value)
    return (
        0.0 <= grid_x < float(map_msg.info.width)
        and 0.0 <= grid_y < float(map_msg.info.height)
    )


def _world_to_map_cell(
    map_msg: OccupancyGrid,
    x_value: float,
    y_value: float,
) -> tuple[int, int] | None:
    """Convert a world point to integer map-cell indices."""
    if map_msg.info.resolution <= 0.0:
        return None
    grid_x, grid_y = _world_to_map_grid(map_msg, x_value, y_value)
    cell_x = math.floor(grid_x)
    cell_y = math.floor(grid_y)
    if (
        cell_x < 0
        or cell_y < 0
        or cell_x >= int(map_msg.info.width)
        or cell_y >= int(map_msg.info.height)
    ):
        return None
    return int(cell_x), int(cell_y)


def _map_cell_to_world(
    map_msg: OccupancyGrid,
    cell_x: int,
    cell_y: int,
) -> tuple[float, float]:
    """Convert map-cell indices to the world point at the cell center."""
    origin = map_msg.info.origin
    yaw = _yaw_from_quaternion(origin.orientation)
    resolution = float(map_msg.info.resolution)
    map_x = (float(cell_x) + 0.5) * resolution
    map_y = (float(cell_y) + 0.5) * resolution
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        float(origin.position.x) + cos_yaw * map_x - sin_yaw * map_y,
        float(origin.position.y) + sin_yaw * map_x + cos_yaw * map_y,
    )


def _cell_occupancy(map_msg: OccupancyGrid, cell_x: int, cell_y: int) -> int | None:
    """Return the occupancy value for a map cell, or None when out of bounds."""
    width = int(map_msg.info.width)
    height = int(map_msg.info.height)
    if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
        return None
    index = cell_y * width + cell_x
    if index < 0 or index >= len(map_msg.data):
        return None
    return int(map_msg.data[index])


def _cell_is_free(
    map_msg: OccupancyGrid,
    cell_x: int,
    cell_y: int,
    occupied_threshold: int,
    treat_unknown_as_occupied: bool,
) -> bool:
    """Return whether a map cell can be used as a planner start cell."""
    occupancy = _cell_occupancy(map_msg, cell_x, cell_y)
    if occupancy is None:
        return False
    if occupancy < 0:
        return not treat_unknown_as_occupied
    return occupancy < occupied_threshold


def _cell_has_clearance(
    map_msg: OccupancyGrid,
    cell_x: int,
    cell_y: int,
    clearance_cells: int,
    occupied_threshold: int,
    treat_unknown_as_occupied: bool,
) -> bool:
    """Return whether a cell is far enough from occupied cells."""
    if not _cell_is_free(
        map_msg,
        cell_x,
        cell_y,
        occupied_threshold,
        treat_unknown_as_occupied,
    ):
        return False

    if clearance_cells <= 0:
        return True

    radius_squared = clearance_cells * clearance_cells
    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            if dx * dx + dy * dy > radius_squared:
                continue
            if not _cell_is_free(
                map_msg,
                cell_x + dx,
                cell_y + dy,
                occupied_threshold,
                treat_unknown_as_occupied,
            ):
                return False
    return True


def _find_nearest_free_start_pose(
    map_msg: OccupancyGrid,
    start_pose: dict[str, Any],
    search_radius_m: float,
    clearance_m: float,
    occupied_threshold: int,
    treat_unknown_as_occupied: bool,
) -> tuple[dict[str, Any] | None, float]:
    """Find the nearest nearby free start pose for Nav2 planning."""
    start_cell = _world_to_map_cell(
        map_msg,
        float(start_pose["x"]),
        float(start_pose["y"]),
    )
    if start_cell is None or map_msg.info.resolution <= 0.0:
        return None, 0.0

    resolution = float(map_msg.info.resolution)
    search_cells = max(0, int(math.ceil(search_radius_m / resolution)))
    clearance_cells = max(0, int(math.ceil(clearance_m / resolution)))
    start_x, start_y = start_cell

    if _cell_has_clearance(
        map_msg,
        start_x,
        start_y,
        clearance_cells,
        occupied_threshold,
        treat_unknown_as_occupied,
    ):
        return dict(start_pose), 0.0

    candidate_offsets = []
    for dy in range(-search_cells, search_cells + 1):
        for dx in range(-search_cells, search_cells + 1):
            distance_cells_squared = dx * dx + dy * dy
            if distance_cells_squared > search_cells * search_cells:
                continue
            candidate_offsets.append((distance_cells_squared, dx, dy))
    candidate_offsets.sort()

    for _, dx, dy in candidate_offsets:
        cell_x = start_x + dx
        cell_y = start_y + dy
        if not _cell_has_clearance(
            map_msg,
            cell_x,
            cell_y,
            clearance_cells,
            occupied_threshold,
            treat_unknown_as_occupied,
        ):
            continue

        x_value, y_value = _map_cell_to_world(map_msg, cell_x, cell_y)
        distance_m = math.hypot(
            x_value - float(start_pose["x"]),
            y_value - float(start_pose["y"]),
        )
        adjusted_pose = dict(start_pose)
        adjusted_pose["x"] = x_value
        adjusted_pose["y"] = y_value
        return adjusted_pose, distance_m

    return None, 0.0


class NavPlanningConsoleNode(Node):
    """Bridge ROS map, TF, and Nav2 planning into a local web console."""

    def __init__(self) -> None:
        super().__init__("nav_planning_console")
        self._declare_parameters()

        self.web_host = str(self.get_parameter("web_host").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.planner_action_name = str(
            self.get_parameter("planner_action_name").value
        )
        self.navigator_action_name = str(
            self.get_parameter("navigator_action_name").value
        )
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.manual_control_enabled = bool(
            self.get_parameter("manual_control_enabled").value
        )
        self.manual_max_linear_velocity = max(
            0.0,
            float(self.get_parameter("manual_max_linear_velocity").value),
        )
        self.manual_max_angular_velocity = max(
            0.0,
            float(self.get_parameter("manual_max_angular_velocity").value),
        )
        self.manual_field_relative_enabled = bool(
            self.get_parameter("manual_field_relative_enabled").value
        )
        self.manual_orientation_topic = str(
            self.get_parameter("manual_orientation_topic").value
        )
        self.manual_orientation_timeout_sec = max(
            0.0,
            float(self.get_parameter("manual_orientation_timeout_sec").value),
        )
        self.person_tracking_control_enabled = bool(
            self.get_parameter("person_tracking_control_enabled").value
        )
        self.person_tracking_node_name = str(
            self.get_parameter("person_tracking_node_name").value
        ).lstrip("/")
        self.person_tracking_image_topic = str(
            self.get_parameter("person_tracking_image_topic").value
        )
        self.person_tracking_engine_path = str(
            self.get_parameter("person_tracking_engine_path").value
        )
        self.person_tracking_show_window = bool(
            self.get_parameter("person_tracking_show_window").value
        )
        self.person_tracking_publish_annotated_image = bool(
            self.get_parameter("person_tracking_publish_annotated_image").value
        )
        self.person_tracking_detection_topic = str(
            self.get_parameter("person_tracking_detection_topic").value
        )
        self.person_tracking_annotated_image_topic = str(
            self.get_parameter("person_tracking_annotated_image_topic").value
        )
        self.person_tracking_confidence_threshold = float(
            self.get_parameter("person_tracking_confidence_threshold").value
        )
        self.person_tracking_nms_threshold = float(
            self.get_parameter("person_tracking_nms_threshold").value
        )
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.planner_server_timeout_sec = float(
            self.get_parameter("planner_server_timeout_sec").value
        )
        self.navigator_server_timeout_sec = float(
            self.get_parameter("navigator_server_timeout_sec").value
        )
        self.planning_timeout_sec = float(
            self.get_parameter("planning_timeout_sec").value
        )
        self.navigation_plane_z = float(
            self.get_parameter("navigation_plane_z").value
        )
        self.pose_update_period_sec = float(
            self.get_parameter("pose_update_period_sec").value
        )
        self.start_pose_rescue_enabled = bool(
            self.get_parameter("start_pose_rescue_enabled").value
        )
        self.start_pose_rescue_radius = max(
            0.0,
            float(self.get_parameter("start_pose_rescue_radius").value),
        )
        self.start_pose_rescue_clearance = max(
            0.0,
            float(self.get_parameter("start_pose_rescue_clearance").value),
        )
        self.start_pose_occupied_threshold = int(
            self.get_parameter("start_pose_occupied_threshold").value
        )
        self.start_pose_unknown_is_occupied = bool(
            self.get_parameter("start_pose_unknown_is_occupied").value
        )
        static_dir_param = str(self.get_parameter("static_dir").value)
        self.static_dir = self._resolve_static_dir(static_dir_param)

        self._lock = threading.RLock()
        self._map_msg: OccupancyGrid | None = None
        self._map_png: bytes | None = None
        self._map_revision = 0
        self._map_received_time = 0.0
        self._pose: dict[str, Any] | None = None
        self._pose_error = "Waiting for TF."
        self._path_points: list[dict[str, float]] = []
        self._path_revision = 0
        self._goal: dict[str, float] | None = None
        self._planner_status = "idle"
        self._planner_error = ""
        self._navigation_goal: dict[str, float] | None = None
        self._navigation_status = "idle"
        self._navigation_error = ""
        self._navigation_feedback: dict[str, Any] = {}
        self._navigation_goal_handle: Any | None = None
        self._navigation_goal_token = 0
        self._manual_command: dict[str, Any] = {
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
            "requested_linear_x": 0.0,
            "requested_linear_y": 0.0,
            "field_relative": False,
            "heading_relative_yaw": None,
            "active": False,
            "stamp": 0.0,
        }
        self._manual_heading_yaw: float | None = None
        self._manual_heading_reference_yaw: float | None = None
        self._manual_heading_relative_yaw: float | None = None
        self._manual_heading_frame_id = ""
        self._manual_heading_stamp: float | None = None
        self._manual_heading_received_time = 0.0
        self._person_tracking_process: subprocess.Popen[bytes] | None = None
        self._person_tracking_control_status = "stopped"
        self._person_tracking_error = ""
        self._person_tracking_image: bytes | None = None
        self._person_tracking_image_content_type = "image/jpeg"
        self._person_tracking_image_revision = 0
        self._person_tracking_image_received_time = 0.0
        self._person_tracking_image_stamp: float | None = None
        self._person_tracking_detections: list[dict[str, Any]] = []
        self._person_tracking_detections_received_time = 0.0

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_timer = self.create_timer(
            self.pose_update_period_sec,
            self._update_pose_from_tf,
        )
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            self.planner_action_name,
        )
        self.navigator_client = ActionClient(
            self,
            NavigateToPose,
            self.navigator_action_name,
        )
        self.manual_cmd_pub = self.create_publisher(
            Twist,
            self.manual_cmd_topic,
            10,
        )
        if self.manual_field_relative_enabled and self.manual_orientation_topic:
            self.manual_orientation_sub = self.create_subscription(
                Imu,
                self.manual_orientation_topic,
                self._manual_orientation_callback,
                10,
            )
        self.person_tracking_detection_sub = self.create_subscription(
            String,
            self.person_tracking_detection_topic,
            self._person_tracking_detection_callback,
            10,
        )
        self.person_tracking_image_sub = self.create_subscription(
            CompressedImage,
            self.person_tracking_annotated_image_topic,
            self._person_tracking_image_callback,
            1,
        )

        self.http_server = PlanningConsoleHttpServer(
            (self.web_host, self.web_port),
            self,
        )
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="nav-planning-console-http",
            daemon=True,
        )
        self.http_thread.start()

        self.get_logger().info(
            "Serving Nav2 planning console at "
            f"http://{self.web_host}:{self.web_port}/"
        )
        self.get_logger().info(
            "Using map_topic="
            f"{self.map_topic}, global_frame={self.global_frame}, "
            f"base_frame={self.base_frame}, planner_action="
            f"{self.planner_action_name}, navigator_action="
            f"{self.navigator_action_name}, navigation_plane_z="
            f"{self.navigation_plane_z:.3f}."
        )
        self.get_logger().info(
            f"Manual control publishes Twist commands to {self.manual_cmd_topic}."
        )
        self.get_logger().info(
            "Person tracking console control "
            f"{'enabled' if self.person_tracking_control_enabled else 'disabled'}; "
            f"image_topic={self.person_tracking_image_topic}, "
            f"annotated_image_topic={self.person_tracking_annotated_image_topic}, "
            f"detection_topic={self.person_tracking_detection_topic}."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("web_host", "127.0.0.1")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planner_action_name", "compute_path_to_pose")
        self.declare_parameter("navigator_action_name", "navigate_to_pose")
        self.declare_parameter("manual_cmd_topic", "cmd_vel_manual")
        self.declare_parameter("manual_control_enabled", True)
        self.declare_parameter("manual_max_linear_velocity", 0.25)
        self.declare_parameter("manual_max_angular_velocity", 0.50)
        self.declare_parameter("manual_field_relative_enabled", True)
        self.declare_parameter("manual_orientation_topic", "imu/data")
        self.declare_parameter("manual_orientation_timeout_sec", 1.0)
        self.declare_parameter("person_tracking_control_enabled", True)
        self.declare_parameter("person_tracking_node_name", "person_tracking")
        self.declare_parameter(
            "person_tracking_image_topic",
            "/zed/zed_node/rgb/color/rect/image/compressed",
        )
        self.declare_parameter(
            "person_tracking_engine_path",
            "/home/jetson-nano-x1/Documents/B_Cubed/models/yolo11n.engine",
        )
        self.declare_parameter("person_tracking_show_window", False)
        self.declare_parameter("person_tracking_publish_annotated_image", True)
        self.declare_parameter(
            "person_tracking_detection_topic",
            "/person_tracking/detections",
        )
        self.declare_parameter(
            "person_tracking_annotated_image_topic",
            "/person_tracking/annotated_image/compressed",
        )
        self.declare_parameter("person_tracking_confidence_threshold", 0.4)
        self.declare_parameter("person_tracking_nms_threshold", 0.45)
        self.declare_parameter("planner_id", "")
        self.declare_parameter("planner_server_timeout_sec", 2.0)
        self.declare_parameter("navigator_server_timeout_sec", 2.0)
        self.declare_parameter("planning_timeout_sec", 10.0)
        self.declare_parameter("navigation_plane_z", 0.0)
        self.declare_parameter("pose_update_period_sec", 0.2)
        self.declare_parameter("start_pose_rescue_enabled", True)
        self.declare_parameter("start_pose_rescue_radius", 0.50)
        self.declare_parameter("start_pose_rescue_clearance", 0.25)
        self.declare_parameter("start_pose_occupied_threshold", 65)
        self.declare_parameter("start_pose_unknown_is_occupied", False)
        self.declare_parameter("static_dir", "")

    def destroy_node(self) -> bool:
        """Stop the HTTP server when the ROS node is destroyed."""
        self._stop_managed_person_tracking(wait_sec=1.5)
        if hasattr(self, "http_server"):
            self.http_server.shutdown()
            self.http_server.server_close()
        return super().destroy_node()

    def _resolve_static_dir(self, static_dir_param: str) -> Path:
        if static_dir_param:
            return Path(static_dir_param).expanduser()

        try:
            share_dir = Path(get_package_share_directory("nav_planning_console"))
            static_dir = share_dir / "web"
            if static_dir.is_dir():
                return static_dir
        except PackageNotFoundError:
            pass

        return Path(__file__).resolve().parents[1] / "web"

    def _map_callback(self, msg: OccupancyGrid) -> None:
        try:
            map_png = _render_map_png(msg)
        except (IndexError, ValueError) as error:
            self.get_logger().warning(f"Unable to render map PNG: {error}")
            return

        with self._lock:
            self._map_msg = msg
            self._map_png = map_png
            self._map_revision += 1
            self._map_received_time = time.monotonic()

    def _person_tracking_detection_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().warning(
                f"Unable to parse person-tracking detections: {error}"
            )
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            detections = []

        cleaned_detections = []
        for detection in detections:
            if isinstance(detection, dict):
                cleaned_detections.append(dict(detection))

        with self._lock:
            self._person_tracking_detections = cleaned_detections
            self._person_tracking_detections_received_time = time.monotonic()

    def _person_tracking_image_callback(self, msg: CompressedImage) -> None:
        with self._lock:
            self._person_tracking_image = bytes(msg.data)
            self._person_tracking_image_content_type = (
                _content_type_from_compressed_format(msg.format)
            )
            self._person_tracking_image_revision += 1
            self._person_tracking_image_received_time = time.monotonic()
            self._person_tracking_image_stamp = _time_to_float(msg.header.stamp)

    def _manual_orientation_callback(self, msg: Imu) -> None:
        if msg.orientation_covariance[0] == -1.0:
            self.get_logger().warn(
                "Manual field-relative control is waiting for IMU orientation.",
                throttle_duration_sec=2.0,
            )
            return

        yaw = _yaw_from_quaternion(msg.orientation)
        now = time.monotonic()
        with self._lock:
            if self._manual_heading_reference_yaw is None:
                self._manual_heading_reference_yaw = yaw
                self.get_logger().info(
                    "Manual field-relative control latched startup heading "
                    f"{math.degrees(yaw):.1f} deg as forward."
                )
            reference_yaw = self._manual_heading_reference_yaw
            self._manual_heading_yaw = yaw
            self._manual_heading_relative_yaw = _wrap_pi(yaw - reference_yaw)
            self._manual_heading_frame_id = msg.header.frame_id
            self._manual_heading_stamp = _time_to_float(msg.header.stamp)
            self._manual_heading_received_time = now

    def _update_pose_from_tf(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            with self._lock:
                self._pose = None
                self._pose_error = str(error)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        with self._lock:
            self._pose = {
                "frame_id": self.global_frame,
                "child_frame_id": self.base_frame,
                "stamp": _time_to_float(transform.header.stamp),
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
                "yaw": _yaw_from_quaternion(rotation),
            }
            self._pose_error = ""

    def map_png_snapshot(self) -> tuple[int, bytes] | None:
        """Return the latest rendered map PNG."""
        with self._lock:
            if self._map_png is None:
                return None
            return self._map_revision, self._map_png

    def person_tracking_image_snapshot(self) -> tuple[int, str, bytes] | None:
        """Return the latest person-tracking annotated image."""
        with self._lock:
            if self._person_tracking_image is None:
                return None
            return (
                self._person_tracking_image_revision,
                self._person_tracking_image_content_type,
                self._person_tracking_image,
            )

    def snapshot_state(self) -> dict[str, Any]:
        """Return JSON-serializable state for the browser overlay."""
        with self._lock:
            map_msg = self._map_msg
            map_revision = self._map_revision
            map_age = (
                time.monotonic() - self._map_received_time
                if self._map_received_time > 0.0
                else None
            )
            pose = dict(self._pose) if self._pose is not None else None
            pose_error = self._pose_error
            path_points = [dict(point) for point in self._path_points]
            path_revision = self._path_revision
            goal = dict(self._goal) if self._goal is not None else None
            planner_status = self._planner_status
            planner_error = self._planner_error
            navigation_goal = (
                dict(self._navigation_goal)
                if self._navigation_goal is not None
                else None
            )
            navigation_status = self._navigation_status
            navigation_error = self._navigation_error
            navigation_feedback = dict(self._navigation_feedback)
            manual_command = dict(self._manual_command)

        map_payload = None
        if map_msg is not None:
            origin = map_msg.info.origin
            map_payload = {
                "frame_id": map_msg.header.frame_id,
                "stamp": _time_to_float(map_msg.header.stamp),
                "resolution": float(map_msg.info.resolution),
                "width": int(map_msg.info.width),
                "height": int(map_msg.info.height),
                "origin": {
                    "x": origin.position.x,
                    "y": origin.position.y,
                    "z": origin.position.z,
                    "yaw": _yaw_from_quaternion(origin.orientation),
                },
            }

        return {
            "map_revision": map_revision,
            "map_age_sec": map_age,
            "map": map_payload,
            "pose": pose,
            "pose_error": pose_error,
            "path_revision": path_revision,
            "path": {
                "frame_id": self.global_frame,
                "points": path_points,
            },
            "goal": goal,
            "planner": {
                "action_name": self.planner_action_name,
                "status": planner_status,
                "error": planner_error,
            },
            "navigation": {
                "action_name": self.navigator_action_name,
                "status": navigation_status,
                "error": navigation_error,
                "goal": navigation_goal,
                "feedback": navigation_feedback,
            },
            "manual": {
                "enabled": self.manual_control_enabled,
                "cmd_topic": self.manual_cmd_topic,
                "max_linear_velocity": self.manual_max_linear_velocity,
                "max_angular_velocity": self.manual_max_angular_velocity,
                "orientation": self._manual_orientation_state_payload(),
                "command": manual_command,
            },
            "person_tracking": self._person_tracking_state_payload(),
        }

    def _manual_orientation_state_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            yaw = self._manual_heading_yaw
            reference_yaw = self._manual_heading_reference_yaw
            relative_yaw = self._manual_heading_relative_yaw
            frame_id = self._manual_heading_frame_id
            stamp = self._manual_heading_stamp
            received_time = self._manual_heading_received_time

        age = now - received_time if received_time > 0.0 else None
        available = yaw is not None and reference_yaw is not None
        fresh = (
            available
            and age is not None
            and (
                self.manual_orientation_timeout_sec <= 0.0
                or age <= self.manual_orientation_timeout_sec
            )
        )
        return {
            "enabled": self.manual_field_relative_enabled,
            "topic": self.manual_orientation_topic,
            "available": available,
            "fresh": fresh,
            "age_sec": age,
            "timeout_sec": self.manual_orientation_timeout_sec,
            "frame_id": frame_id,
            "stamp": stamp,
            "yaw": yaw,
            "reference_yaw": reference_yaw,
            "relative_yaw": relative_yaw,
        }

    def _person_tracking_node_is_running(self) -> bool:
        target_name = self.person_tracking_node_name.lstrip("/")
        target_full_name = f"/{target_name}"
        for node_name, namespace in self.get_node_names_and_namespaces():
            namespace = namespace.rstrip("/") or "/"
            full_name = (
                f"/{node_name}"
                if namespace == "/"
                else f"{namespace}/{node_name}"
            )
            if node_name == target_name or full_name == target_full_name:
                return True
        return False

    def _refresh_person_tracking_process(self) -> tuple[bool, str]:
        with self._lock:
            process = self._person_tracking_process
            status = self._person_tracking_control_status

        if process is None:
            return False, status

        return_code = process.poll()
        if return_code is None:
            return True, "starting"

        status = "stopped" if return_code == 0 else f"exited {return_code}"
        error = "" if return_code == 0 else "Managed person tracking process exited."
        with self._lock:
            if self._person_tracking_process is process:
                self._person_tracking_process = None
                self._person_tracking_control_status = status
                self._person_tracking_error = error
        return False, status

    def _person_tracking_state_payload(self) -> dict[str, Any]:
        managed_running, process_status = self._refresh_person_tracking_process()
        node_running = self._person_tracking_node_is_running()
        now = time.monotonic()

        with self._lock:
            image_available = self._person_tracking_image is not None
            image_revision = self._person_tracking_image_revision
            image_received_time = self._person_tracking_image_received_time
            image_stamp = self._person_tracking_image_stamp
            detections = [
                dict(detection)
                for detection in self._person_tracking_detections
            ]
            detections_received_time = (
                self._person_tracking_detections_received_time
            )
            error = self._person_tracking_error

        if node_running:
            status = "running" if managed_running else "external"
        elif managed_running:
            status = process_status
        else:
            status = process_status or "stopped"

        image_age = (
            now - image_received_time
            if image_received_time > 0.0
            else None
        )
        detections_age = (
            now - detections_received_time
            if detections_received_time > 0.0
            else None
        )

        return {
            "control_enabled": self.person_tracking_control_enabled,
            "enabled": bool(node_running or managed_running),
            "managed": managed_running,
            "status": status,
            "error": error,
            "node_name": self.person_tracking_node_name,
            "image_topic": self.person_tracking_image_topic,
            "annotated_image_topic": self.person_tracking_annotated_image_topic,
            "detection_topic": self.person_tracking_detection_topic,
            "image": {
                "available": image_available,
                "revision": image_revision,
                "age_sec": image_age,
                "stamp": image_stamp,
            },
            "detections": detections,
            "detections_age_sec": detections_age,
        }

    def set_person_tracking_enabled(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Start or stop the console-managed person-tracking process."""
        enabled = _coerce_bool(payload.get("enabled", False))
        if enabled:
            self._start_person_tracking()
        else:
            self._stop_person_tracking()
        return {"person_tracking": self._person_tracking_state_payload()}

    def _person_tracking_launch_command(self) -> list[str]:
        ros2_executable = shutil.which("ros2")
        if ros2_executable is None:
            raise PlanningConsoleError(
                "The ros2 command is not available in the console environment."
            )

        return [
            ros2_executable,
            "launch",
            "person_tracking",
            "person_tracking.launch.py",
            f"image_topic:={self.person_tracking_image_topic}",
            f"engine_path:={self.person_tracking_engine_path}",
            f"show_window:={str(self.person_tracking_show_window).lower()}",
            (
                "publish_annotated_image:="
                f"{str(self.person_tracking_publish_annotated_image).lower()}"
            ),
            f"detection_topic:={self.person_tracking_detection_topic}",
            (
                "annotated_image_topic:="
                f"{self.person_tracking_annotated_image_topic}"
            ),
            (
                "confidence_threshold:="
                f"{self.person_tracking_confidence_threshold:.3f}"
            ),
            f"nms_threshold:={self.person_tracking_nms_threshold:.3f}",
        ]

    def _start_person_tracking(self) -> None:
        if not self.person_tracking_control_enabled:
            raise PlanningConsoleError("Person tracking control is disabled.")

        managed_running, _ = self._refresh_person_tracking_process()
        if managed_running or self._person_tracking_node_is_running():
            with self._lock:
                self._person_tracking_control_status = "running"
                self._person_tracking_error = ""
            return

        command = self._person_tracking_launch_command()
        try:
            process = subprocess.Popen(  # noqa: S603 - command is fixed argv.
                command,
                env=os.environ.copy(),
                start_new_session=True,
            )
        except OSError as error:
            with self._lock:
                self._person_tracking_control_status = "failed"
                self._person_tracking_error = str(error)
            raise PlanningConsoleError(
                f"Unable to start person tracking: {error}"
            ) from error

        with self._lock:
            self._person_tracking_process = process
            self._person_tracking_control_status = "starting"
            self._person_tracking_error = ""

    def _stop_person_tracking(self) -> None:
        stopped = self._stop_managed_person_tracking(wait_sec=4.0)
        if stopped:
            return
        if self._person_tracking_node_is_running():
            raise PlanningConsoleError(
                "Person tracking is running outside the web console; stop that "
                "launch process directly."
            )
        with self._lock:
            self._person_tracking_control_status = "stopped"
            self._person_tracking_error = ""

    def _stop_managed_person_tracking(self, wait_sec: float) -> bool:
        with self._lock:
            process = self._person_tracking_process
            self._person_tracking_process = None

        if process is None:
            return False
        if process.poll() is not None:
            with self._lock:
                self._person_tracking_control_status = "stopped"
                self._person_tracking_error = ""
            return True

        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=wait_sec)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
        finally:
            with self._lock:
                self._person_tracking_control_status = "stopped"
                self._person_tracking_error = ""

        return True

    def publish_manual_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Publish a normalized manual-control command as a Twist."""
        if not self.manual_control_enabled:
            raise PlanningConsoleError("Manual control is disabled.")

        x_value = float(payload.get("x", 0.0))
        y_value = float(payload.get("y", 0.0))
        angular_value = float(payload.get("angular", 0.0))

        magnitude = math.hypot(x_value, y_value)
        if magnitude > 1.0:
            x_value /= magnitude
            y_value /= magnitude
            magnitude = 1.0

        x_value = _clamp(x_value, -1.0, 1.0)
        y_value = _clamp(y_value, -1.0, 1.0)
        angular_value = _clamp(angular_value, -1.0, 1.0)

        requested_x = x_value
        requested_y = y_value
        heading = None
        field_relative = False
        if self.manual_field_relative_enabled and magnitude > 1e-6:
            heading = self._fresh_manual_heading()
            if heading is None:
                raise PlanningConsoleError(
                    "Manual orientation is not available yet; waiting for "
                    f"{self.manual_orientation_topic}."
                )
            x_value, y_value = _rotate_start_frame_to_body_frame(
                x_value,
                y_value,
                float(heading["relative_yaw"]),
            )
            field_relative = True

        twist = Twist()
        twist.linear.x = x_value * self.manual_max_linear_velocity
        twist.linear.y = y_value * self.manual_max_linear_velocity
        twist.angular.z = angular_value * self.manual_max_angular_velocity

        active = (
            abs(twist.linear.x) > 1e-5
            or abs(twist.linear.y) > 1e-5
            or abs(twist.angular.z) > 1e-5
        )
        command = {
            "linear_x": twist.linear.x,
            "linear_y": twist.linear.y,
            "angular_z": twist.angular.z,
            "requested_linear_x": requested_x * self.manual_max_linear_velocity,
            "requested_linear_y": requested_y * self.manual_max_linear_velocity,
            "field_relative": field_relative,
            "heading_relative_yaw": (
                float(heading["relative_yaw"]) if heading is not None else None
            ),
            "active": active,
            "stamp": time.time(),
        }

        self.manual_cmd_pub.publish(twist)
        with self._lock:
            self._manual_command = command

        return {"manual": {"command": command}}

    def _fresh_manual_heading(self) -> dict[str, Any] | None:
        state = self._manual_orientation_state_payload()
        if not state["fresh"]:
            return None
        return state

    def plan_to_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Send a ComputePathToPose request to Nav2."""
        x_value = float(payload["x"])
        y_value = float(payload["y"])
        yaw = payload.get("yaw")
        start_pose_adjustment: dict[str, Any] | None = None
        with self._lock:
            map_msg = self._map_msg
            start_pose = dict(self._pose) if self._pose is not None else None
        if yaw is None:
            yaw = start_pose["yaw"] if start_pose is not None else 0.0
        yaw_value = float(yaw)

        if map_msg is None:
            message = "No map has been received yet."
            self._set_planner_error(message)
            raise PlanningConsoleError(message)
        if not _point_in_map_bounds(map_msg, x_value, y_value):
            message = "Goal is outside the current map bounds."
            self._set_planner_error(message)
            raise PlanningConsoleError(message)
        if start_pose is not None and not _point_in_map_bounds(
            map_msg,
            float(start_pose["x"]),
            float(start_pose["y"]),
        ):
            message = "Robot pose is outside the current map bounds."
            self._set_planner_error(message)
            raise PlanningConsoleError(message)
        if start_pose is not None and self.start_pose_rescue_enabled:
            adjusted_start_pose, adjustment_distance = _find_nearest_free_start_pose(
                map_msg,
                start_pose,
                self.start_pose_rescue_radius,
                self.start_pose_rescue_clearance,
                self.start_pose_occupied_threshold,
                self.start_pose_unknown_is_occupied,
            )
            if adjusted_start_pose is None:
                message = (
                    "Robot pose is inside or too close to an obstacle, and no "
                    "free planning start was found within "
                    f"{self.start_pose_rescue_radius:.2f} m."
                )
                self._set_planner_error(message)
                raise PlanningConsoleError(message)
            if adjustment_distance > 1e-6:
                start_pose_adjustment = {
                    "original": {
                        "x": float(start_pose["x"]),
                        "y": float(start_pose["y"]),
                        "z": float(start_pose.get("z", self.navigation_plane_z)),
                        "yaw": float(start_pose["yaw"]),
                    },
                    "adjusted": {
                        "x": float(adjusted_start_pose["x"]),
                        "y": float(adjusted_start_pose["y"]),
                        "z": float(
                            adjusted_start_pose.get("z", self.navigation_plane_z)
                        ),
                        "yaw": float(adjusted_start_pose["yaw"]),
                    },
                    "distance_m": adjustment_distance,
                }
                start_pose = adjusted_start_pose
                self.get_logger().warning(
                    "Planner start pose was inside or too close to an obstacle; "
                    f"using nearest free start {adjustment_distance:.2f} m away."
                )

        with self._lock:
            self._goal = {
                "x": x_value,
                "y": y_value,
                "z": self.navigation_plane_z,
                "yaw": yaw_value,
            }
            if start_pose_adjustment is None:
                self._planner_status = "planning"
            else:
                self._planner_status = (
                    "planning "
                    f"(start shifted {start_pose_adjustment['distance_m']:.2f} m)"
                )
            self._planner_error = ""

        if not self.planner_client.wait_for_server(
            timeout_sec=self.planner_server_timeout_sec
        ):
            self._set_planner_error(
                f"Planner action '{self.planner_action_name}' is not available. "
                "Start the Nav2 planner server or the handheld planner-only launch."
            )
            raise PlanningConsoleError(
                f"Planner action '{self.planner_action_name}' is not available. "
                "Start the Nav2 planner server or the handheld planner-only launch."
            )

        goal_msg = ComputePathToPose.Goal()
        stamp = self.get_clock().now().to_msg()
        goal_msg.goal.header.frame_id = self.global_frame
        goal_msg.goal.header.stamp = stamp
        goal_msg.goal.pose.position.x = x_value
        goal_msg.goal.pose.position.y = y_value
        goal_msg.goal.pose.position.z = self.navigation_plane_z
        quat_x, quat_y, quat_z, quat_w = _quaternion_from_yaw(yaw_value)
        goal_msg.goal.pose.orientation.x = quat_x
        goal_msg.goal.pose.orientation.y = quat_y
        goal_msg.goal.pose.orientation.z = quat_z
        goal_msg.goal.pose.orientation.w = quat_w
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = start_pose is not None
        if start_pose is not None:
            start_quat = _quaternion_from_yaw(float(start_pose["yaw"]))
            goal_msg.start.header.frame_id = self.global_frame
            goal_msg.start.header.stamp = stamp
            goal_msg.start.pose.position.x = float(start_pose["x"])
            goal_msg.start.pose.position.y = float(start_pose["y"])
            goal_msg.start.pose.position.z = self.navigation_plane_z
            goal_msg.start.pose.orientation.x = start_quat[0]
            goal_msg.start.pose.orientation.y = start_quat[1]
            goal_msg.start.pose.orientation.z = start_quat[2]
            goal_msg.start.pose.orientation.w = start_quat[3]

        send_future = self.planner_client.send_goal_async(goal_msg)
        goal_handle = self._wait_for_future(
            send_future,
            self.planning_timeout_sec,
            "Timed out while sending the planning goal.",
        )
        if goal_handle is None or not goal_handle.accepted:
            self._set_planner_error("Planning goal was rejected.")
            raise PlanningConsoleError("Planning goal was rejected.")

        result_future = goal_handle.get_result_async()
        action_result = self._wait_for_future(
            result_future,
            self.planning_timeout_sec,
            "Timed out while waiting for the planned path.",
        )

        path_msg = action_result.result.path
        path_points = _path_to_points(path_msg)
        if action_result.status != GoalStatus.STATUS_SUCCEEDED:
            status_text = _goal_status_text(action_result.status)
            message = f"Planner {status_text} (status {action_result.status})."
            self._set_planner_error(message)
            raise PlanningConsoleError(message)

        planning_time = action_result.result.planning_time
        planning_time_sec = float(planning_time.sec) + float(planning_time.nanosec) * 1e-9

        with self._lock:
            self._path_points = path_points
            self._path_revision += 1
            self._planner_status = f"planned {len(path_points)} poses"
            if start_pose_adjustment is not None:
                self._planner_status += (
                    f" (start shifted {start_pose_adjustment['distance_m']:.2f} m)"
                )
            self._planner_error = ""

        result = {
            "goal": {
                "x": x_value,
                "y": y_value,
                "z": self.navigation_plane_z,
                "yaw": yaw_value,
            },
            "path_revision": self._path_revision,
            "path": {
                "frame_id": path_msg.header.frame_id,
                "points": path_points,
            },
            "planning_time_sec": planning_time_sec,
        }
        if start_pose_adjustment is not None:
            result["start_pose_adjustment"] = start_pose_adjustment
        return result

    def clear_path(self) -> None:
        """Clear the cached path and goal overlays."""
        with self._lock:
            self._path_points = []
            self._goal = None
            self._path_revision += 1
            self._planner_status = "idle"
            self._planner_error = ""

    def navigate_to_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Send a NavigateToPose request to Nav2 using a planar goal."""
        goal = self._navigation_goal_from_payload(payload)
        x_value = float(goal["x"])
        y_value = float(goal["y"])
        yaw_value = float(goal["yaw"])

        if not self.navigator_client.wait_for_server(
            timeout_sec=self.navigator_server_timeout_sec
        ):
            message = (
                f"Navigator action '{self.navigator_action_name}' is not available. "
                "Start the Nav2 BT navigator launch before executing a path."
            )
            self._set_navigation_error(message)
            raise PlanningConsoleError(message)

        with self._lock:
            self._navigation_goal_token += 1
            token = self._navigation_goal_token
            self._navigation_goal = {
                "x": x_value,
                "y": y_value,
                "z": self.navigation_plane_z,
                "yaw": yaw_value,
            }
            self._navigation_status = "sending"
            self._navigation_error = ""
            self._navigation_feedback = {}
            self._navigation_goal_handle = None

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = self.global_frame
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = x_value
        nav_goal.pose.pose.position.y = y_value
        nav_goal.pose.pose.position.z = self.navigation_plane_z
        quat_x, quat_y, quat_z, quat_w = _quaternion_from_yaw(yaw_value)
        nav_goal.pose.pose.orientation.x = quat_x
        nav_goal.pose.pose.orientation.y = quat_y
        nav_goal.pose.pose.orientation.z = quat_z
        nav_goal.pose.pose.orientation.w = quat_w

        send_future = self.navigator_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda feedback_msg: self._handle_navigation_feedback(
                feedback_msg.feedback,
                token,
            ),
        )
        goal_handle = self._wait_for_future(
            send_future,
            self.planning_timeout_sec,
            "Timed out while sending the navigation goal.",
            self._set_navigation_error,
        )
        if goal_handle is None or not goal_handle.accepted:
            self._set_navigation_error("Navigation goal was rejected.")
            raise PlanningConsoleError("Navigation goal was rejected.")

        with self._lock:
            if token == self._navigation_goal_token:
                self._navigation_goal_handle = goal_handle
                self._navigation_status = "active"

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._handle_navigation_result(future, token)
        )

        return {
            "navigation": {
                "status": "active",
                "goal": {
                    "x": x_value,
                    "y": y_value,
                    "z": self.navigation_plane_z,
                    "yaw": yaw_value,
                },
            }
        }

    def cancel_navigation(self) -> dict[str, Any]:
        """Cancel the currently active NavigateToPose goal."""
        with self._lock:
            goal_handle = self._navigation_goal_handle
            token = self._navigation_goal_token

        if goal_handle is None:
            with self._lock:
                self._navigation_status = "idle"
                self._navigation_error = ""
                self._navigation_feedback = {}
            return {"navigation": {"status": "idle"}}

        cancel_future = goal_handle.cancel_goal_async()
        self._wait_for_future(
            cancel_future,
            self.navigator_server_timeout_sec,
            "Timed out while canceling the navigation goal.",
            self._set_navigation_error,
        )

        with self._lock:
            if token == self._navigation_goal_token:
                self._navigation_status = "canceling"
                self._navigation_error = ""

        return {"navigation": {"status": "canceling"}}

    def _navigation_goal_from_payload(self, payload: dict[str, Any]) -> dict[str, float]:
        """Return a navigation goal from the request body or cached plan goal."""
        if "x" in payload and "y" in payload:
            yaw = payload.get("yaw")
            if yaw is None:
                with self._lock:
                    cached_goal = dict(self._goal) if self._goal is not None else None
                yaw = cached_goal["yaw"] if cached_goal is not None else 0.0
            return {
                "x": float(payload["x"]),
                "y": float(payload["y"]),
                "yaw": float(yaw),
            }

        with self._lock:
            cached_goal = dict(self._goal) if self._goal is not None else None
        if cached_goal is None:
            raise PlanningConsoleError("No planned goal is available to navigate to.")
        return {
            "x": float(cached_goal["x"]),
            "y": float(cached_goal["y"]),
            "yaw": float(cached_goal.get("yaw", 0.0)),
        }

    def _handle_navigation_feedback(self, feedback: Any, token: int) -> None:
        """Cache NavigateToPose feedback for the browser."""
        feedback_payload: dict[str, Any] = {}
        if hasattr(feedback, "distance_remaining"):
            feedback_payload["distance_remaining"] = float(feedback.distance_remaining)
        if hasattr(feedback, "navigation_time"):
            feedback_payload["navigation_time_sec"] = _duration_to_float(
                feedback.navigation_time
            )
        if hasattr(feedback, "estimated_time_remaining"):
            feedback_payload["estimated_time_remaining_sec"] = _duration_to_float(
                feedback.estimated_time_remaining
            )
        if hasattr(feedback, "number_of_recoveries"):
            feedback_payload["number_of_recoveries"] = int(
                feedback.number_of_recoveries
            )
        if hasattr(feedback, "current_pose"):
            feedback_payload["current_pose"] = _pose_to_payload(
                feedback.current_pose.pose
            )

        with self._lock:
            if token != self._navigation_goal_token:
                return
            self._navigation_feedback = feedback_payload
            if self._navigation_status not in ("canceling", "canceled"):
                self._navigation_status = "active"

    def _handle_navigation_result(self, future: Any, token: int) -> None:
        """Update navigation status when NavigateToPose finishes."""
        try:
            action_result = future.result()
            status = int(action_result.status)
        except Exception as error:  # noqa: BLE001 - surface action failure to UI.
            with self._lock:
                if token == self._navigation_goal_token:
                    self._navigation_status = "failed"
                    self._navigation_error = str(error)
                    self._navigation_goal_handle = None
            return

        status_text = _goal_status_text(status)
        error_message = ""
        if status == GoalStatus.STATUS_ABORTED:
            error_message = "Navigator aborted the goal."
        elif status not in (
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
        ):
            error_message = f"Navigator returned {status_text}."

        with self._lock:
            if token != self._navigation_goal_token:
                return
            self._navigation_status = status_text
            self._navigation_error = error_message
            self._navigation_goal_handle = None

    def _wait_for_future(
        self,
        future: Any,
        timeout_sec: float,
        timeout_message: str,
        timeout_handler: Callable[[str], None] | None = None,
    ) -> Any:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_sec):
            if timeout_handler is None:
                self._set_planner_error(timeout_message)
            else:
                timeout_handler(timeout_message)
            raise PlanningConsoleError(timeout_message)
        return future.result()

    def _set_planner_error(self, message: str) -> None:
        with self._lock:
            self._planner_status = "error"
            self._planner_error = message

    def _set_navigation_error(self, message: str) -> None:
        with self._lock:
            self._navigation_status = "failed"
            self._navigation_error = message


def main(args: list[str] | None = None) -> None:
    """Run the planning console node."""
    rclpy.init(args=args)
    node = NavPlanningConsoleNode()
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
