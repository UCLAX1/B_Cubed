#!/usr/bin/env python3
"""Serve a local web console for viewing a map and requesting Nav2 plans."""

from __future__ import annotations

import json
import math
import mimetypes
from pathlib import Path
import posixpath
import struct
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable
from urllib.parse import unquote, urlparse
import zlib

from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


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


class PlanningConsoleError(RuntimeError):
    """Raised when the web console cannot complete a planning request."""


class PlanningConsoleHttpServer(ThreadingHTTPServer):
    """HTTP server that carries a reference to the ROS node."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address: tuple[str, int], node: "NavPlanningConsoleNode"):
        self.node = node
        super().__init__(server_address, PlanningConsoleRequestHandler)


class PlanningConsoleRequestHandler(BaseHTTPRequestHandler):
    """HTTP API and static file handler for the planning console."""

    server: PlanningConsoleHttpServer

    def do_OPTIONS(self) -> None:
        """Handle browser CORS preflight requests."""
        self.send_response(HTTPStatus.NO_CONTENT)
        self._send_common_headers()
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        """Serve API responses and static assets."""
        parsed_url = urlparse(self.path)
        if parsed_url.path == "/api/state":
            self._send_json(self.server.node.snapshot_state())
            return
        if parsed_url.path == "/api/map.png":
            self._send_map_png()
            return
        if parsed_url.path == "/api/health":
            self._send_json({"ok": True})
            return

        self._send_static_file(parsed_url.path)

    def do_POST(self) -> None:
        """Handle planning and path-management API requests."""
        parsed_url = urlparse(self.path)
        try:
            if parsed_url.path == "/api/plan":
                payload = self._read_json_body()
                result = self.server.node.plan_to_goal(payload)
                self._send_json({"ok": True, **result})
                return
            if parsed_url.path == "/api/clear_path":
                self.server.node.clear_path()
                self._send_json({"ok": True})
                return
            if parsed_url.path == "/api/navigate":
                payload = self._read_json_body()
                result = self.server.node.navigate_to_goal(payload)
                self._send_json({"ok": True, **result})
                return
            if parsed_url.path == "/api/cancel_navigation":
                result = self.server.node.cancel_navigation()
                self._send_json({"ok": True, **result})
                return
        except PlanningConsoleError as error:
            self._send_json(
                {"ok": False, "error": str(error)},
                status=HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self._send_json(
                {"ok": False, "error": str(error)},
                status=HTTPStatus.BAD_REQUEST,
            )
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format_string: str, *args: Any) -> None:
        """Route HTTP logs through ROS so they appear with the node output."""
        self.server.node.get_logger().debug(format_string % args)

    def _read_json_body(self) -> dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            return {}
        if content_length > 65536:
            raise ValueError("Request body is too large.")
        raw_body = self.rfile.read(content_length)
        payload = json.loads(raw_body.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object.")
        return payload

    def _send_common_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("X-Content-Type-Options", "nosniff")

    def _send_json(
        self,
        payload: dict[str, Any],
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self._send_common_headers()
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_map_png(self) -> None:
        snapshot = self.server.node.map_png_snapshot()
        if snapshot is None:
            self.send_error(HTTPStatus.NOT_FOUND, "No map has been received yet.")
            return

        revision, png_data = snapshot
        self.send_response(HTTPStatus.OK)
        self._send_common_headers()
        self.send_header("Content-Type", "image/png")
        self.send_header("Cache-Control", "no-store")
        self.send_header("ETag", f'"map-{revision}"')
        self.send_header("Content-Length", str(len(png_data)))
        self.end_headers()
        self.wfile.write(png_data)

    def _send_static_file(self, request_path: str) -> None:
        static_dir = self.server.node.static_dir
        if request_path in ("", "/"):
            request_path = "/index.html"

        normalized = posixpath.normpath(unquote(request_path)).lstrip("/")
        file_path = (static_dir / normalized).resolve()
        static_root = static_dir.resolve()
        if static_root not in file_path.parents and file_path != static_root:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not file_path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        content_type = mimetypes.guess_type(str(file_path))[0]
        if content_type is None:
            content_type = "application/octet-stream"

        body = file_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self._send_common_headers()
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


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

    def _declare_parameters(self) -> None:
        self.declare_parameter("web_host", "127.0.0.1")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planner_action_name", "compute_path_to_pose")
        self.declare_parameter("navigator_action_name", "navigate_to_pose")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("planner_server_timeout_sec", 2.0)
        self.declare_parameter("navigator_server_timeout_sec", 2.0)
        self.declare_parameter("planning_timeout_sec", 10.0)
        self.declare_parameter("navigation_plane_z", 0.0)
        self.declare_parameter("pose_update_period_sec", 0.2)
        self.declare_parameter("static_dir", "")

    def destroy_node(self) -> bool:
        """Stop the HTTP server when the ROS node is destroyed."""
        if hasattr(self, "http_server"):
            self.http_server.shutdown()
            self.http_server.server_close()
        return super().destroy_node()

    def _resolve_static_dir(self, static_dir_param: str) -> Path:
        if static_dir_param:
            return Path(static_dir_param).expanduser()

        try:
            share_dir = Path(get_package_share_directory("depth_processing"))
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
        }

    def plan_to_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Send a ComputePathToPose request to Nav2."""
        x_value = float(payload["x"])
        y_value = float(payload["y"])
        yaw = payload.get("yaw")
        with self._lock:
            start_pose = dict(self._pose) if self._pose is not None else None
        if yaw is None:
            yaw = start_pose["yaw"] if start_pose is not None else 0.0
        yaw_value = float(yaw)

        with self._lock:
            self._goal = {
                "x": x_value,
                "y": y_value,
                "z": self.navigation_plane_z,
                "yaw": yaw_value,
            }
            self._planner_status = "planning"
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
            message = f"Planner returned status {action_result.status}."
            self._set_planner_error(message)
            raise PlanningConsoleError(message)

        planning_time = action_result.result.planning_time
        planning_time_sec = float(planning_time.sec) + float(planning_time.nanosec) * 1e-9

        with self._lock:
            self._path_points = path_points
            self._path_revision += 1
            self._planner_status = f"planned {len(path_points)} poses"
            self._planner_error = ""

        return {
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
