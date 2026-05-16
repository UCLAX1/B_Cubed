"""HTTP API and static asset serving for the planning console."""

from __future__ import annotations

import json
import mimetypes
import posixpath
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import TYPE_CHECKING, Any
from urllib.parse import unquote, urlparse

from nav_planning_console.errors import PlanningConsoleError

if TYPE_CHECKING:
    from nav_planning_console.node import NavPlanningConsoleNode


class PlanningConsoleHttpServer(ThreadingHTTPServer):
    """HTTP server that carries a reference to the ROS node."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        node: "NavPlanningConsoleNode",
    ):
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
            if parsed_url.path == "/api/manual_cmd":
                payload = self._read_json_body()
                result = self.server.node.publish_manual_command(payload)
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
