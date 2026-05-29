"""Unix socket command format for local Pi motor control IPC."""

from __future__ import annotations

import socket
import struct
import time


DEFAULT_SOCKET_PATH = "/tmp/b_cubed_motor_cmd.sock"
COMMAND_STRUCT = struct.Struct("<dffff")


def pack_command(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    state: float,
) -> bytes:
    """Pack a motor command for the local Unix datagram socket."""
    return COMMAND_STRUCT.pack(
        time.monotonic(),
        float(linear_x),
        float(linear_y),
        float(angular_z),
        float(state),
    )


def unpack_command(data: bytes) -> tuple[float, float, float, float, float]:
    """Unpack a motor command into timestamp, x, y, yaw, and state."""
    return COMMAND_STRUCT.unpack(data)


class MotorCommandClient:
    """Tiny best-effort Unix datagram client for RL-to-motor commands."""

    def __init__(self, socket_path: str = DEFAULT_SOCKET_PATH):
        self.socket_path = socket_path
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def send(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        state: float,
    ) -> bool:
        """Send one command packet, returning false if the motor server is absent."""
        packet = pack_command(linear_x, linear_y, angular_z, state)
        try:
            self.socket.sendto(packet, self.socket_path)
        except OSError:
            return False
        return True

    def close(self) -> None:
        """Close the socket."""
        self.socket.close()
