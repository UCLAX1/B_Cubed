"""CAN hardware interface for the Kiwi drive motors."""

from __future__ import annotations

import json
import logging
from pathlib import Path
import struct
import time
from typing import Any

try:
    import can
except ImportError:  # pragma: no cover - exercised on robot images with python-can.
    can = None


class ProcessEncoderData(can.Listener if can is not None else object):
    """Track motor encoder positions from status frame 2."""

    STATUS_2 = 0x2051880

    def __init__(self, motor_pos: list[float | None]):
        super().__init__()
        self.motor_pos = motor_pos

    def on_message_received(self, msg: Any) -> None:
        """Update the latest position for a motor from a CAN message."""
        if msg.arbitration_id & 0xFFFFFFFF0 != self.STATUS_2:
            return

        raw_position_bytes = msg.data[0:4]
        position_float = struct.unpack("<f", raw_position_bytes)[0]
        motor_id = msg.arbitration_id & 0xF
        if 0 <= motor_id < len(self.motor_pos):
            self.motor_pos[motor_id] = position_float


class CanBus:
    """Small wrapper around python-can with the motor branch defaults."""

    EXT_MASK = 0x80000000

    def __init__(
        self,
        channel: str = "can0",
        interface: str = "socketcan",
        bitrate: int = 1000000,
        logger: logging.Logger | None = None,
    ):
        self.notifier = None
        self.bus = None
        self.motor_pos: list[float | None] = [None] * 30
        self.channel = channel
        self.interface = interface
        self.bitrate = bitrate
        self.logger = logger

    def start(self) -> None:
        """Open the CAN bus and start listening for encoder frames."""
        if can is None:
            self._log_error("python-can is not installed; CAN bus is disabled.")
            return

        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                interface=self.interface,
                bitrate=self.bitrate,
            )
            listener = ProcessEncoderData(motor_pos=self.motor_pos)
            self.notifier = can.Notifier(self.bus, [listener])
            self._log_info(
                f"CAN bus started on {self.interface}:{self.channel} "
                f"at {self.bitrate} bps."
            )
        except Exception as error:  # noqa: BLE001 - keep controller alive dry-run.
            self._log_error(f"Failed to start CAN bus: {error}")
            self.bus = None
            self.notifier = None

    def started_successfully(self) -> bool:
        """Return true when the CAN bus is available."""
        return self.bus is not None

    def send_message(self, arbitration_id: int, data: bytes | bytearray | list[int]) -> None:
        """Send a CAN message with an extended arbitration ID."""
        if self.bus is None or can is None:
            return

        message = can.Message(
            arbitration_id=arbitration_id | self.EXT_MASK,
            data=bytes(data),
            is_extended_id=True,
        )
        try:
            self.bus.send(message)
        except can.CanError as error:
            self._log_error(f"Failed to send CAN message: {error}")

    def close(self) -> None:
        """Stop CAN listeners and close the bus."""
        if self.notifier is not None:
            self.notifier.stop()
            self.notifier = None
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
        self._log_info("CAN bus stopped.")

    def _log_info(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(message)

    def _log_error(self, message: str) -> None:
        if self.logger is not None:
            self.logger.error(message)


class Motor:
    """Single motor controller using the CAN protocol from motors_test."""

    DUTY_CYCLE_ID = 0x2050080
    HEARTBEAT_ID = 0x2052480

    def __init__(
        self,
        can_bus: CanBus,
        motor_id: int,
        *,
        max_duty_cycle: float = 0.5,
        init_pos_path: str | Path = "motor_init_pos.json",
        wait_timeout_sec: float = 2.0,
        logger: logging.Logger | None = None,
    ):
        self.can_bus = can_bus
        self.motor_id = motor_id
        self.max_duty_cycle = abs(float(max_duty_cycle))
        self.init_pos_path = Path(init_pos_path)
        self.wait_timeout_sec = max(0.0, float(wait_timeout_sec))
        self.logger = logger
        self.init_pos = self._load_initial_position()

        self._wait_for_position()
        self._log_info(f"Motor {self.motor_id} connected at {self.get_pos():.3f}.")

    def set_power(self, power: float) -> None:
        """Set normalized motor power using the duty-cycle CAN frame."""
        msg_id = self.DUTY_CYCLE_ID + self.motor_id
        clamped_power = max(-1.0, min(float(power), 1.0))
        duty_cycle = clamped_power * self.max_duty_cycle
        duty_data = struct.pack("<f", duty_cycle)
        data = bytearray(8)
        data[:4] = duty_data
        self.can_bus.send_message(msg_id, data)

    def send_heartbeat(self) -> None:
        """Keep the motor controller alive."""
        msg_id = self.HEARTBEAT_ID + self.motor_id
        self.can_bus.send_message(msg_id, [0xFF] * 8)

    def get_pos(self) -> float:
        """Return motor position relative to the saved initial position."""
        position = self.can_bus.motor_pos[self.motor_id]
        if position is None:
            return 0.0
        return float(position) - self.init_pos

    def reset_encoder(self) -> None:
        """Persist the current encoder position as the new zero offset."""
        position = self.can_bus.motor_pos[self.motor_id]
        if position is None:
            return

        self.init_pos = float(position)
        data = self._load_initial_positions()
        data[str(self.motor_id)] = self.init_pos
        self.init_pos_path.parent.mkdir(parents=True, exist_ok=True)
        self.init_pos_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        self._log_info(f"Reset motor {self.motor_id} encoder to {self.init_pos:.3f}.")

    def _load_initial_position(self) -> float:
        return float(self._load_initial_positions().get(str(self.motor_id), 0.0))

    def _load_initial_positions(self) -> dict[str, float]:
        if not self.init_pos_path.is_file():
            return {}
        try:
            data = json.loads(self.init_pos_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        if not isinstance(data, dict):
            return {}
        return data

    def _wait_for_position(self) -> None:
        if not self.can_bus.started_successfully():
            raise RuntimeError("CAN bus is not started.")

        start = time.monotonic()
        next_heartbeat = 0.0
        while self.can_bus.motor_pos[self.motor_id] is None:
            now = time.monotonic()
            if now >= next_heartbeat:
                self.send_heartbeat()
                next_heartbeat = now + 0.10
            if now - start > self.wait_timeout_sec:
                raise TimeoutError(
                    f"Timed out waiting for motor {self.motor_id} encoder status."
                )
            time.sleep(0.05)

    def _log_info(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(message)
