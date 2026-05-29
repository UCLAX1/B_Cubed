#!/usr/bin/env python3
"""Non-ROS Unix socket motor command server for the Pi."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import socket
import time

from low_level_runner.hardware_interface import CanBus, Motor
from low_level_runner.kiwi_kinematics import WheelPowers, twist_to_wheel_powers
from low_level_runner.motor_ipc import DEFAULT_SOCKET_PATH, unpack_command


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--can-channel", default="can0")
    parser.add_argument("--can-interface", default="socketcan")
    parser.add_argument("--can-bitrate", type=int, default=1000000)
    parser.add_argument("--max-duty-cycle", type=float, default=0.5)
    parser.add_argument("--motor-wait-timeout-sec", type=float, default=2.0)
    parser.add_argument("--init-pos-path", default="~/.ros/b_cubed_motor_init_pos.json")
    parser.add_argument("--top-left-motor-id", type=int, default=3)
    parser.add_argument("--top-right-motor-id", type=int, default=9)
    parser.add_argument("--bottom-motor-id", type=int, default=7)
    parser.add_argument("--linear-to-power-scale", type=float, default=1.0)
    parser.add_argument("--angular-to-power-scale", type=float, default=1.0)
    parser.add_argument("--max-power", type=float, default=1.0)
    parser.add_argument("--normalize-over-limit", action="store_true")
    parser.add_argument("--watchdog-timeout-sec", type=float, default=0.30)
    parser.add_argument("--heartbeat-rate-hz", type=float, default=50.0)
    parser.add_argument("--status-rate-hz", type=float, default=2.0)
    parser.add_argument("--reset-encoders-on-start", action="store_true")
    return parser


class MotorIpcServer:
    """Own CAN hardware and accept command packets over a Unix socket."""

    def __init__(self, args: argparse.Namespace, logger: logging.Logger):
        self.args = args
        self.logger = logger
        self.socket_path = Path(args.socket_path)
        self.socket: socket.socket | None = None
        self.bus: CanBus | None = None
        self.motors: dict[str, Motor] = {}
        self.last_command_time = 0.0
        self.last_packet_latency_sec = 0.0
        self.last_source_state = 0.0
        self.last_powers = WheelPowers(0.0, 0.0, 0.0)
        self.last_status_time = 0.0
        self.hardware_active = False
        self.running = True

    def start(self) -> None:
        self._bind_socket()
        self._start_hardware()
        self._run_loop()

    def stop(self) -> None:
        self.running = False
        self._stop_motors()
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        self.socket_path.unlink(missing_ok=True)

    def _bind_socket(self) -> None:
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self.socket_path.unlink(missing_ok=True)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.bind(str(self.socket_path))
        self.socket.settimeout(1.0 / max(1.0, float(self.args.heartbeat_rate_hz)))
        self.logger.info("Motor IPC socket listening at %s", self.socket_path)

    def _start_hardware(self) -> None:
        self.bus = CanBus(
            channel=self.args.can_channel,
            interface=self.args.can_interface,
            bitrate=self.args.can_bitrate,
            logger=self.logger,
        )
        self.bus.start()
        if not self.bus.started_successfully():
            raise RuntimeError(
                f"CAN bus unavailable on {self.args.can_interface}:{self.args.can_channel}"
            )

        motor_ids = {
            "top_left": self.args.top_left_motor_id,
            "top_right": self.args.top_right_motor_id,
            "bottom": self.args.bottom_motor_id,
        }
        for name, motor_id in motor_ids.items():
            motor = Motor(
                self.bus,
                motor_id,
                max_duty_cycle=self.args.max_duty_cycle,
                init_pos_path=Path(self.args.init_pos_path).expanduser(),
                wait_timeout_sec=self.args.motor_wait_timeout_sec,
                logger=self.logger,
            )
            motor.set_power(0.0)
            if self.args.reset_encoders_on_start:
                motor.reset_encoder()
            self.motors[name] = motor

        self.hardware_active = True
        self.logger.info(
            "Motor IPC CAN active: can=%s:%s bitrate=%s ids=%s",
            self.args.can_interface,
            self.args.can_channel,
            self.args.can_bitrate,
            motor_ids,
        )

    def _run_loop(self) -> None:
        assert self.socket is not None
        while self.running:
            try:
                data = self.socket.recv(128)
            except TimeoutError:
                self._watchdog_tick()
                continue
            except socket.timeout:
                self._watchdog_tick()
                continue

            try:
                timestamp, linear_x, linear_y, angular_z, state = unpack_command(data)
            except Exception as error:  # noqa: BLE001 - reject malformed local packets.
                self.logger.warning("Ignoring malformed motor IPC packet: %s", error)
                continue

            now = time.monotonic()
            self.last_command_time = now
            self.last_packet_latency_sec = max(0.0, now - timestamp)
            self.last_source_state = state
            powers = twist_to_wheel_powers(
                linear_x,
                linear_y,
                angular_z,
                linear_scale=self.args.linear_to_power_scale,
                angular_scale=self.args.angular_to_power_scale,
                max_power=self.args.max_power,
                normalize_over_limit=self.args.normalize_over_limit,
            )
            self._send_motor_command(powers)
            self._report_status(now)

    def _watchdog_tick(self) -> None:
        now = time.monotonic()
        if (
            self.last_command_time > 0.0
            and now - self.last_command_time > self.args.watchdog_timeout_sec
            and self.last_powers.as_list() != [0.0, 0.0, 0.0]
        ):
            self.logger.warning("Motor IPC watchdog stale; zeroing motors.")
            self._send_motor_command(WheelPowers(0.0, 0.0, 0.0))
        else:
            self._send_motor_heartbeat()
        self._report_status(now)

    def _send_motor_command(self, powers: WheelPowers) -> None:
        self.last_powers = powers
        power_by_name = {
            "top_left": powers.top_left,
            "top_right": powers.top_right,
            "bottom": powers.bottom,
        }
        for name, motor in self.motors.items():
            motor.send_heartbeat()
            motor.set_power(power_by_name[name])

    def _send_motor_heartbeat(self) -> None:
        for motor in self.motors.values():
            motor.send_heartbeat()

    def _stop_motors(self) -> None:
        if not self.hardware_active:
            return
        self._send_motor_command(WheelPowers(0.0, 0.0, 0.0))

    def _report_status(self, now: float) -> None:
        period = 1.0 / max(0.1, float(self.args.status_rate_hz))
        if now - self.last_status_time < period:
            return
        self.last_status_time = now
        age = None if self.last_command_time <= 0.0 else now - self.last_command_time
        self.logger.info(
            "motor_ipc active=%s age=%s latency=%.4f state=%.2f powers=%s",
            self.hardware_active,
            "none" if age is None else f"{age:.3f}",
            self.last_packet_latency_sec,
            self.last_source_state,
            self.last_powers.as_list(),
        )


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logger = logging.getLogger("motor_ipc_server")
    args = _build_parser().parse_args()
    server = MotorIpcServer(args, logger)
    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
