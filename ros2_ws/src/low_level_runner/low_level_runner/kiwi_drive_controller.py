#!/usr/bin/env python3
"""ROS 2 controller that converts Twist commands into Kiwi drive motor power."""

from __future__ import annotations

from pathlib import Path
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from low_level_runner.hardware_interface import CanBus, Motor
from low_level_runner.kiwi_kinematics import WheelPowers, twist_to_wheel_powers


def _zero_twist() -> Twist:
    return Twist()


class KiwiDriveController(Node):
    """Subscribe to Nav2/manual velocity commands and drive the CAN motors."""

    def __init__(self, node_name: str = "kiwi_drive_controller") -> None:
        super().__init__(node_name)
        self._declare_parameters()

        self.nav_cmd_topic = str(self.get_parameter("nav_cmd_topic").value)
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)

        self.heartbeat_rate_hz = max(
            1.0,
            float(self.get_parameter("heartbeat_rate_hz").value),
        )
        self.nav_cmd_timeout = Duration(
            seconds=float(self.get_parameter("nav_cmd_timeout_sec").value)
        )
        self.manual_cmd_timeout = Duration(
            seconds=float(self.get_parameter("manual_cmd_timeout_sec").value)
        )

        self.linear_scale = float(self.get_parameter("linear_to_power_scale").value)
        self.angular_scale = float(self.get_parameter("angular_to_power_scale").value)
        self.max_power = abs(float(self.get_parameter("max_power").value))
        self.normalize_over_limit = bool(
            self.get_parameter("normalize_over_limit").value
        )
        self.power_slew_rate = max(
            0.0,
            float(self.get_parameter("power_slew_rate_per_sec").value),
        )
        self.power_deadband = max(
            0.0,
            float(self.get_parameter("power_deadband").value),
        )
        self.min_power = max(
            0.0,
            float(self.get_parameter("min_power").value),
        )

        self.hardware_enabled = bool(self.get_parameter("hardware_enabled").value)
        self.can_channel = str(self.get_parameter("can_channel").value)
        self.can_interface = str(self.get_parameter("can_interface").value)
        self.can_bitrate = int(self.get_parameter("can_bitrate").value)
        self.max_duty_cycle = abs(float(self.get_parameter("max_duty_cycle").value))
        self.motor_wait_timeout_sec = float(
            self.get_parameter("motor_wait_timeout_sec").value
        )
        self.reset_encoders_on_start = bool(
            self.get_parameter("reset_encoders_on_start").value
        )
        self.init_pos_path = Path(
            str(self.get_parameter("init_pos_path").value)
        ).expanduser()

        self.motor_ids = {
            "top_left": int(self.get_parameter("top_left_motor_id").value),
            "top_right": int(self.get_parameter("top_right_motor_id").value),
            "bottom": int(self.get_parameter("bottom_motor_id").value),
        }
        self.motors: dict[str, Motor] = {}
        self.bus: CanBus | None = None
        self.hardware_active = False
        self.hardware_status = "disabled"
        self.hardware_error = ""

        self.last_nav_cmd = _zero_twist()
        self.last_manual_cmd = _zero_twist()
        self.last_nav_cmd_rx = None
        self.last_manual_cmd_rx = None
        self.last_source = "idle"
        self.last_powers = WheelPowers(0.0, 0.0, 0.0)
        self.commanded_powers = WheelPowers(0.0, 0.0, 0.0)
        self.last_power_update_time = self.get_clock().now()
        self.last_status_summary = ""

        self.create_subscription(Twist, self.nav_cmd_topic, self._nav_cmd_callback, 10)
        self.create_subscription(
            Twist,
            self.manual_cmd_topic,
            self._manual_cmd_callback,
            10,
        )

        if self.hardware_enabled:
            self._start_hardware()
        else:
            self.get_logger().warn("Hardware output is disabled; publishing dry-run.")

        self.heartbeat_timer = self.create_timer(
            1.0 / self.heartbeat_rate_hz,
            self._heartbeat_loop,
        )
        self.get_logger().info(
            f"Kiwi drive controller listening to nav={self.nav_cmd_topic}, "
            f"manual={self.manual_cmd_topic}; syncing CAN power commands to "
            "received ROS commands."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("nav_cmd_topic", "cmd_vel")
        self.declare_parameter("manual_cmd_topic", "cmd_vel_manual")
        self.declare_parameter("heartbeat_rate_hz", 50.0)
        self.declare_parameter("nav_cmd_timeout_sec", 0.50)
        self.declare_parameter("manual_cmd_timeout_sec", 0.30)

        self.declare_parameter("linear_to_power_scale", 1.0)
        self.declare_parameter("angular_to_power_scale", 1.0)
        self.declare_parameter("max_power", 1.0)
        self.declare_parameter("normalize_over_limit", False)
        self.declare_parameter("power_slew_rate_per_sec", 0.0)
        self.declare_parameter("power_deadband", 0.0)
        self.declare_parameter("min_power", 0.0)

        self.declare_parameter("hardware_enabled", True)
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_interface", "socketcan")
        self.declare_parameter("can_bitrate", 1000000)
        self.declare_parameter("max_duty_cycle", 0.5)
        self.declare_parameter("motor_wait_timeout_sec", 2.0)
        self.declare_parameter("reset_encoders_on_start", False)
        self.declare_parameter("init_pos_path", "motor_init_pos.json")

        self.declare_parameter("top_left_motor_id", 3)
        self.declare_parameter("top_right_motor_id", 9)
        self.declare_parameter("bottom_motor_id", 7)

    def _nav_cmd_callback(self, msg: Twist) -> None:
        self.last_nav_cmd = msg
        self.last_nav_cmd_rx = self.get_clock().now()
        if not self._manual_command_is_fresh():
            self._apply_current_command()

    def _manual_cmd_callback(self, msg: Twist) -> None:
        self.last_manual_cmd = msg
        self.last_manual_cmd_rx = self.get_clock().now()
        self._apply_current_command()

    def _apply_current_command(self) -> None:
        command, source = self._select_command()
        self._apply_command(command, source)

    def _apply_command(self, command: Twist, source: str) -> None:
        powers = twist_to_wheel_powers(
            command.linear.x,
            command.linear.y,
            command.angular.z,
            linear_scale=self.linear_scale,
            angular_scale=self.angular_scale,
            max_power=self.max_power,
            normalize_over_limit=self.normalize_over_limit,
        )
        command_age = self._command_age(source)
        powers = self._shape_low_power_outputs(powers)
        powers = self._slew_limit_powers(powers)
        self.last_source = source
        self.last_powers = powers

        self._send_motor_command(powers)
        self._log_status(powers, source, command_age)

    def _heartbeat_loop(self) -> None:
        command, source = self._select_command()
        if source == "idle":
            if self.last_source != "idle" or self.last_powers.as_list() != [0.0, 0.0, 0.0]:
                self._apply_command(_zero_twist(), "idle")
            else:
                self._send_motor_heartbeat()
                self._log_status(self.last_powers, source, self._command_age(source))
            return

        if self.power_slew_rate > 0.0:
            self._apply_command(command, source)
            return

        self._send_motor_heartbeat()
        self._log_status(self.last_powers, source, self._command_age(source))

    def _select_command(self) -> tuple[Twist, str]:
        if self._manual_command_is_fresh():
            return self.last_manual_cmd, "manual"

        if self._nav_command_is_fresh():
            return self.last_nav_cmd, "nav"

        return _zero_twist(), "idle"

    def _manual_command_is_fresh(self) -> bool:
        if self.last_manual_cmd_rx is None:
            return False
        return self._is_fresh(self.last_manual_cmd_rx, self.manual_cmd_timeout)

    def _nav_command_is_fresh(self) -> bool:
        if self.last_nav_cmd_rx is None:
            return False
        return self._is_fresh(self.last_nav_cmd_rx, self.nav_cmd_timeout)

    def _is_fresh(self, stamp, timeout: Duration) -> bool:
        return self.get_clock().now() - stamp <= timeout

    def _command_age(self, source: str) -> float | None:
        if source == "manual" and self.last_manual_cmd_rx is not None:
            return (self.get_clock().now() - self.last_manual_cmd_rx).nanoseconds / 1e9
        if source == "nav" and self.last_nav_cmd_rx is not None:
            return (self.get_clock().now() - self.last_nav_cmd_rx).nanoseconds / 1e9
        return None

    def _shape_low_power_outputs(self, powers: WheelPowers) -> WheelPowers:
        if self.power_deadband <= 0.0 and self.min_power <= 0.0:
            return powers

        return WheelPowers(
            *[
                self._shape_single_power(power)
                for power in powers.as_list()
            ]
        )

    def _shape_single_power(self, power: float) -> float:
        magnitude = abs(power)
        if magnitude <= self.power_deadband:
            return 0.0
        if self.min_power <= self.power_deadband:
            return power

        limit = max(self.max_power, self.min_power)
        if limit <= self.power_deadband:
            return power
        scaled = self.min_power + (
            (magnitude - self.power_deadband)
            * (limit - self.min_power)
            / (limit - self.power_deadband)
        )
        return self._copy_sign(min(scaled, limit), power)

    @staticmethod
    def _copy_sign(magnitude: float, sign_source: float) -> float:
        return magnitude if sign_source >= 0.0 else -magnitude

    def _slew_limit_powers(self, target: WheelPowers) -> WheelPowers:
        if self.power_slew_rate <= 0.0:
            self.commanded_powers = target
            self.last_power_update_time = self.get_clock().now()
            return target

        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_power_update_time).nanoseconds / 1e9)
        self.last_power_update_time = now
        max_delta = self.power_slew_rate * dt
        current = self.commanded_powers.as_list()
        limited = [
            self._move_toward(current_power, target_power, max_delta)
            for current_power, target_power in zip(current, target.as_list())
        ]
        self.commanded_powers = WheelPowers(*limited)
        return self.commanded_powers

    @staticmethod
    def _move_toward(current: float, target: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return current
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + max_delta * (1.0 if delta > 0.0 else -1.0)

    def _send_motor_command(self, powers: WheelPowers) -> None:
        if not self.hardware_active:
            return

        power_by_name = {
            "top_left": powers.top_left,
            "top_right": powers.top_right,
            "bottom": powers.bottom,
        }
        for name, motor in self.motors.items():
            motor.send_heartbeat()
            motor.set_power(power_by_name[name])

    def _send_motor_heartbeat(self) -> None:
        if self.power_slew_rate > 0.0:
            self.last_power_update_time = self.get_clock().now()

        if not self.hardware_active:
            return

        for motor in self.motors.values():
            motor.send_heartbeat()

    def _log_status(
        self,
        powers: WheelPowers,
        source: str,
        command_age: float | None,
    ) -> None:
        age_text = "none" if command_age is None else f"{command_age:.3f}"
        power_values = [round(power, 4) for power in powers.as_list()]
        summary_key = (
            f"source={source}, hardware_active={self.hardware_active}, "
            f"powers={power_values}"
        )
        if summary_key != self.last_status_summary:
            self.get_logger().info(f"{summary_key}, cmd_age_sec={age_text}")
            self.last_status_summary = summary_key

    def _start_hardware(self) -> None:
        self.hardware_status = "starting"
        self.hardware_error = ""
        self.bus = CanBus(
            channel=self.can_channel,
            interface=self.can_interface,
            bitrate=self.can_bitrate,
            logger=self.get_logger(),
        )
        self.bus.start()
        if not self.bus.started_successfully():
            self.hardware_status = "can_start_failed"
            self.hardware_error = f"CAN bus unavailable on {self.can_interface}:{self.can_channel}"
            self.get_logger().error("Running without CAN motor output.")
            return

        try:
            for name, motor_id in self.motor_ids.items():
                motor = Motor(
                    self.bus,
                    motor_id,
                    max_duty_cycle=self.max_duty_cycle,
                    init_pos_path=self.init_pos_path,
                    wait_timeout_sec=self.motor_wait_timeout_sec,
                    logger=self.get_logger(),
                )
                motor.set_power(0.0)
                if self.reset_encoders_on_start:
                    motor.reset_encoder()
                self.motors[name] = motor
        except (RuntimeError, TimeoutError) as error:
            self.hardware_status = "motor_init_failed"
            self.hardware_error = str(error)
            self.get_logger().error(f"Motor initialization failed: {error}")
            self._close_hardware()
            return

        self.hardware_active = True
        self.hardware_status = "active"
        self.hardware_error = ""
        self.get_logger().info(
            "Motor output enabled with IDs "
            f"top_left={self.motor_ids['top_left']}, "
            f"top_right={self.motor_ids['top_right']}, "
            f"bottom={self.motor_ids['bottom']}."
        )

    def _stop_motors(self) -> None:
        if not self.hardware_active:
            return
        for motor in self.motors.values():
            motor.send_heartbeat()
            motor.set_power(0.0)

    def _close_hardware(self) -> None:
        self.hardware_active = False
        if self.hardware_enabled and self.hardware_status == "active":
            self.hardware_status = "closed"
        self.motors = {}
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def destroy_node(self) -> bool:
        """Stop motors before shutting the node down."""
        self._stop_motors()
        time.sleep(0.02)
        self._close_hardware()
        return super().destroy_node()


def run_controller(args=None, *, node_name: str = "kiwi_drive_controller") -> None:
    """Run the Kiwi drive controller node."""
    rclpy.init(args=args)
    node = KiwiDriveController(node_name=node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    """Run the Kiwi drive controller node."""
    run_controller(args=args)


if __name__ == "__main__":
    main()
