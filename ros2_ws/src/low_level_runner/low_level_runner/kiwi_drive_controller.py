#!/usr/bin/env python3
"""ROS 2 controller that converts Twist commands into Kiwi drive motor power."""

from __future__ import annotations

import json
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from low_level_runner.hardware_interface import CanBus, Motor
from low_level_runner.kiwi_kinematics import WheelPowers, twist_to_wheel_powers


def _zero_twist() -> Twist:
    return Twist()


class KiwiDriveController(Node):
    """Subscribe to Nav2/manual velocity commands and drive the CAN motors."""

    def __init__(self) -> None:
        super().__init__("kiwi_drive_controller")
        self._declare_parameters()

        self.nav_cmd_topic = str(self.get_parameter("nav_cmd_topic").value)
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.motor_command_topic = str(
            self.get_parameter("motor_command_topic").value
        )
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.control_rate_hz = max(
            1.0,
            float(self.get_parameter("control_rate_hz").value),
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

        self.last_nav_cmd = _zero_twist()
        self.last_manual_cmd = _zero_twist()
        self.last_nav_cmd_rx = None
        self.last_manual_cmd_rx = None
        self.last_source = "idle"
        self.last_powers = WheelPowers(0.0, 0.0, 0.0)
        self.last_status_summary = ""

        self.motor_command_pub = self.create_publisher(
            Float32MultiArray,
            self.motor_command_topic,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
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

        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_loop,
        )
        self.get_logger().info(
            f"Kiwi drive controller listening to nav={self.nav_cmd_topic}, "
            f"manual={self.manual_cmd_topic}; publishing {self.motor_command_topic}."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("nav_cmd_topic", "cmd_vel")
        self.declare_parameter("manual_cmd_topic", "cmd_vel_manual")
        self.declare_parameter("motor_command_topic", "kiwi_drive/motor_powers")
        self.declare_parameter("status_topic", "kiwi_drive/status")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("nav_cmd_timeout_sec", 0.50)
        self.declare_parameter("manual_cmd_timeout_sec", 0.30)

        self.declare_parameter("linear_to_power_scale", 1.0)
        self.declare_parameter("angular_to_power_scale", 1.0)
        self.declare_parameter("max_power", 1.0)
        self.declare_parameter("normalize_over_limit", False)

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

    def _manual_cmd_callback(self, msg: Twist) -> None:
        self.last_manual_cmd = msg
        self.last_manual_cmd_rx = self.get_clock().now()

    def _control_loop(self) -> None:
        command, source = self._select_command()
        powers = twist_to_wheel_powers(
            command.linear.x,
            command.linear.y,
            command.angular.z,
            linear_scale=self.linear_scale,
            angular_scale=self.angular_scale,
            max_power=self.max_power,
            normalize_over_limit=self.normalize_over_limit,
        )
        self.last_source = source
        self.last_powers = powers

        self._publish_motor_command(powers)
        self._send_motor_command(powers)
        self._publish_status(command, powers, source)

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

    def _publish_motor_command(self, powers: WheelPowers) -> None:
        self.motor_command_pub.publish(Float32MultiArray(data=powers.as_list()))

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

    def _publish_status(self, command: Twist, powers: WheelPowers, source: str) -> None:
        status = {
            "source": source,
            "hardware_enabled": self.hardware_enabled,
            "hardware_active": self.hardware_active,
            "nav_cmd_age_sec": self._age_sec(self.last_nav_cmd_rx),
            "manual_cmd_age_sec": self._age_sec(self.last_manual_cmd_rx),
            "cmd": {
                "linear_x": command.linear.x,
                "linear_y": command.linear.y,
                "angular_z": command.angular.z,
            },
            "motor_powers": {
                "top_left": powers.top_left,
                "top_right": powers.top_right,
                "bottom": powers.bottom,
            },
        }
        status_text = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(String(data=status_text))

        summary = (
            f"source={source}, hardware_active={self.hardware_active}, "
            f"powers={powers.as_list()}"
        )
        if summary != self.last_status_summary:
            self.get_logger().info(summary)
            self.last_status_summary = summary

    def _age_sec(self, stamp) -> float | None:
        if stamp is None:
            return None
        age = self.get_clock().now() - stamp
        return float(age.nanoseconds) / 1e9

    def _start_hardware(self) -> None:
        self.bus = CanBus(
            channel=self.can_channel,
            interface=self.can_interface,
            bitrate=self.can_bitrate,
            logger=self.get_logger(),
        )
        self.bus.start()
        if not self.bus.started_successfully():
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
            self.get_logger().error(f"Motor initialization failed: {error}")
            self._close_hardware()
            return

        self.hardware_active = True
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


def main(args=None) -> None:
    """Run the Kiwi drive controller node."""
    rclpy.init(args=args)
    node = KiwiDriveController()
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
