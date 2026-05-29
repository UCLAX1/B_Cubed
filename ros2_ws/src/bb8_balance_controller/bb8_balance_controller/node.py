"""ROS 2 residual balance controller for B_Cubed."""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String

from bb8_balance_controller.math_utils import clip_norm, quat_to_roll_pitch
from bb8_balance_controller.policy import load_policy_or_pd
from low_level_runner.hardware_interface import CanBus, Motor
from low_level_runner.kiwi_kinematics import WheelPowers, twist_to_wheel_powers


def _zero_twist() -> Twist:
    return Twist()


class BalanceControllerNode(Node):
    """Combine Nav2 velocity with a bounded learned balance residual."""

    def __init__(self) -> None:
        super().__init__("bb8_balance_controller")
        self._declare_parameters()
        self._read_parameters()

        self.policy, self.policy_backend = load_policy_or_pd(
            self.policy_path,
            self.prefer_cuda,
            self.pd_kp,
            self.pd_kd,
            self.max_balance_accel,
            self.roll_correction_sign,
            self.pitch_correction_sign,
            self.allow_pd_fallback,
        )

        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.last_attitude_rx = None
        self.previous_tilt_stamp = None
        self.previous_tilt = None

        self.nav_command = np.zeros(3, dtype=np.float32)
        self.manual_command = np.zeros(3, dtype=np.float32)
        self.previous_command_xy = np.zeros(2, dtype=np.float32)
        self.command_accel = np.zeros(2, dtype=np.float32)
        self.last_nav_rx = None
        self.last_manual_rx = None

        self.odom_velocity = np.zeros(2, dtype=np.float32)
        self.last_odom_rx = None

        self.previous_action = np.zeros(2, dtype=np.float32)
        self.filtered_action = np.zeros(2, dtype=np.float32)
        self.balance_velocity = np.zeros(2, dtype=np.float32)
        self.output_velocity = np.zeros(2, dtype=np.float32)
        self.last_command_source = "idle"
        self.last_control_time = self.get_clock().now()
        self.last_status_summary = ""
        self.last_powers = WheelPowers(0.0, 0.0, 0.0)
        self.bus: CanBus | None = None
        self.motors: dict[str, Motor] = {}
        self.hardware_active = False
        self.hardware_status = "disabled"
        self.hardware_error = ""

        self.cmd_pub = (
            self.create_publisher(Twist, self.output_twist_topic, 10)
            if self.publish_ros_outputs and self.output_twist_topic
            else None
        )
        self.jet_cmd_pub = (
            self.create_publisher(Float32MultiArray, self.jet_cmd_topic, 10)
            if self.publish_ros_outputs and self.jet_cmd_topic
            else None
        )
        self.status_pub = (
            self.create_publisher(String, self.status_topic, 10)
            if self.publish_status_topic and self.status_topic
            else None
        )

        self.create_subscription(Twist, self.nav_cmd_topic, self._nav_callback, 10)
        if self.manual_cmd_topic:
            self.create_subscription(
                Twist,
                self.manual_cmd_topic,
                self._manual_callback,
                10,
            )
        if self.imu_topic:
            self.create_subscription(Imu, self.imu_topic, self._imu_callback, 10)
        if self.tilt_topic:
            self.create_subscription(
                Float32MultiArray, self.tilt_topic, self._tilt_callback, 10
            )
        if self.odom_topic:
            self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)

        if self.direct_motor_output_enabled:
            self._start_hardware()
        else:
            self.get_logger().info("Direct CAN motor output is disabled.")

        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.create_timer(1.0 / self.status_rate_hz, self._report_status)

        self.get_logger().info(
            "Balance controller ready: "
            f"policy_backend={self.policy_backend}, "
            f"nav_cmd_topic={self.nav_cmd_topic}, "
            f"manual_cmd_topic={self.manual_cmd_topic}, "
            f"direct_motor_output={self.direct_motor_output_enabled}, "
            f"publish_ros_outputs={self.publish_ros_outputs}, "
            f"publish_status_topic={self.publish_status_topic}."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("nav_cmd_topic", "cmd_vel")
        self.declare_parameter("manual_cmd_topic", "cmd_vel_manual")
        self.declare_parameter("output_twist_topic", "cmd_vel_balanced")
        self.declare_parameter("jet_cmd_topic", "jet_cmd")
        self.declare_parameter("status_topic", "balance/status")
        self.declare_parameter("publish_ros_outputs", False)
        self.declare_parameter("publish_status_topic", False)
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("tilt_topic", "sense_hat/raw")
        self.declare_parameter("tilt_topic_degrees", True)
        self.declare_parameter("odom_topic", "")

        self.declare_parameter("policy_path", "")
        self.declare_parameter("allow_pd_fallback", False)
        self.declare_parameter("prefer_cuda", False)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("require_attitude", True)
        self.declare_parameter("zero_on_stale_nav", True)
        self.declare_parameter("attitude_timeout_sec", 0.25)
        self.declare_parameter("nav_timeout_sec", 0.5)
        self.declare_parameter("manual_timeout_sec", 0.30)
        self.declare_parameter("odom_timeout_sec", 0.5)

        self.declare_parameter("roll_offset_rad", 0.0)
        self.declare_parameter("pitch_offset_rad", 0.0)
        self.declare_parameter("roll_correction_sign", -1.0)
        self.declare_parameter("pitch_correction_sign", 1.0)
        self.declare_parameter("pd_kp", 3.0)
        self.declare_parameter("pd_kd", 0.45)

        self.declare_parameter("max_nav_speed_m_s", 1.0)
        self.declare_parameter("max_balance_accel_m_s2", 0.8)
        self.declare_parameter("max_correction_speed_m_s", 0.18)
        self.declare_parameter("max_output_accel_m_s2", 1.8)
        self.declare_parameter("balance_velocity_leak_per_s", 1.2)
        self.declare_parameter("action_filter_alpha", 0.65)
        self.declare_parameter("idle_action_nav_deadband_m_s", 0.03)
        self.declare_parameter("idle_action_tilt_deadband_rad", 0.035)
        self.declare_parameter("idle_action_rate_deadband_rad_s", 0.2)
        self.declare_parameter("tilt_cutoff_rad", 0.65)
        self.declare_parameter("servo_neutral", [0.0, 0.0, 0.0])
        self.declare_parameter("active_state_value", 1.0)
        self.declare_parameter("stopped_state_value", 0.0)

        self.declare_parameter("direct_motor_output_enabled", False)
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
        self.declare_parameter("linear_to_power_scale", 1.0)
        self.declare_parameter("angular_to_power_scale", 1.0)
        self.declare_parameter("max_power", 1.0)
        self.declare_parameter("normalize_over_limit", False)

    def _read_parameters(self) -> None:
        self.nav_cmd_topic = str(self.get_parameter("nav_cmd_topic").value)
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.output_twist_topic = str(self.get_parameter("output_twist_topic").value)
        self.jet_cmd_topic = str(self.get_parameter("jet_cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.publish_ros_outputs = bool(self.get_parameter("publish_ros_outputs").value)
        self.publish_status_topic = bool(self.get_parameter("publish_status_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.tilt_topic = str(self.get_parameter("tilt_topic").value)
        self.tilt_topic_degrees = bool(self.get_parameter("tilt_topic_degrees").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.policy_path = str(self.get_parameter("policy_path").value)
        self.allow_pd_fallback = bool(self.get_parameter("allow_pd_fallback").value)
        self.prefer_cuda = bool(self.get_parameter("prefer_cuda").value)
        self.control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.status_rate_hz = max(0.5, float(self.get_parameter("status_rate_hz").value))
        self.require_attitude = bool(self.get_parameter("require_attitude").value)
        self.zero_on_stale_nav = bool(self.get_parameter("zero_on_stale_nav").value)
        self.attitude_timeout = Duration(
            seconds=float(self.get_parameter("attitude_timeout_sec").value)
        )
        self.nav_timeout = Duration(
            seconds=float(self.get_parameter("nav_timeout_sec").value)
        )
        self.manual_timeout = Duration(
            seconds=float(self.get_parameter("manual_timeout_sec").value)
        )
        self.odom_timeout = Duration(
            seconds=float(self.get_parameter("odom_timeout_sec").value)
        )

        self.roll_offset = float(self.get_parameter("roll_offset_rad").value)
        self.pitch_offset = float(self.get_parameter("pitch_offset_rad").value)
        self.roll_correction_sign = float(
            self.get_parameter("roll_correction_sign").value
        )
        self.pitch_correction_sign = float(
            self.get_parameter("pitch_correction_sign").value
        )
        self.pd_kp = float(self.get_parameter("pd_kp").value)
        self.pd_kd = float(self.get_parameter("pd_kd").value)

        self.max_nav_speed = float(self.get_parameter("max_nav_speed_m_s").value)
        self.max_balance_accel = float(
            self.get_parameter("max_balance_accel_m_s2").value
        )
        self.max_correction_speed = float(
            self.get_parameter("max_correction_speed_m_s").value
        )
        self.max_output_accel = float(
            self.get_parameter("max_output_accel_m_s2").value
        )
        self.balance_velocity_leak = float(
            self.get_parameter("balance_velocity_leak_per_s").value
        )
        self.action_filter_alpha = float(self.get_parameter("action_filter_alpha").value)
        self.idle_action_nav_deadband = float(
            self.get_parameter("idle_action_nav_deadband_m_s").value
        )
        self.idle_action_tilt_deadband = float(
            self.get_parameter("idle_action_tilt_deadband_rad").value
        )
        self.idle_action_rate_deadband = float(
            self.get_parameter("idle_action_rate_deadband_rad_s").value
        )
        self.tilt_cutoff = float(self.get_parameter("tilt_cutoff_rad").value)
        servo_neutral = list(self.get_parameter("servo_neutral").value)
        self.servo_neutral = [float(value) for value in servo_neutral[:3]]
        while len(self.servo_neutral) < 3:
            self.servo_neutral.append(0.0)
        self.active_state_value = float(self.get_parameter("active_state_value").value)
        self.stopped_state_value = float(self.get_parameter("stopped_state_value").value)
        self.direct_motor_output_enabled = bool(
            self.get_parameter("direct_motor_output_enabled").value
        )
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
        self.init_pos_path = str(self.get_parameter("init_pos_path").value)
        self.motor_ids = {
            "top_left": int(self.get_parameter("top_left_motor_id").value),
            "top_right": int(self.get_parameter("top_right_motor_id").value),
            "bottom": int(self.get_parameter("bottom_motor_id").value),
        }
        self.linear_scale = float(self.get_parameter("linear_to_power_scale").value)
        self.angular_scale = float(self.get_parameter("angular_to_power_scale").value)
        self.max_power = abs(float(self.get_parameter("max_power").value))
        self.normalize_over_limit = bool(
            self.get_parameter("normalize_over_limit").value
        )

    def _nav_callback(self, msg: Twist) -> None:
        self.nav_command = self._twist_to_command(msg)
        self.last_nav_rx = self.get_clock().now()

    def _manual_callback(self, msg: Twist) -> None:
        self.manual_command = self._twist_to_command(msg)
        self.last_manual_rx = self.get_clock().now()

    def _twist_to_command(self, msg: Twist) -> np.ndarray:
        command_xy = np.array([msg.linear.x, msg.linear.y], dtype=np.float32)
        command_xy = clip_norm(command_xy, self.max_nav_speed)
        return np.array(
            [command_xy[0], command_xy[1], msg.angular.z],
            dtype=np.float32,
        )

    def _imu_callback(self, msg: Imu) -> None:
        if msg.orientation_covariance[0] == -1.0:
            self.get_logger().warn(
                "IMU orientation covariance marks orientation unavailable.",
                throttle_duration_sec=2.0,
            )
            return
        roll, pitch = quat_to_roll_pitch(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.roll = roll - self.roll_offset
        self.pitch = pitch - self.pitch_offset
        self.roll_rate = float(msg.angular_velocity.x)
        self.pitch_rate = float(msg.angular_velocity.y)
        self.last_attitude_rx = self.get_clock().now()

    def _tilt_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        now = self.get_clock().now()
        scale = math.pi / 180.0 if self.tilt_topic_degrees else 1.0
        roll = float(msg.data[0]) * scale - self.roll_offset
        pitch = float(msg.data[1]) * scale - self.pitch_offset
        if self.previous_tilt is not None and self.previous_tilt_stamp is not None:
            dt = self._seconds_since(self.previous_tilt_stamp, now)
            if dt > 1e-4:
                self.roll_rate = (roll - self.previous_tilt[0]) / dt
                self.pitch_rate = (pitch - self.previous_tilt[1]) / dt
        self.roll = roll
        self.pitch = pitch
        self.previous_tilt = (roll, pitch)
        self.previous_tilt_stamp = now
        self.last_attitude_rx = now

    def _odom_callback(self, msg: Odometry) -> None:
        self.odom_velocity = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y],
            dtype=np.float32,
        )
        self.last_odom_rx = self.get_clock().now()

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        dt = max(1e-3, self._seconds_since(self.last_control_time, now))
        self.last_control_time = now

        safe, reason = self._safety_state(now)
        if not safe:
            self._publish_stop(reason)
            return

        command, source = self._select_command(now)
        command_xy = command[:2]
        self.command_accel = (command_xy - self.previous_command_xy) / dt
        self.previous_command_xy = command_xy.copy()
        measured_velocity = self._measured_body_velocity(now)
        obs = np.array(
            [
                self.roll,
                self.pitch,
                self.roll_rate,
                self.pitch_rate,
                measured_velocity[0],
                measured_velocity[1],
                command_xy[0],
                command_xy[1],
                self.command_accel[0],
                self.command_accel[1],
                self.previous_action[0],
                self.previous_action[1],
            ],
            dtype=np.float32,
        )

        action = np.asarray(self.policy(obs), dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)
        alpha = min(0.98, max(0.0, self.action_filter_alpha))
        if self._is_stable_idle(command_xy):
            action = np.zeros(2, dtype=np.float32)
            self.filtered_action[:] = 0.0
        else:
            self.filtered_action = alpha * self.filtered_action + (1.0 - alpha) * action
        self.previous_action = action

        balance_accel = self.filtered_action * self.max_balance_accel
        self.balance_velocity += balance_accel * dt
        leak = max(0.0, 1.0 - self.balance_velocity_leak * dt)
        self.balance_velocity *= leak
        self.balance_velocity = clip_norm(self.balance_velocity, self.max_correction_speed)

        target_velocity = clip_norm(
            command_xy + self.balance_velocity,
            self.max_nav_speed + self.max_correction_speed,
        )
        max_delta = self.max_output_accel * dt
        delta = clip_norm(target_velocity - self.output_velocity, max_delta)
        self.output_velocity += delta

        self._emit_command(
            self.output_velocity,
            float(command[2]),
            self.active_state_value,
        )
        self.last_command_source = source

    def _safety_state(self, now) -> tuple[bool, str]:
        if self.require_attitude and not self._fresh(
            self.last_attitude_rx, self.attitude_timeout, now
        ):
            return False, "stale_attitude"
        if self.zero_on_stale_nav and not self._has_fresh_command(now):
            return False, "stale_command"
        if np.linalg.norm([self.roll, self.pitch]) > self.tilt_cutoff:
            return False, "tilt_cutoff"
        return True, "ready"

    def _select_command(self, now) -> tuple[np.ndarray, str]:
        if self._fresh(self.last_manual_rx, self.manual_timeout, now):
            return self.manual_command, "manual"
        if self._fresh(self.last_nav_rx, self.nav_timeout, now):
            return self.nav_command, "nav"
        return np.zeros(3, dtype=np.float32), "idle"

    def _has_fresh_command(self, now) -> bool:
        return self._fresh(
            self.last_manual_rx,
            self.manual_timeout,
            now,
        ) or self._fresh(self.last_nav_rx, self.nav_timeout, now)

    def _measured_body_velocity(self, now) -> np.ndarray:
        if self._fresh(self.last_odom_rx, self.odom_timeout, now):
            return self.odom_velocity
        return self.output_velocity

    def _is_stable_idle(self, nav_xy: np.ndarray) -> bool:
        tilt_norm = float(np.linalg.norm([self.roll, self.pitch]))
        rate_norm = float(np.linalg.norm([self.roll_rate, self.pitch_rate]))
        return (
            float(np.linalg.norm(nav_xy)) < self.idle_action_nav_deadband
            and tilt_norm < self.idle_action_tilt_deadband
            and rate_norm < self.idle_action_rate_deadband
        )

    def _publish_stop(self, reason: str) -> None:
        self.filtered_action[:] = 0.0
        self.previous_action[:] = 0.0
        self.previous_command_xy[:] = 0.0
        self.command_accel[:] = 0.0
        self.balance_velocity[:] = 0.0
        self.output_velocity[:] = 0.0
        self.last_command_source = "idle"
        self._emit_command(np.zeros(2, dtype=np.float32), 0.0, self.stopped_state_value)
        self.last_status_summary = reason

    def _emit_command(
        self,
        velocity_xy: np.ndarray,
        yaw_rate: float,
        state_value: float,
    ) -> None:
        powers = twist_to_wheel_powers(
            float(velocity_xy[0]),
            float(velocity_xy[1]),
            float(yaw_rate),
            linear_scale=self.linear_scale,
            angular_scale=self.angular_scale,
            max_power=self.max_power,
            normalize_over_limit=self.normalize_over_limit,
        )
        self.last_powers = powers
        self._send_motor_command(powers)

        if self.cmd_pub is None and self.jet_cmd_pub is None:
            return

        twist = _zero_twist()
        twist.linear.x = float(velocity_xy[0])
        twist.linear.y = float(velocity_xy[1])
        twist.angular.z = float(yaw_rate)
        if self.cmd_pub is not None:
            self.cmd_pub.publish(twist)

        if self.jet_cmd_pub is not None:
            jet_cmd = Float32MultiArray()
            jet_cmd.data = [
                float(velocity_xy[0]),
                float(velocity_xy[1]),
                float(yaw_rate),
                self.servo_neutral[0],
                self.servo_neutral[1],
                self.servo_neutral[2],
                float(state_value),
            ]
            self.jet_cmd_pub.publish(jet_cmd)

    def _report_status(self) -> None:
        now = self.get_clock().now()
        safe, reason = self._safety_state(now)
        status = (
            f"safe={safe}, reason={reason}, backend={self.policy_backend}, "
            f"command_source={self.last_command_source}, "
            f"hardware={self.hardware_status}, "
            f"roll={self.roll:.3f}, pitch={self.pitch:.3f}, "
            f"balance_v=({self.balance_velocity[0]:.3f},{self.balance_velocity[1]:.3f}), "
            f"out_v=({self.output_velocity[0]:.3f},{self.output_velocity[1]:.3f}), "
            f"powers={self.last_powers.as_list()}"
        )
        if self.status_pub is not None:
            self.status_pub.publish(String(data=status))
        summary = f"{safe}:{reason}:{self.policy_backend}"
        if summary != self.last_status_summary:
            self.get_logger().info(status)
            self.last_status_summary = summary

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
            self.get_logger().error(self.hardware_error)
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
        self.get_logger().info(
            "Direct CAN motor output active: "
            f"top_left={self.motor_ids['top_left']}, "
            f"top_right={self.motor_ids['top_right']}, "
            f"bottom={self.motor_ids['bottom']}."
        )

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

    def _stop_motors(self) -> None:
        if not self.hardware_active:
            return
        for motor in self.motors.values():
            motor.send_heartbeat()
            motor.set_power(0.0)

    def _close_hardware(self) -> None:
        self.hardware_active = False
        if self.direct_motor_output_enabled and self.hardware_status == "active":
            self.hardware_status = "closed"
        self.motors = {}
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def destroy_node(self) -> bool:
        self._stop_motors()
        time.sleep(0.02)
        self._close_hardware()
        return super().destroy_node()

    def _fresh(self, stamp: Optional[object], timeout: Duration, now) -> bool:
        if stamp is None:
            return False
        return now - stamp <= timeout

    @staticmethod
    def _seconds_since(previous, now) -> float:
        if previous is None:
            return 0.0
        return float((now - previous).nanoseconds) / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BalanceControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
