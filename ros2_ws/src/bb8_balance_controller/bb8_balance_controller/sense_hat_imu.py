"""Publish Raspberry Pi IMU attitude as sensor_msgs/Imu."""

from __future__ import annotations

import math
import sys
from typing import Mapping

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from bb8_balance_controller.math_utils import rpy_to_quat, wrap_pi

if "/usr/lib/python3/dist-packages" not in sys.path:
    sys.path.append("/usr/lib/python3/dist-packages")

try:
    import RTIMU
except ImportError:  # pragma: no cover - only available on the Raspberry Pi.
    RTIMU = None

try:
    from sense_hat import SenseHat
except ImportError:  # pragma: no cover - only available on the Raspberry Pi.
    SenseHat = None


GRAVITY_M_S2 = 9.80665


def _diag_covariance(values: list[float]) -> list[float]:
    covariance = [0.0] * 9
    for index, value in enumerate(values[:3]):
        covariance[index * 4] = float(value)
    return covariance


class SenseHatImuNode(Node):
    """Bridge Raspberry Pi IMU readings into ROS IMU data."""

    def __init__(self) -> None:
        super().__init__("sense_hat_imu")
        self._declare_parameters()
        self._read_parameters()

        self.backend_name = ""
        self.sense = None
        self.rtimu = None
        self.rtimu_poll_interval_sec = 0.0
        self._start_backend()
        self._last_rtimu_not_ready_log = self.get_clock().now()

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.raw_tilt_pub = None
        if self.raw_tilt_topic:
            self.raw_tilt_pub = self.create_publisher(
                Float32MultiArray,
                self.raw_tilt_topic,
                10,
            )

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_imu)
        self.get_logger().info(
            f"Publishing {self.backend_name} IMU on {self.imu_topic} "
            f"at {self.publish_rate_hz:.1f} Hz."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("backend", "rtimu")
        self.declare_parameter("rtimu_settings_file", "RTIMULib")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("raw_tilt_topic", "sense_hat/raw")
        self.declare_parameter("frame_id", "sense_hat_imu_link")
        self.declare_parameter("publish_rate_hz", 50.0)

        self.declare_parameter("compass_enabled", True)
        self.declare_parameter("gyro_enabled", True)
        self.declare_parameter("accel_enabled", True)
        self.declare_parameter("orientation_in_degrees", True)
        self.declare_parameter("gyro_in_degrees_per_second", True)
        self.declare_parameter("accelerometer_in_g", True)
        self.declare_parameter("raw_tilt_degrees", True)

        self.declare_parameter("roll_source", "roll")
        self.declare_parameter("pitch_source", "pitch")
        self.declare_parameter("yaw_source", "yaw")
        self.declare_parameter("gyro_x_source", "x")
        self.declare_parameter("gyro_y_source", "y")
        self.declare_parameter("gyro_z_source", "z")
        self.declare_parameter("accel_x_source", "x")
        self.declare_parameter("accel_y_source", "y")
        self.declare_parameter("accel_z_source", "z")

        self.declare_parameter("roll_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("gyro_x_sign", 1.0)
        self.declare_parameter("gyro_y_sign", 1.0)
        self.declare_parameter("gyro_z_sign", 1.0)
        self.declare_parameter("accel_x_sign", 1.0)
        self.declare_parameter("accel_y_sign", 1.0)
        self.declare_parameter("accel_z_sign", 1.0)

        self.declare_parameter("roll_offset_rad", 0.0)
        self.declare_parameter("pitch_offset_rad", 0.0)
        self.declare_parameter("yaw_offset_rad", 0.0)

        self.declare_parameter("orientation_covariance_diagonal", [0.0025] * 3)
        self.declare_parameter("angular_velocity_covariance_diagonal", [0.01] * 3)
        self.declare_parameter("linear_acceleration_covariance_diagonal", [0.25] * 3)

    def _read_parameters(self) -> None:
        self.backend = str(self.get_parameter("backend").value)
        self.rtimu_settings_file = str(
            self.get_parameter("rtimu_settings_file").value
        )
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.raw_tilt_topic = str(self.get_parameter("raw_tilt_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )

        self.compass_enabled = bool(self.get_parameter("compass_enabled").value)
        self.gyro_enabled = bool(self.get_parameter("gyro_enabled").value)
        self.accel_enabled = bool(self.get_parameter("accel_enabled").value)
        self.orientation_in_degrees = bool(
            self.get_parameter("orientation_in_degrees").value
        )
        self.gyro_in_degrees_per_second = bool(
            self.get_parameter("gyro_in_degrees_per_second").value
        )
        self.accelerometer_in_g = bool(
            self.get_parameter("accelerometer_in_g").value
        )
        self.raw_tilt_degrees = bool(self.get_parameter("raw_tilt_degrees").value)

        self.roll_source = str(self.get_parameter("roll_source").value)
        self.pitch_source = str(self.get_parameter("pitch_source").value)
        self.yaw_source = str(self.get_parameter("yaw_source").value)
        self.gyro_sources = [
            str(self.get_parameter("gyro_x_source").value),
            str(self.get_parameter("gyro_y_source").value),
            str(self.get_parameter("gyro_z_source").value),
        ]
        self.accel_sources = [
            str(self.get_parameter("accel_x_source").value),
            str(self.get_parameter("accel_y_source").value),
            str(self.get_parameter("accel_z_source").value),
        ]

        self.roll_sign = float(self.get_parameter("roll_sign").value)
        self.pitch_sign = float(self.get_parameter("pitch_sign").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.gyro_signs = [
            float(self.get_parameter("gyro_x_sign").value),
            float(self.get_parameter("gyro_y_sign").value),
            float(self.get_parameter("gyro_z_sign").value),
        ]
        self.accel_signs = [
            float(self.get_parameter("accel_x_sign").value),
            float(self.get_parameter("accel_y_sign").value),
            float(self.get_parameter("accel_z_sign").value),
        ]

        self.roll_offset = float(self.get_parameter("roll_offset_rad").value)
        self.pitch_offset = float(self.get_parameter("pitch_offset_rad").value)
        self.yaw_offset = float(self.get_parameter("yaw_offset_rad").value)

        self.orientation_covariance = _diag_covariance(
            list(self.get_parameter("orientation_covariance_diagonal").value)
        )
        self.angular_velocity_covariance = _diag_covariance(
            list(self.get_parameter("angular_velocity_covariance_diagonal").value)
        )
        self.linear_acceleration_covariance = _diag_covariance(
            list(self.get_parameter("linear_acceleration_covariance_diagonal").value)
        )

    def _start_backend(self) -> None:
        backend = self.backend.lower()
        if backend in ("rtimu", "auto") and RTIMU is not None:
            self._start_rtimu_backend()
            return

        if backend == "rtimu":
            raise RuntimeError("RTIMU is not installed; cannot start IMU backend.")

        if backend in ("sense_hat", "sensehat", "auto") and SenseHat is not None:
            self._start_sense_hat_backend()
            return

        if backend in ("sense_hat", "sensehat"):
            raise RuntimeError(
                "sense_hat is not installed. Install python3-sense-hat on the Pi."
            )

        raise RuntimeError(
            "No IMU backend available. Install RTIMU or set backend:=sense_hat."
        )

    def _start_rtimu_backend(self) -> None:
        settings = RTIMU.Settings(self.rtimu_settings_file)
        imu = RTIMU.RTIMU(settings)
        if not imu.IMUInit():
            raise RuntimeError("RTIMU IMUInit failed.")

        imu.setSlerpPower(0.02)
        imu.setGyroEnable(self.gyro_enabled)
        imu.setAccelEnable(self.accel_enabled)
        imu.setCompassEnable(self.compass_enabled)

        self.rtimu = imu
        self.backend_name = "RTIMU"
        if hasattr(imu, "IMUGetPollInterval"):
            self.rtimu_poll_interval_sec = max(
                0.0,
                float(imu.IMUGetPollInterval()) / 1000.0,
            )

    def _start_sense_hat_backend(self) -> None:
        self.sense = SenseHat()
        self.sense.set_imu_config(
            self.compass_enabled,
            self.gyro_enabled,
            self.accel_enabled,
        )
        self.backend_name = "Sense HAT"

    def _publish_imu(self) -> None:
        try:
            orientation, gyro, accel = self._read_sample()
        except RuntimeError as exc:
            if str(exc) == "RTIMU sample is not ready yet.":
                return
            self.get_logger().error(
                f"Failed to read IMU: {exc}",
                throttle_duration_sec=2.0,
            )
            return
        except Exception as exc:  # noqa: BLE001 - keep ROS node alive for retry.
            self.get_logger().error(
                f"Failed to read IMU: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        roll = self._mapped_angle(
            orientation,
            self.roll_source,
            self.roll_sign,
            self.roll_offset,
        )
        pitch = self._mapped_angle(
            orientation,
            self.pitch_source,
            self.pitch_sign,
            self.pitch_offset,
        )
        yaw = self._mapped_angle(
            orientation,
            self.yaw_source,
            self.yaw_sign,
            self.yaw_offset,
        )
        quat = rpy_to_quat(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        msg.orientation_covariance = self.orientation_covariance

        angular_velocity = self._mapped_vector(gyro, self.gyro_sources, self.gyro_signs)
        if self.gyro_in_degrees_per_second:
            angular_velocity = [math.radians(value) for value in angular_velocity]
        msg.angular_velocity.x = angular_velocity[0]
        msg.angular_velocity.y = angular_velocity[1]
        msg.angular_velocity.z = angular_velocity[2]
        msg.angular_velocity_covariance = self.angular_velocity_covariance

        linear_acceleration = self._mapped_vector(
            accel,
            self.accel_sources,
            self.accel_signs,
        )
        if self.accelerometer_in_g:
            linear_acceleration = [value * GRAVITY_M_S2 for value in linear_acceleration]
        msg.linear_acceleration.x = linear_acceleration[0]
        msg.linear_acceleration.y = linear_acceleration[1]
        msg.linear_acceleration.z = linear_acceleration[2]
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        self.imu_pub.publish(msg)
        self._publish_raw_tilt(roll, pitch)

    def _read_sample(
        self,
    ) -> tuple[Mapping[str, float], Mapping[str, float], Mapping[str, float]]:
        if self.rtimu is not None:
            return self._read_rtimu_sample()
        if self.sense is not None:
            return self._read_sense_hat_sample()
        raise RuntimeError("IMU backend is not initialized.")

    def _read_rtimu_sample(
        self,
    ) -> tuple[Mapping[str, float], Mapping[str, float], Mapping[str, float]]:
        if not self.rtimu.IMURead():
            raise RuntimeError("RTIMU sample is not ready yet.")

        data = self.rtimu.getIMUData()
        roll, pitch, yaw = data["fusionPose"]
        gyro_x, gyro_y, gyro_z = data.get("gyro", (0.0, 0.0, 0.0))
        accel_x, accel_y, accel_z = data.get("accel", (0.0, 0.0, 0.0))

        orientation = {
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
        }
        gyro = {"x": float(gyro_x), "y": float(gyro_y), "z": float(gyro_z)}
        accel = {"x": float(accel_x), "y": float(accel_y), "z": float(accel_z)}
        return orientation, gyro, accel

    def _read_sense_hat_sample(
        self,
    ) -> tuple[Mapping[str, float], Mapping[str, float], Mapping[str, float]]:
        return (
            self._read_sense_hat_orientation_radians(),
            self.sense.get_gyroscope_raw(),
            self.sense.get_accelerometer_raw(),
        )

    def _read_sense_hat_orientation_radians(self) -> Mapping[str, float]:
        if self.orientation_in_degrees:
            if hasattr(self.sense, "get_orientation_degrees"):
                orientation = self.sense.get_orientation_degrees()
            else:
                orientation = self.sense.get_orientation()
            return {
                key: math.radians(float(value))
                for key, value in orientation.items()
            }

        if hasattr(self.sense, "get_orientation_radians"):
            return self.sense.get_orientation_radians()

        orientation = self.sense.get_orientation()
        return {
            key: math.radians(float(value))
            for key, value in orientation.items()
        }

    def _mapped_angle(
        self,
        values: Mapping[str, float],
        key: str,
        sign: float,
        offset: float,
    ) -> float:
        return wrap_pi(sign * float(values[key]) - offset)

    def _mapped_vector(
        self,
        values: Mapping[str, float],
        keys: list[str],
        signs: list[float],
    ) -> list[float]:
        return [sign * float(values[key]) for key, sign in zip(keys, signs)]

    def _publish_raw_tilt(self, roll: float, pitch: float) -> None:
        if self.raw_tilt_pub is None:
            return
        scale = 180.0 / math.pi if self.raw_tilt_degrees else 1.0
        msg = Float32MultiArray()
        msg.data = [roll * scale, pitch * scale]
        self.raw_tilt_pub.publish(msg)


def main(args=None) -> None:
    """Run the Sense HAT IMU publisher."""
    rclpy.init(args=args)
    node = SenseHatImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
