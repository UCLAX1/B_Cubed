#!/usr/bin/env python3
"""
Read RTIMU orientation data and zero the mounted IMU pose at startup.

The script samples the IMU while the robot is held in the flat reference
position, stores that pose as the offset, and then prints adjusted values so
the flat pose reads approximately 0 for roll and pitch.
"""

import math
import sys
import time

SETTINGS_FILE = "RTIMULib"
CALIBRATION_SAMPLES = 60

sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU  # type: ignore[import-not-found]


def normalize_degrees(angle_deg: float) -> float:
    """Wrap an angle to the range [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def apply_counterbalance(
    roll_deg: float,
    pitch_deg: float,
    roll_reference_deg: float,
    pitch_reference_deg: float,
) -> tuple[float, float]:
    """Subtract the startup reference pose and wrap the result."""
    adjusted_roll = normalize_degrees(roll_deg - roll_reference_deg)
    adjusted_pitch = normalize_degrees(pitch_deg - pitch_reference_deg)
    return adjusted_roll, adjusted_pitch


def calibrate_reference_pose(imu, poll_interval: float) -> tuple[float, float]:
    """Average a handful of samples to capture the current flat pose."""
    print(f"Calibrating flat pose from {CALIBRATION_SAMPLES} samples...")
    print("Hold the Pi in its flat reference position now.")

    roll_samples: list[float] = []
    pitch_samples: list[float] = []

    while len(roll_samples) < CALIBRATION_SAMPLES:
        if imu.IMURead():
            data = imu.getIMUData()
            fusion_pose = data["fusionPose"]
            roll_samples.append(math.degrees(fusion_pose[0]))
            pitch_samples.append(math.degrees(fusion_pose[1]))
        time.sleep(poll_interval)

    roll_reference_deg = sum(roll_samples) / len(roll_samples)
    pitch_reference_deg = sum(pitch_samples) / len(pitch_samples)

    print(
        f"Flat reference: R={roll_reference_deg:+.1f}°  "
        f"P={pitch_reference_deg:+.1f}°"
    )
    print("-" * 60)
    return roll_reference_deg, pitch_reference_deg


def main() -> int:
    settings = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(settings)

    if not imu.IMUInit():
        print("IMU init failed")
        return 1

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval() / 1000.0

    roll_reference_deg, pitch_reference_deg = calibrate_reference_pose(
        imu, poll_interval
    )

    print("Reading IMU with counterbalance offsets...")
    print("-" * 60)

    while True:
        if imu.IMURead():
            data = imu.getIMUData()
            fusion_pose = data["fusionPose"]

            roll_deg = math.degrees(fusion_pose[0])
            pitch_deg = math.degrees(fusion_pose[1])
            yaw_deg = math.degrees(fusion_pose[2])

            adjusted_roll, adjusted_pitch = apply_counterbalance(
                roll_deg,
                pitch_deg,
                roll_reference_deg,
                pitch_reference_deg,
            )

            print(
                f"Raw:   R={roll_deg:7.1f}  P={pitch_deg:7.1f}  Y={yaw_deg:7.1f}"
            )
            print(
                f"Adj:   R={adjusted_roll:7.1f}  P={adjusted_pitch:7.1f}  Y={yaw_deg:7.1f}"
            )
            print("-" * 60)

        time.sleep(poll_interval)


if __name__ == "__main__":
    raise SystemExit(main())