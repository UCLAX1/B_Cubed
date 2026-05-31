#!/usr/bin/env python3
"""
Read RTIMU orientation data and apply fixed counterbalance offsets.

The mounting pose is treated as the reference zero:
- roll starts around -160°, so we add +160°
- pitch starts around +75°, so we add -75°

This prints both the raw fused orientation and the adjusted values.
"""

import math
import sys
import time

SETTINGS_FILE = "RTIMULib"
ROLL_OFFSET_DEG = 160.0
PITCH_OFFSET_DEG = -75.0

sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU  # type: ignore[import-not-found]


def normalize_degrees(angle_deg: float) -> float:
    """Wrap an angle to the range [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def apply_counterbalance(roll_deg: float, pitch_deg: float) -> tuple[float, float]:
    """Apply the fixed mounting offsets and wrap the result."""
    adjusted_roll = normalize_degrees(roll_deg + ROLL_OFFSET_DEG)
    adjusted_pitch = normalize_degrees(pitch_deg + PITCH_OFFSET_DEG)
    return adjusted_roll, adjusted_pitch


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

    print("Reading IMU with counterbalance offsets...")
    print(f"Roll offset: {ROLL_OFFSET_DEG:+.1f}°")
    print(f"Pitch offset: {PITCH_OFFSET_DEG:+.1f}°")
    print("-" * 60)

    while True:
        if imu.IMURead():
            data = imu.getIMUData()
            fusion_pose = data["fusionPose"]

            roll_deg = math.degrees(fusion_pose[0])
            pitch_deg = math.degrees(fusion_pose[1])
            yaw_deg = math.degrees(fusion_pose[2])

            adjusted_roll, adjusted_pitch = apply_counterbalance(roll_deg, pitch_deg)

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