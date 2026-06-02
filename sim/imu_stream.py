#!/usr/bin/env python3
"""Stream RTIMU fusion poses as CSV for the balance controller."""

import os
import sys
import time

sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU  # type: ignore[import-not-found]  # noqa: E402


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(SCRIPT_DIR, "RTIMULib")


def main():
    imu = RTIMU.RTIMU(RTIMU.Settings(SETTINGS_FILE))
    if not imu.IMUInit():
        print("IMU init failed", file=sys.stderr, flush=True)
        return 1

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)
    poll_interval = imu.IMUGetPollInterval() / 1000.0

    while True:
        if imu.IMURead():
            roll, pitch, yaw = imu.getIMUData()["fusionPose"]
            print(f"{roll},{pitch},{yaw}", flush=True)
        time.sleep(poll_interval)


if __name__ == "__main__":
    raise SystemExit(main())
