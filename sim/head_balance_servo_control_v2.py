import sys
import time
import math

from head_balance_math import find_motor_angles

DESIRED_ANGLE = 0.0

SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU

settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)

if not imu.IMUInit():
    print("IMU init failed")
    sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval() / 1000.0

try:
    while True:
        if imu.IMURead():
            data = imu.getIMUData()
            fusion_pose = data["fusionPose"]

            r2d = math.degrees
            roll = r2d(fusion_pose[0])
            pitch = r2d(fusion_pose[1])
            yaw = r2d(fusion_pose[2])

            arm_tgt, lazy_tgt, head_tgt = find_motor_angles(pitch, roll, DESIRED_ANGLE)

            print(f"IMU: R={roll:7.1f}° P={pitch:7.1f}° Y={yaw:7.1f}°")
            print(f"Targets: arm={arm_tgt:+7.2f}° lazy={lazy_tgt:+7.2f}° head={head_tgt:+7.2f}°")
            print("-" * 60)

        time.sleep(poll_interval)

except KeyboardInterrupt:
    print("\nStopped.")
