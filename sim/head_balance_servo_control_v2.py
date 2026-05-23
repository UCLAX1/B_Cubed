import sys
import time
import math

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

            print(f"R={r2d(fusion_pose[0]):.1f} P={r2d(fusion_pose[1]):.1f} Y={r2d(fusion_pose[2]):.1f}")

        time.sleep(poll_interval)

except KeyboardInterrupt:
    print("\nStopped.")
