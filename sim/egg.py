import sys, os, time

# RTIMULib needs a settings file
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

while True:
    if imu.IMURead():
        data = imu.getIMUData()
        fusionPose = data["fusionPose"]
        accel = data["accel"]
        gyro = data["gyro"]
        compass = data["compass"]

        import math
        r2d = math.degrees

        print(f"Orientation: R={r2d(fusionPose[0]):.1f} P={r2d(fusionPose[1]):.1f} Y={r2d(fusionPose[2]):.1f}")
        print(f"Accel:  X={accel[0]:.4f} Y={accel[1]:.4f} Z={accel[2]:.4f}")
        print(f"Gyro:   X={gyro[0]:.4f} Y={gyro[1]:.4f} Z={gyro[2]:.4f}")
        print(f"Mag:    X={compass[0]:.4f} Y={compass[1]:.4f} Z={compass[2]:.4f}")
        print("-" * 50)

    time.sleep(poll_interval)