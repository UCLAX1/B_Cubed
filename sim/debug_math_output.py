"""Print what find_motor_angles wants to do, based on live IMU data. No servos move."""

import math
import sys
import time

DESIRED_ANGLE = 0.0
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")

ARM_MIN, ARM_MAX = -30.0, 30.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0
ARM_SERVO_MIN, ARM_SERVO_MAX = -0.5, 0.5
ARM_VERTICAL_OFFSET = 0.0


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def wrap_degrees(angle_deg):
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def find_motor_angles(pitch, roll, desired_angle):
    pitch_r = math.radians(pitch)
    roll_r = math.radians(roll)

    n1 = math.sin(pitch_r) * math.cos(roll_r)
    n2 = -math.sin(roll_r)
    n3 = math.cos(pitch_r) * math.cos(roll_r)

    arm = -math.degrees(math.acos(clamp(n3, -1.0, 1.0)))
    if pitch < 0:
        arm = -arm

    lazy = 0.0
    if roll != 0:
        denom = math.sqrt(n1**2 + n2**2)
        lazy = math.degrees(math.acos(clamp(n1 / denom, -1.0, 1.0))) if denom > 1e-9 else 0.0

    if roll < 0:
        lazy = -lazy
        arm = -arm

    if abs(lazy) > 90:
        arm = -arm
        lazy = lazy - 180 if lazy > 0 else lazy + 180

    return arm, lazy


def capture_reference(imu, poll_interval):
    print("Waiting for first IMU sample to set flat reference...")
    while True:
        if imu.IMURead():
            data = imu.getIMUData()
            fp = data["fusionPose"]
            r = math.degrees(fp[0])
            p = math.degrees(fp[1])
            print(f"Flat reference: R={r:+.2f}°  P={p:+.2f}°")
            return r, p
        time.sleep(poll_interval)


def main():
    import RTIMU  # type: ignore

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

    roll_ref, pitch_ref = capture_reference(imu, poll_interval)

    print("\nLive math output (no servos moving). Ctrl-C to stop.\n")
    print(f"{'IMU raw':>20}  {'adjusted':>20}  {'arm_tgt':>8}  {'lazy_tgt':>9}  {'arm_cmd':>8}")
    print("-" * 80)

    try:
        while True:
            if not imu.IMURead():
                time.sleep(poll_interval)
                continue

            fp = imu.getIMUData()["fusionPose"]
            raw_roll  = math.degrees(fp[0])
            raw_pitch = math.degrees(fp[1])
            yaw       = math.degrees(fp[2])

            adj_roll  = wrap_degrees(raw_roll  - roll_ref)
            adj_pitch = wrap_degrees(raw_pitch - pitch_ref)

            arm_tgt, lazy_tgt = find_motor_angles(adj_pitch, adj_roll, DESIRED_ANGLE)

            arm_tgt_clamped  = clamp(arm_tgt,  ARM_MIN, ARM_MAX)
            lazy_tgt_clamped = clamp(lazy_tgt, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
            arm_cmd = clamp((arm_tgt_clamped / 90.0) + ARM_VERTICAL_OFFSET,
                            ARM_SERVO_MIN, ARM_SERVO_MAX)

            print(
                f"R={raw_roll:+6.1f}° P={raw_pitch:+6.1f}°  "
                f"adjR={adj_roll:+6.1f}° adjP={adj_pitch:+6.1f}°  "
                f"arm={arm_tgt_clamped:+7.2f}°  "
                f"lazy={lazy_tgt_clamped:+7.2f}°  "
                f"arm_cmd={arm_cmd:+.3f}",
                flush=True,
            )
            time.sleep(poll_interval)

    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()
