"""
Head balance debug (no servo movement): read IMU, calculate targets/commands, print.
Same as head_balance_servo_control_slow.py but doesn't initialize or move servos.
"""

import math
import sys
import time

from head_balance_math import find_motor_angles
from servo_control_filters import PIDCommandDamper

DEBUG = True
DESIRED_ANGLE = 0.0

SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU

ARM_MIN, ARM_MAX = -30.0, 30.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0
HEAD_MIN, HEAD_MAX = -float('inf'), float('inf')

ARM_VERTICAL_OFFSET = -0.66
LAZY_CPR = 1493
HEAD_CPR = 2048

IMU_ALPHA = 0.5
ARM_DEADBAND_DEG = 1.0
CONT_DEADBAND_DEG = 2.0
CONT_MAX_ANGLE_SPEED = 0.5

ARM_STEP = 0.005

ARM_PD_KP = 8.0
ARM_PD_KI = 0.0
ARM_PD_KD = 0.35
ARM_PD_MAX_VELOCITY_DEG_S = 0.5
ARM_PD_DEADBAND_DEG = 1.0
ARM_TARGET_NOISE_DEG = 0.35
ARM_TARGET_NOISE_RATE_DEG_S = 12.0
ARM_PD_INTEGRAL_LIMIT_DEG_S = 45.0


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def angle_to_servo_value(angle_deg, servo_type="standard"):
    if servo_type == "standard":
        return clamp(angle_deg / 90.0, -1.0, 1.0)
    return clamp(angle_deg / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)


class AngleFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_deg):
        if self.value is None:
            self.value = new_deg
            return self.value
        diff = (new_deg - self.value + 180.0) % 360.0 - 180.0
        self.value += self.alpha * diff
        if self.value > 180.0:
            self.value -= 360.0
        if self.value <= -180.0:
            self.value += 360.0
        return self.value


def main(max_runtime_s=None, max_updates=None):
    print("Initializing IMU...")
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
    print(f"IMU initialized. Poll interval: {poll_interval * 1000:.1f}ms\n")

    roll_filter = AngleFilter(IMU_ALPHA)
    pitch_filter = AngleFilter(IMU_ALPHA)
    yaw_filter = AngleFilter(IMU_ALPHA)

    arm_tgt_last = 0.0
    lazy_tgt_last = 0.0
    head_tgt_last = 0.0
    arm_cmd_last = 0.0

    arm_damper = PIDCommandDamper(
        initial_angle=0.0,
        kp=ARM_PD_KP,
        ki=ARM_PD_KI,
        kd=ARM_PD_KD,
        max_velocity_deg_s=ARM_PD_MAX_VELOCITY_DEG_S,
        deadband_deg=ARM_PD_DEADBAND_DEG,
        target_noise_deg=ARM_TARGET_NOISE_DEG,
        target_noise_rate_deg_s=ARM_TARGET_NOISE_RATE_DEG_S,
        integral_limit_deg_s=ARM_PD_INTEGRAL_LIMIT_DEG_S,
    )

    try:
        print("Reading IMU. Press Ctrl+C to stop.\n")

        last_print = time.time()
        start_time = time.time()
        update_count = 0
        servo_update_interval = 0.1

        while True:
            now = time.time()
            if max_runtime_s is not None and now - start_time >= max_runtime_s:
                print(f"\nReached {max_runtime_s:.2f}s; stopping.")
                break

            if imu.IMURead():
                data = imu.getIMUData()
                fusion_pose = data["fusionPose"]
                roll = roll_filter.update(math.degrees(fusion_pose[0]))
                pitch = pitch_filter.update(math.degrees(fusion_pose[1]))
                yaw = yaw_filter.update(math.degrees(fusion_pose[2]))

                # Servo update at slower interval
                if now - last_print >= servo_update_interval:
                    dt = servo_update_interval

                    arm_tgt, lazy_tgt, head_tgt = find_motor_angles(
                        pitch, roll, DESIRED_ANGLE
                    )

                    if abs(arm_tgt - arm_tgt_last) < ARM_DEADBAND_DEG:
                        arm_tgt = arm_tgt_last
                    if abs(lazy_tgt - lazy_tgt_last) < CONT_DEADBAND_DEG:
                        lazy_tgt = lazy_tgt_last
                    if abs(head_tgt - head_tgt_last) < CONT_DEADBAND_DEG:
                        head_tgt = head_tgt_last

                    arm_tgt = clamp(arm_tgt, ARM_MIN, ARM_MAX)
                    lazy_tgt = clamp(lazy_tgt, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
                    head_tgt = clamp(head_tgt, HEAD_MIN, HEAD_MAX)

                    arm_tgt_last = arm_tgt
                    lazy_tgt_last = lazy_tgt
                    head_tgt_last = head_tgt

                    damped_arm_tgt = arm_damper.update(arm_tgt, dt)

                    arm_cmd = clamp(
                        angle_to_servo_value(damped_arm_tgt, "standard") + ARM_VERTICAL_OFFSET,
                        -1.0, 1.0
                    )

                    # Simulate lazy/head without encoder feedback (just for debug)
                    lazy_cmd = angle_to_servo_value(lazy_tgt, "continuous")
                    head_cmd = angle_to_servo_value(head_tgt, "continuous")

                    # Arm: step incrementally towards target
                    step_direction = 1.0 if arm_cmd > arm_cmd_last else (-1.0 if arm_cmd < arm_cmd_last else 0.0)
                    arm_cmd = arm_cmd_last + step_direction * ARM_STEP
                    arm_cmd = clamp(arm_cmd, -1.0, 1.0)
                    arm_cmd_last = arm_cmd

                    update_count += 1

                    print(f"IMU(filt): R={roll:7.2f}°  P={pitch:7.2f}°  Y={yaw:7.2f}°")
                    print(f"Tgt(deg): arm={arm_tgt:+7.2f}°  lazy={lazy_tgt:+7.2f}°  head={head_tgt:+7.2f}°")
                    print(f"Cmd(val): arm={arm_cmd:+.3f}  lazy={lazy_cmd:+.3f}  head={head_cmd:+.3f}")
                    print("-" * 70)

                    if max_updates is not None and update_count >= max_updates:
                        print(f"\nReached {max_updates} updates; stopping.")
                        break

                    last_print = now

            time.sleep(poll_interval)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        print("Done.")
    return 0


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Debug head balance (no servo movement).")
    parser.add_argument("--duration", type=float, default=None, help="Run for N seconds.")
    parser.add_argument("--updates", type=int, default=None, help="Stop after N updates.")
    args = parser.parse_args()

    sys.exit(main(max_runtime_s=args.duration, max_updates=args.updates))
