"""
IMU reader with smoothed servo control.

Reads IMU data, calculates target motor angles, and moves the servos to
balance. The arm servo uses open-loop PD command damping to reduce jitter near
the target angle.
"""

import math
import sys
import time

from gpiozero import DigitalOutputDevice, Servo  # type: ignore[import-not-found]
from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore[import-not-found]

from head_balance_math import find_motor_angles
from servo_control_filters import PIDCommandDamper

DEBUG = True
DESIRED_ANGLE = 0.0

# ============================================================================
# IMU INIT
# ============================================================================
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU  # type: ignore[import-not-found]  # noqa: E402

settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)
if not imu.IMUInit():
    if DEBUG:
        print("IMU init failed")
    sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
imu_poll_interval = imu.IMUGetPollInterval() / 1000.0

# ============================================================================
# SERVO INIT - use pigpio factory to reduce software-PWM jitter
# ============================================================================
# Requires: sudo systemctl enable --now pigpiod
try:
    pin_factory = PiGPIOFactory()
except Exception as exc:
    if DEBUG:
        print(
            f"pigpio not available ({exc}); falling back to default. "
            "Run `sudo systemctl start pigpiod` to reduce jitter."
        )
    pin_factory = None

arm_servo = Servo(13, initial_value=None, pin_factory=pin_factory)
lazy_susan_servo = Servo(12, initial_value=None, pin_factory=pin_factory)
head_servo = Servo(18, initial_value=None, pin_factory=pin_factory)
MOSFET = DigitalOutputDevice(16)

ARM_MIN, ARM_MAX = -120.0, 120.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0
HEAD_MIN, HEAD_MAX = -90.0, 90.0

# Lower alpha = smoother but more lag. Range is about 0.05 to 0.4.
IMU_ALPHA = 0.15

# Target deadband in degrees. This kills micro-buzz from tiny IMU changes.
ARM_DEADBAND_DEG = 1.0
CONT_DEADBAND_DEG = 2.0

# Slew limit on the commanded servo value, in -1..1 units per second.
ARM_SLEW_PER_SEC = 0.8
CONT_SLEW_PER_SEC = 0.6

# Continuous-servo speed scaling. Bigger denominator = gentler response.
CONT_MAX_ANGLE_SPEED = 15.0

# Arm jitter damping. This is open-loop because this script does not read the
# arm encoder; the controller damps the commanded angle near the setpoint.
ARM_PD_KP = 8.0
ARM_PD_KI = 0.0
ARM_PD_KD = 0.35
ARM_PD_MAX_VELOCITY_DEG_S = 160.0
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


def slew_limit(current, target, max_step):
    return current + clamp(target - current, -max_step, max_step)


class AngleFilter:
    """Exponential moving average with wrap-around handling for angles."""

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


def main():
    roll_filter = AngleFilter(IMU_ALPHA)
    pitch_filter = AngleFilter(IMU_ALPHA)
    yaw_filter = AngleFilter(IMU_ALPHA)

    arm_cmd_last = 0.0
    lazy_cmd_last = 0.0
    head_cmd_last = 0.0

    arm_tgt_last = 0.0
    lazy_tgt_last = 0.0
    head_tgt_last = 0.0

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
        if DEBUG:
            print("MOSFET on...")
        MOSFET.on()
        time.sleep(0.5)

        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        arm_damper.reset(0.0)
        time.sleep(1)

        last_print = time.time()
        last_servo_update = time.time()
        print_interval = 0.5
        servo_update_interval = 0.00625

        while True:
            if imu.IMURead():
                now = time.time()
                data = imu.getIMUData()
                fusion_pose = data["fusionPose"]
                roll = roll_filter.update(math.degrees(fusion_pose[0]))
                pitch = pitch_filter.update(math.degrees(fusion_pose[1]))
                yaw = yaw_filter.update(math.degrees(fusion_pose[2]))

                if now - last_servo_update >= servo_update_interval:
                    dt = now - last_servo_update
                    last_servo_update = now

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

                    arm_cmd = angle_to_servo_value(damped_arm_tgt, "standard")
                    lazy_cmd = angle_to_servo_value(lazy_tgt, "continuous")
                    head_cmd = angle_to_servo_value(head_tgt, "continuous")

                    arm_cmd = slew_limit(
                        arm_cmd_last, arm_cmd, ARM_SLEW_PER_SEC * dt
                    )
                    lazy_cmd = slew_limit(
                        lazy_cmd_last, lazy_cmd, CONT_SLEW_PER_SEC * dt
                    )
                    head_cmd = slew_limit(
                        head_cmd_last, head_cmd, CONT_SLEW_PER_SEC * dt
                    )

                    arm_servo.value = arm_cmd
                    lazy_susan_servo.value = lazy_cmd
                    head_servo.value = head_cmd

                    arm_cmd_last = arm_cmd
                    lazy_cmd_last = lazy_cmd
                    head_cmd_last = head_cmd

                if DEBUG and now - last_print >= print_interval:
                    print(
                        f"IMU(filt): R={roll:7.2f}  P={pitch:7.2f}  Y={yaw:7.2f}"
                    )
                    print(
                        f"Cmd: arm={arm_cmd_last:+.3f}  "
                        f"lazy={lazy_cmd_last:+.3f}  head={head_cmd_last:+.3f}"
                    )
                    print("-" * 70)
                    last_print = now

            time.sleep(imu_poll_interval)

    except KeyboardInterrupt:
        if DEBUG:
            print("\nStopping...")
    finally:
        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        time.sleep(0.5)
        MOSFET.off()
        if DEBUG:
            print("Done.")


if __name__ == "__main__":
    main()
