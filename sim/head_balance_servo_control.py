"""
IMU Reader with Servo Control
Reads IMU data, calculates target motor angles, and moves servos to balance.
Uses gpiozero Servo for direct control (matches servo_sample.py).

All three servos are treated as standard (positional) servos.

Pin numbers below are BCM GPIO (the default for gpiozero), matching the
wiring table:
  - servo 0 (15kg, arm)        -> physical 33 / BCM 13
  - servo 1 (70kg, lazy susan) -> physical 32 / BCM 12
  - servo 2 (5kg,  head)       -> physical 12 / BCM 18
  - MOSFET                     -> physical 36 / BCM 16
  - encoder 0 (head):    A=pin 37/BCM 26, B=pin 31/BCM 6, ABS=pin 29/BCM 5
  - encoder 1 (lazy):    A=pin 13/BCM 27, B=pin 15/BCM 22, ABS=pin 11/BCM 17
"""

import sys
import time
import math
from gpiozero import Servo, DigitalOutputDevice
from head_balance_math import find_motor_angles

# ============================================================================
# DEBUG FLAG - Set to True to see print output, False to suppress
# ============================================================================
DEBUG = True  # Change to False to silence all output

DESIRED_ANGLE = 0.0

# ============================================================================
# IMU INITIALIZATION (egg.py style)
# ============================================================================
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU
from ServoEx import ServoEx

settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)

if not imu.IMUInit():
    if DEBUG:
        print("IMU init failed")
    sys.exit(1)

imu.setSlerpPower(
    0.02
)  # NOTE: raise to 0.05-0.1 if you see slow drift while stationary
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

imu_poll_interval = imu.IMUGetPollInterval() / 1000.0

if DEBUG:
    print(f"IMU initialized. Poll interval: {imu_poll_interval*1000:.1f}ms\n")

# ============================================================================
# PIN ASSIGNMENTS (BCM GPIO numbers)
# ============================================================================
# Servos
ARM_SERVO_PIN  = 13   # servo 0 (15kg)  - physical pin 33
LAZY_SERVO_PIN = 12   # servo 1 (70kg)  - physical pin 32
HEAD_SERVO_PIN = 18   # servo 2 (5kg)   - physical pin 12

# Power switch for servo rail
MOSFET_PIN     = 16   # physical pin 36

# Head encoder (encoder 0)
HEAD_ENC_A     = 26   # physical pin 37
HEAD_ENC_B     = 6    # physical pin 31
HEAD_ENC_ABS   = 5    # physical pin 29

# Lazy Susan encoder (encoder 1)
LAZY_ENC_A     = 27   # physical pin 13
LAZY_ENC_B     = 22   # physical pin 15
LAZY_ENC_ABS   = 17   # physical pin 11

# ============================================================================
# HARDWARE INIT
# ============================================================================
arm_servo        = Servo(ARM_SERVO_PIN,  initial_value=None)
lazy_susan_servo = Servo(LAZY_SERVO_PIN, initial_value=None)
head_servo       = Servo(HEAD_SERVO_PIN, initial_value=None)
MOSFET = DigitalOutputDevice(MOSFET_PIN)

# Power servos before talking to them so ServoEx's is_active waits succeed
if DEBUG:
    print("Turning MOSFET ON to power servos for init...")
MOSFET.on()
time.sleep(0.5)

# ServoEx instances bundle servo + quadrature encoder + absolute encoder.
# These reuse the same servo pins as the bare Servo objects above; that is
# intentional if ServoEx is read-only on the servo, but if it tries to drive
# the pin you'll want to choose one or the other.
head_motor = ServoEx(
    servo_pin=HEAD_SERVO_PIN,
    encoder_pin_a=HEAD_ENC_A,
    encoder_pin_b=HEAD_ENC_B,
    absolute_encoder_pin=HEAD_ENC_ABS,
)
lazy_motor = ServoEx(
    servo_pin=LAZY_SERVO_PIN,
    encoder_pin_a=LAZY_ENC_A,
    encoder_pin_b=LAZY_ENC_B,
    absolute_encoder_pin=LAZY_ENC_ABS,
)

# Angle limits (degrees) to prevent mechanical damage
ARM_MIN, ARM_MAX = -120, 120
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90, 90
HEAD_MIN, HEAD_MAX = -90, 90

# Servo range: assume ±90° maps to ±1.0 on the gpiozero Servo value.
# If your servos have a wider/narrower range, adjust SERVO_RANGE_DEG accordingly.
SERVO_RANGE_DEG = 90.0

# Slew-rate limit: max change in servo value per second (-1..1 range).
# 2.0 = full sweep in 1 second. Lower = smoother but slower to respond.
SERVO_SLEW_RATE = 2.0


def angle_to_servo_value(angle_deg):
    """Convert a target angle in degrees to a servo value in [-1, 1]."""
    return max(min(angle_deg / SERVO_RANGE_DEG, 1.0), -1.0)


def slew_limit(current, target, max_delta):
    """Move current toward target by at most max_delta."""
    diff = target - current
    if diff > max_delta:
        return current + max_delta
    if diff < -max_delta:
        return current - max_delta
    return target


# ============================================================================
# MAIN LOOP
# ============================================================================


def main():
    # Track last commanded servo values for slew limiting
    arm_cmd = 0.0
    lazy_susan_cmd = 0.0
    head_cmd = 0.0

    try:
        if DEBUG:
            print("Turning MOSFET ON...")
        MOSFET.on()
        time.sleep(0.5)

        if DEBUG:
            print("Centering all servos...")
        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        time.sleep(1)

        if DEBUG:
            print("Reading IMU data and moving servos. Press Ctrl+C to stop.\n")

        last_print = time.time()
        last_servo_update = time.time()
        print_interval = 0.5  # Print every 500ms
        servo_update_interval = 0.05  # 20Hz servo updates; smoothness from slew limit

        while True:
            try:
                got_data = imu.IMURead()
            except Exception as e:
                if DEBUG:
                    print(f"IMU read error: {e}")
                got_data = False

            if got_data:
                current_time = time.time()

                data = imu.getIMUData()
                fusionPose = data["fusionPose"]

                roll = math.degrees(fusionPose[0])
                pitch = math.degrees(fusionPose[1])
                yaw = math.degrees(fusionPose[2])

                # Compute absolute target angles for all three positional servos
                arm_target, lazy_susan_target, head_target = find_motor_angles(
                    pitch, roll, DESIRED_ANGLE
                )

                # Clamp to safe mechanical limits
                arm_target = max(min(arm_target, ARM_MAX), ARM_MIN)
                lazy_susan_target = max(
                    min(lazy_susan_target, LAZY_SUSAN_MAX), LAZY_SUSAN_MIN
                )
                head_target = max(min(head_target, HEAD_MAX), HEAD_MIN)

                if current_time - last_print >= print_interval:
                    if DEBUG:
                        print(
                            f"IMU: Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°"
                        )
                        print(
                            f"Targets: Arm={arm_target:7.2f}°  Lazy Susan={lazy_susan_target:7.2f}°  Head={head_target:7.2f}°"
                        )
                        print("-" * 80)
                    last_print = current_time

                # Update servos at controlled rate with slew limiting
                dt = current_time - last_servo_update
                if dt >= servo_update_interval:
                    arm_raw = angle_to_servo_value(arm_target)
                    lazy_raw = angle_to_servo_value(lazy_susan_target)
                    head_raw = angle_to_servo_value(head_target)

                    max_delta = SERVO_SLEW_RATE * dt
                    arm_cmd = slew_limit(arm_cmd, arm_raw, max_delta)
                    lazy_susan_cmd = slew_limit(lazy_susan_cmd, lazy_raw, max_delta)
                    head_cmd = slew_limit(head_cmd, head_raw, max_delta)

                    arm_servo.value = arm_cmd
                    lazy_susan_servo.value = lazy_susan_cmd
                    head_servo.value = head_cmd

                    last_servo_update = current_time

            # Short sleep — IMURead() returns False until fresh data is ready,
            # so this just keeps the loop from pegging the CPU.
            time.sleep(0.001)

    except KeyboardInterrupt:
        if DEBUG:
            print("\n\nStopping...")

    finally:
        if DEBUG:
            print("Centering servos...")
        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        time.sleep(0.5)

        if DEBUG:
            print("Turning MOSFET OFF...")
        MOSFET.off()
        if DEBUG:
            print("Done!")


if __name__ == "__main__":
    main()