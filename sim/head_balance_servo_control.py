"""
IMU Reader with Servo Control
Reads IMU data, calculates target motor angles, and closes the position loop
on the two continuous-rotation servos using their encoders.

Servo / Encoder layout (BCM numbering):
  Arm (servo 0, 150kg, standard positional):  servo=BCM 13                 (pin 33)
  Lazy Susan (servo 1, 70kg, continuous):     servo=BCM 12                 (pin 32)
      encoder 1: A=BCM 27 (pin 13), B=BCM 22 (pin 15), ABS=BCM 17 (pin 11)
  Head (servo 2, 5kg, continuous):            servo=BCM 18                 (pin 12)
      encoder 0: A=BCM 26 (pin 37), B=BCM 6  (pin 31), ABS=BCM 5  (pin 29)
  MOSFET: BCM 16 (pin 36)
"""

import sys
import time
import math
from gpiozero import Servo, DigitalOutputDevice
from head_balance_math import find_motor_angles
from ServoEx import ServoEx

# ============================================================================
# DEBUG / SETPOINT
# ============================================================================
DEBUG = True
DESIRED_ANGLE = 0.0

# ============================================================================
# IMU INITIALIZATION
# ============================================================================
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU

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
if DEBUG:
    print(f"IMU initialized. Poll interval: {imu_poll_interval*1000:.1f}ms\n")

# ============================================================================
# PINS (BCM)
# ============================================================================
ARM_SERVO_PIN = 13
MOSFET_PIN    = 16

# Head (encoder 0)
HEAD_SERVO_PIN = 18
HEAD_ENC_A     = 13
HEAD_ENC_B     = 15
HEAD_ENC_ABS   = 11

# Lazy Susan (encoder 1)
LAZY_SERVO_PIN = 12
LAZY_ENC_A     = 27
LAZY_ENC_B     = 22
LAZY_ENC_ABS   = 17

# ============================================================================
# HARDWARE INIT
# ============================================================================
# Arm: plain positional servo
arm_servo = Servo(ARM_SERVO_PIN, initial_value=None)
MOSFET = DigitalOutputDevice(MOSFET_PIN)

# Power servos before talking to them so ServoEx's is_active waits succeed
if DEBUG:
    print("Turning MOSFET ON to power servos for init...")
MOSFET.on()
time.sleep(0.5)

# ServoEx instances bundle servo + quadrature encoder + absolute encoder
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

# ============================================================================
# LIMITS & TUNING
# ============================================================================
# Mechanical limits (degrees, target side)
ARM_MIN, ARM_MAX = -120, 120
LAZY_MIN, LAZY_MAX = -90, 90
HEAD_MIN, HEAD_MAX = -90, 90

# Arm: ±SERVO_RANGE_DEG maps to ±1.0 on gpiozero Servo value.
ARM_RANGE_DEG = 90.0

# --- Closed-loop tuning ----------------------------------------------------
# P-gain: servo value per degree of error. 1/30 → 30° error commands full speed.
LAZY_KP = 1.0 / 30.0
HEAD_KP = 1.0 / 30.0

# Deadband: no command if |error| under this. Kills jitter at rest.
LAZY_DEADBAND_DEG = 1.0
HEAD_DEADBAND_DEG = 1.0

# Min drive to overcome stiction (0 = disabled). Bump to 0.05–0.15 if motor
# "wants to move but doesn't" with small commands.
MIN_DRIVE = 0.0

# Cap commanded speed (1.0 = full).
MAX_DRIVE = 1.0

# Slew rate on the velocity command (units per second). Prevents abrupt
# speed changes when the IMU jerks.
VEL_SLEW_RATE = 4.0

# Sign convention: flip if positive command moves the encoder negative.
# Find empirically by commanding a small positive value and watching the angle.
LAZY_DIR = +1
HEAD_DIR = +1

# Calibration offset: encoder-zero angle vs. mechanical-zero angle (degrees).
# Set so that the resting/centered physical position reads as 0°.
LAZY_OFFSET_DEG = 0.0
HEAD_OFFSET_DEG = 0.0

# --- Rates ----------------------------------------------------------------
CONTROL_RATE_HZ = 50.0
CONTROL_INTERVAL = 1.0 / CONTROL_RATE_HZ
PRINT_INTERVAL = 0.5

# ============================================================================
# HELPERS
# ============================================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_to_arm_value(angle_deg):
    """Standard positional servo: angle → [-1, 1]."""
    return clamp(angle_deg / ARM_RANGE_DEG, -1.0, 1.0)


def get_motor_angle_deg(motor, offset_deg):
    """ServoEx position is in rotations; convert to degrees and apply offset."""
    return motor.get_position() * 360.0 - offset_deg


def position_controller(target_deg, current_deg, kp, deadband_deg,
                        min_drive, max_drive,
                        prev_cmd, max_delta, direction):
    """
    P controller for a continuous-rotation servo with position feedback.
    Returns a new servo value in [-1, 1], slew-limited from prev_cmd.
    """
    error = target_deg - current_deg

    if abs(error) < deadband_deg:
        raw_cmd = 0.0
    else:
        raw_cmd = direction * kp * error
        raw_cmd = clamp(raw_cmd, -max_drive, max_drive)
        if min_drive > 0 and 0 < abs(raw_cmd) < min_drive:
            raw_cmd = math.copysign(min_drive, raw_cmd)

    # Slew-limit the velocity command itself
    diff = raw_cmd - prev_cmd
    if diff >  max_delta: return prev_cmd + max_delta
    if diff < -max_delta: return prev_cmd - max_delta
    return raw_cmd


# ============================================================================
# MAIN LOOP
# ============================================================================
def main():
    lazy_cmd = 0.0
    head_cmd = 0.0

    try:
        if DEBUG:
            print("Centering arm; stopping continuous servos...")
        arm_servo.value = 0
        head_motor.value = 0   # 0 = stop for continuous
        lazy_motor.value = 0
        time.sleep(1)

        if DEBUG:
            print("Running. Press Ctrl+C to stop.\n")

        last_print = time.time()
        last_control = time.time()

        while True:
            # ServoEx.update() must run every loop:
            #   - drives the absolute-encoder PWM decode (edge detection)
            #   - periodically self-centers the relative encoder
            try:
                head_motor.update()
                lazy_motor.update()
            except Exception as e:
                if DEBUG:
                    print(f"Encoder update error: {e}")

            try:
                got_data = imu.IMURead()
            except Exception as e:
                if DEBUG:
                    print(f"IMU read error: {e}")
                got_data = False

            current_time = time.time()

            if got_data:
                data = imu.getIMUData()
                fusionPose = data["fusionPose"]
                roll  = math.degrees(fusionPose[0])
                pitch = math.degrees(fusionPose[1])
                yaw   = math.degrees(fusionPose[2])

                arm_target, lazy_target, head_target = find_motor_angles(
                    pitch, roll, DESIRED_ANGLE
                )
                arm_target  = clamp(arm_target,  ARM_MIN,  ARM_MAX)
                lazy_target = clamp(lazy_target, LAZY_MIN, LAZY_MAX)
                head_target = clamp(head_target, HEAD_MIN, HEAD_MAX)

                # --- Control step ----------------------------------------
                dt = current_time - last_control
                if dt >= CONTROL_INTERVAL:
                    lazy_current = get_motor_angle_deg(lazy_motor, LAZY_OFFSET_DEG)
                    head_current = get_motor_angle_deg(head_motor, HEAD_OFFSET_DEG)
                    max_delta = VEL_SLEW_RATE * dt

                    lazy_cmd = position_controller(
                        lazy_target, lazy_current,
                        LAZY_KP, LAZY_DEADBAND_DEG,
                        MIN_DRIVE, MAX_DRIVE,
                        lazy_cmd, max_delta, LAZY_DIR,
                    )
                    head_cmd = position_controller(
                        head_target, head_current,
                        HEAD_KP, HEAD_DEADBAND_DEG,
                        MIN_DRIVE, MAX_DRIVE,
                        head_cmd, max_delta, HEAD_DIR,
                    )

                    lazy_motor.value = lazy_cmd
                    head_motor.value = head_cmd
                    arm_servo.value = angle_to_arm_value(arm_target)

                    last_control = current_time

                # --- Logging --------------------------------------------
                if current_time - last_print >= PRINT_INTERVAL:
                    if DEBUG:
                        lc = get_motor_angle_deg(lazy_motor, LAZY_OFFSET_DEG)
                        hc = get_motor_angle_deg(head_motor, HEAD_OFFSET_DEG)
                        print(f"IMU: Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°")
                        print(f"Targets: Arm={arm_target:7.2f}°  Lazy={lazy_target:7.2f}°  Head={head_target:7.2f}°")
                        print(f"Encoders: Lazy={lc:7.2f}°  Head={hc:7.2f}°  | Cmd: Lazy={lazy_cmd:+.3f}  Head={head_cmd:+.3f}")
                        print("-" * 80)
                    last_print = current_time

            time.sleep(0.001)

    except KeyboardInterrupt:
        if DEBUG:
            print("\n\nStopping...")

    finally:
        if DEBUG:
            print("Stopping motors / centering arm...")
        try:
            arm_servo.value = 0
            head_motor.value = 0
            lazy_motor.value = 0
            time.sleep(0.5)
        except Exception as e:
            if DEBUG:
                print(f"Stop error: {e}")

        # Persist encoder positions so we resume in the right place next run
        try:
            head_motor.save_encoder_position()
            lazy_motor.save_encoder_position()
            if DEBUG:
                print("Encoder positions saved.")
        except Exception as e:
            if DEBUG:
                print(f"Save error: {e}")

        if DEBUG:
            print("Turning MOSFET OFF...")
        MOSFET.off()
        if DEBUG:
            print("Done!")


if __name__ == "__main__":
    main()