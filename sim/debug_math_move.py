"""Same math as debug_math_output.py, but actually moves the arm and lazy susan."""

import math
import sys
import time

from gpiozero import DigitalOutputDevice, RotaryEncoder, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

DESIRED_ANGLE = 0.0
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")

ARM_MIN, ARM_MAX = -30.0, 30.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0
ARM_SERVO_MIN, ARM_SERVO_MAX = -0.5, 0.5
ARM_VERTICAL_OFFSET = 0.0

LAZY_CPR = 2048
CONT_DEADBAND_DEG = 4.0
CONT_MAX_SPEED = 0.5
ENCODER_STALL_TIMEOUT_S = 1.0
LAZY_COMMAND_SIGN = -1.0


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def clamp_strict(value, minimum, maximum):
    return clamp(value, minimum + 1e-6, maximum - 1e-6)


def wrap_degrees(angle_deg):
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def fold_lazy(angle_deg):
    angle_deg = wrap_degrees(angle_deg)
    if angle_deg > LAZY_SUSAN_MAX:
        angle_deg -= 180.0
    elif angle_deg < LAZY_SUSAN_MIN:
        angle_deg += 180.0
    return angle_deg


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
            fp = imu.getIMUData()["fusionPose"]
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

    factory = PiGPIOFactory()
    mosfet = DigitalOutputDevice(16, pin_factory=factory)
    arm_servo = Servo(15, initial_value=None, pin_factory=factory)
    lazy_servo = Servo(12, initial_value=None, pin_factory=factory)
    lazy_encoder = RotaryEncoder(a=26, b=6, max_steps=10_000_000, pin_factory=factory)

    mosfet.on()
    time.sleep(0.3)
    arm_servo.value = 0.0
    lazy_servo.value = 0.0
    time.sleep(0.3)

    lazy_stalled_since = None
    lazy_last_steps = None

    print("\nMoving servos. Ctrl-C to stop.\n")

    try:
        while True:
            if not imu.IMURead():
                time.sleep(poll_interval)
                continue

            fp = imu.getIMUData()["fusionPose"]
            raw_roll  = math.degrees(fp[0])
            raw_pitch = math.degrees(fp[1])

            adj_roll  = wrap_degrees(raw_roll  - roll_ref)
            adj_pitch = wrap_degrees(raw_pitch - pitch_ref)

            arm_tgt, lazy_tgt = find_motor_angles(adj_pitch, adj_roll, DESIRED_ANGLE)
            arm_tgt  = clamp(arm_tgt,  ARM_MIN, ARM_MAX)
            lazy_tgt = clamp_strict(lazy_tgt, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)

            arm_cmd = clamp_strict(
                (arm_tgt / 90.0) + ARM_VERTICAL_OFFSET,
                ARM_SERVO_MIN, ARM_SERVO_MAX,
            )

            lazy_actual_raw = lazy_encoder.steps * 360.0 / LAZY_CPR
            lazy_actual = fold_lazy(lazy_actual_raw)
            lazy_error = wrap_degrees(lazy_tgt - lazy_actual)

            if abs(lazy_error) < CONT_DEADBAND_DEG:
                lazy_cmd = 0.0
            else:
                lazy_cmd = LAZY_COMMAND_SIGN * clamp(lazy_error / 45.0, -1.0, 1.0) * CONT_MAX_SPEED

            # Stall detection: zero command if encoder steps don't change
            now = time.monotonic()
            steps_now = lazy_encoder.steps
            if lazy_last_steps is not None and abs(lazy_cmd) > 0.1:
                if steps_now == lazy_last_steps:
                    if lazy_stalled_since is None:
                        lazy_stalled_since = now
                    elif now - lazy_stalled_since >= ENCODER_STALL_TIMEOUT_S:
                        lazy_cmd = 0.0
                else:
                    lazy_stalled_since = None
            else:
                lazy_stalled_since = None
            lazy_last_steps = steps_now

            arm_servo.value  = arm_cmd
            lazy_servo.value = lazy_cmd

            print(
                f"adjR={adj_roll:+5.1f}° adjP={adj_pitch:+5.1f}°  "
                f"arm_tgt={arm_tgt:+6.1f}° arm_cmd={arm_cmd:+.3f}  "
                f"lazy_tgt={lazy_tgt:+6.1f}° lazy_pos={lazy_actual:+6.1f}° lazy_cmd={lazy_cmd:+.3f}",
                flush=True,
            )
            time.sleep(poll_interval)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        arm_servo.value  = 0.0
        lazy_servo.value = 0.0
        time.sleep(0.3)
        mosfet.off()


if __name__ == "__main__":
    main()
