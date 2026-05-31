import sys
import time
import math
import json
import os

from head_balance_math import find_motor_angles
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

DEBUG = True
DESIRED_ANGLE = 0.0
ARM_SERVO_ENABLED = False

ARM_MIN, ARM_MAX = -30.0, 30.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0
HEAD_MIN, HEAD_MAX = -float('inf'), float('inf')

ARM_VERTICAL_OFFSET = 0.0
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5
LAZY_CPR = 1493
HEAD_CPR = 2048
CONT_MAX_ANGLE_SPEED = 0.5
STARTUP_HOME_FILE = "servo_home_absolute.json"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STARTUP_HOME_PATH = os.path.join(SCRIPT_DIR, STARTUP_HOME_FILE)
STARTUP_HOME_DEADBAND_DEG = 3.0
STARTUP_HOME_MAX_COMMAND = 0.25
STARTUP_HOME_SPEED_SCALE = 45.0
STARTUP_HOME_TIMEOUT_S = 10.0

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

factory = PiGPIOFactory()
mosfet = DigitalOutputDevice(16)
mosfet.on()
time.sleep(0.5)

arm_servo = Servo(15, initial_value=None, pin_factory=factory) if ARM_SERVO_ENABLED else None
lazy_susan = ServoEx(12, 26, 6, 5, initial_value=None, pin_factory=factory)
head_servo = ServoEx(20, 4, 22, 17, initial_value=None, pin_factory=factory)

def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def wrap_degrees(angle_deg):
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def fold_lazy_susan_degrees(angle_deg):
    folded = wrap_degrees(angle_deg)

    if folded > LAZY_SUSAN_MAX:
        folded -= 180.0
    elif folded < LAZY_SUSAN_MIN:
        folded += 180.0

    return folded


def shortest_angle_error(target_deg, actual_deg):
    return wrap_degrees(target_deg - actual_deg)


def load_startup_home_targets():
    if not os.path.exists(STARTUP_HOME_PATH):
        return None

    with open(STARTUP_HOME_PATH, "r") as file_handle:
        return json.load(file_handle)


def home_continuous_servo_to_absolute(servo, target_deg, name):
    deadline = time.time() + STARTUP_HOME_TIMEOUT_S
    servo.value = 0.0

    print(f"Homing {name} to saved absolute position {target_deg:+.1f}°...")
    while time.time() < deadline:
        servo.update()
        actual_deg = servo.get_absolute_position() * 360.0
        error_deg = shortest_angle_error(target_deg, actual_deg)

        if abs(error_deg) <= STARTUP_HOME_DEADBAND_DEG:
            servo.value = 0.0
            print(f"  {name} homed at {actual_deg:+.1f}° (error {error_deg:+.1f}°)")
            return True

        servo.value = clamp(
            error_deg / STARTUP_HOME_SPEED_SCALE,
            -STARTUP_HOME_MAX_COMMAND,
            STARTUP_HOME_MAX_COMMAND,
        )
        time.sleep(poll_interval)

    servo.value = 0.0
    print(f"  WARNING: {name} homing timed out at {actual_deg:+.1f}° (target {target_deg:+.1f}°)")
    return False

try:
    home_targets = load_startup_home_targets()
    if home_targets is None:
        print(f"No saved absolute home file found at {STARTUP_HOME_PATH}; skipping startup homing.")
    else:
        if home_targets.get("lazy_abs_valid", True):
            home_continuous_servo_to_absolute(
                lazy_susan,
                float(home_targets.get("lazy_susan_deg", 0.0)),
                "lazy susan",
            )
        else:
            print("Skipping lazy susan homing because the absolute encoder reading was invalid.")

        if home_targets.get("head_abs_valid", True):
            home_continuous_servo_to_absolute(
                head_servo,
                float(home_targets.get("head_deg", 0.0)),
                "head",
            )
        else:
            print("Skipping head homing because the absolute encoder reading was invalid.")

    if arm_servo is not None:
        arm_servo.value = clamp(ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)
    lazy_susan.value = 0.0
    head_servo.value = 0.0
    time.sleep(0.5)

    while True:
        if imu.IMURead():
            data = imu.getIMUData()
            fusion_pose = data["fusionPose"]

            r2d = math.degrees
            roll = r2d(fusion_pose[0])
            pitch = r2d(fusion_pose[1])
            yaw = r2d(fusion_pose[2])

            arm_tgt, lazy_tgt, head_tgt = find_motor_angles(pitch, roll, DESIRED_ANGLE)

            arm_tgt = clamp(arm_tgt, ARM_MIN, ARM_MAX)
            lazy_tgt = clamp(lazy_tgt, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
            head_tgt = clamp(head_tgt, HEAD_MIN, HEAD_MAX)

            lazy_susan.update()
            head_servo.update()

            lazy_actual_raw = lazy_susan.get_position() * 360.0
            lazy_actual = fold_lazy_susan_degrees(lazy_actual_raw)
            lazy_error = shortest_angle_error(lazy_tgt, lazy_actual)
            lazy_cmd = clamp(lazy_error / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)

            head_actual_raw = head_servo.get_position() * 360.0
            head_actual = wrap_degrees(head_actual_raw)
            head_error = shortest_angle_error(head_tgt, head_actual)
            head_cmd = clamp(head_error / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)

            arm_cmd = clamp(
                (arm_tgt / 90.0) + ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX
            )

            lazy_susan.value = lazy_cmd
            head_servo.value = head_cmd
            arm_servo.value = arm_cmd

            if DEBUG:
                print(f"IMU: R={roll:7.1f}° P={pitch:7.1f}° Y={yaw:7.1f}°")
                print(f"Tgt: arm={arm_tgt:+7.2f}° lazy={lazy_tgt:+7.2f}° head={head_tgt:+7.2f}°")
                print(f"Head attempt: {head_tgt:+7.2f}°")
                print(f"Pos: lazy={lazy_actual:+7.2f}° head={head_actual:+7.2f}°")
                print(f"Err: lazy={lazy_error:+7.2f}° head={head_error:+7.2f}°")
                print(f"Cmd: arm={arm_cmd:+.3f} lazy={lazy_cmd:+.3f} head={head_cmd:+.3f}")
                print("-" * 70)

        time.sleep(poll_interval)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    if arm_servo is not None:
        arm_servo.value = 0.0
    lazy_susan.value = 0.0
    head_servo.value = 0.0
    lazy_susan.deactivate_and_save()
    head_servo.deactivate_and_save()
    time.sleep(0.5)
    mosfet.off()
