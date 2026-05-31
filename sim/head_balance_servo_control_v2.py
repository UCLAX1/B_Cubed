import sys
import time
import math

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

ARM_VERTICAL_OFFSET = -0.66
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5
LAZY_CPR = 1493
HEAD_CPR = 2048
CONT_MAX_ANGLE_SPEED = 0.5

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

arm_servo = Servo(13, initial_value=None, pin_factory=factory) if ARM_SERVO_ENABLED else None
lazy_susan = ServoEx(12, 26, 6, 5, initial_value=None, pin_factory=factory)
head_servo = ServoEx(20, 4, 22, 17, initial_value=None)

def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)

try:
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

            lazy_actual = lazy_susan.encoder.steps * 360.0 / LAZY_CPR
            lazy_error = lazy_tgt - lazy_actual
            lazy_cmd = clamp(lazy_error / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)

            head_actual = head_servo.encoder.steps * 360.0 / HEAD_CPR
            head_error = head_tgt - head_actual
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
