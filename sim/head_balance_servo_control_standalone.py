"""Standalone balance controller with lazy susan feedback only.

Copy this file onto the Raspberry Pi as-is. It inlines:
- the motor angle math from head_balance_math.py
- the encoder-backed servo helper from ServoEx.py

This version ignores head movement entirely. It keeps the arm and lazy susan
control path only, because the lazy susan has encoder feedback and the head
encoder is not available.
"""

import json
import math
import os
import sys
import time

from gpiozero import DigitalOutputDevice, RotaryEncoder, Servo
from gpiozero.pins.pigpio import PiGPIOFactory


DEBUG = True
DESIRED_ANGLE = 0.0
ARM_SERVO_ENABLED = True

ARM_MIN, ARM_MAX = -30.0, 30.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0

ARM_VERTICAL_OFFSET = 0.0
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5
LAZY_CPR = 2048
CONT_MAX_ANGLE_SPEED = 0.5
CONT_DEADBAND_DEG = 4.0
ENCODER_STALL_TIMEOUT_S = 1.0
ENCODER_STALL_MIN_MOVE_DEG = 0.5
LAZY_COMMAND_SIGN = -1.0
STRICT_BOUND_EPSILON = 1e-6


def continuous_servo_command(error_deg):
    abs_error = abs(error_deg)

    if abs_error < 4.0:
        return 0.0
    elif abs_error < 10.0:
        speed_scale = 120.0
    elif abs_error < 25.0:
        speed_scale = 60.0
    elif abs_error < 60.0:
        speed_scale = 30.0
    else:
        speed_scale = 20.0

    return clamp(error_deg / speed_scale, -1.0, 1.0) * CONT_MAX_ANGLE_SPEED


def lazy_servo_command(error_deg, actual_deg):
    command = LAZY_COMMAND_SIGN * continuous_servo_command(error_deg)

    distance_to_limit = LAZY_SUSAN_MAX - abs(actual_deg)
    if distance_to_limit <= 0.0:
        if actual_deg > 0.0 and command < 0.0:
            return 0.0
        if actual_deg < 0.0 and command > 0.0:
            return 0.0
        return command

    if distance_to_limit < 10.0:
        command *= clamp(distance_to_limit / 10.0, 0.0, 1.0)

    return command

SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def clamp_strict(value, minimum, maximum):
    if minimum >= maximum:
        return minimum
    return clamp(value, minimum + STRICT_BOUND_EPSILON, maximum - STRICT_BOUND_EPSILON)


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


def mod_360(angle_deg):
    return angle_deg % 360.0


def shortest_angle_error(target_deg, actual_deg):
    return wrap_degrees(target_deg - actual_deg)


def arm_servo_command(target_deg):
    return clamp_strict((target_deg / 90.0) + ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)


def capture_reference_pose(imu_device, poll_interval):
    while True:
        if imu_device.IMURead():
            data = imu_device.getIMUData()
            fusion_pose = data["fusionPose"]
            roll_reference_deg = math.degrees(fusion_pose[0])
            pitch_reference_deg = math.degrees(fusion_pose[1])
            print(
                f"Flat reference: R={roll_reference_deg:+.1f}°  "
                f"P={pitch_reference_deg:+.1f}°"
            )
            return roll_reference_deg, pitch_reference_deg
        time.sleep(poll_interval)


def apply_counterbalance(roll_deg, pitch_deg, roll_reference_deg, pitch_reference_deg):
    return (
        wrap_degrees(roll_deg - roll_reference_deg),
        wrap_degrees(pitch_deg - pitch_reference_deg),
    )


def find_motor_angles(pitch, roll, desired_angle):
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    desired_angle = math.radians(desired_angle)
    lazy_susan = 0.0
    arm = 0.0

    n1 = math.sin(pitch) * math.cos(roll)
    n2 = -math.sin(roll)
    n3 = math.cos(pitch) * math.cos(roll)

    arm = -math.degrees(math.acos(n3))

    if pitch < 0:
        arm = -arm

    if roll != 0:
        lazy_susan = math.degrees(math.acos(n1 / math.sqrt(n1**2 + n2**2)))

    if roll < 0:
        lazy_susan = -lazy_susan
        arm = -arm

    if abs(lazy_susan) > 90:
        arm = -arm
        if lazy_susan > 0:
            lazy_susan -= 180
        else:
            lazy_susan += 180

    return arm, lazy_susan


class AbsoluteEncoder:
    POSITION_HISTORY_MAX_SIZE = 100

    def __init__(self, pin):
        self.pin = pin
        self.position = 0.0
        self.__input_device = None
        self.__time_activated = 0.0
        self.__previous_value = 0
        self.__current_value = 0
        self.__position_history = []

        try:
            from gpiozero import DigitalInputDevice

            self.__input_device = DigitalInputDevice(pin)
        except Exception:
            self.__input_device = None

    def update(self):
        if self.__input_device is None:
            return

        self.__current_value = self.__input_device.value

        if self.__current_value == 1 and self.__previous_value == 0:
            self.__time_activated = time.time()

        if self.__current_value == 0 and self.__previous_value == 1:
            dt = time.time() - self.__time_activated
            new_position = clamp(dt * 1000.0, 0.0, 1.0)

            if len(self.__position_history) >= self.POSITION_HISTORY_MAX_SIZE:
                self.__position_history = self.__position_history[1:]

            self.__position_history.append(new_position)
            self.position = sum(self.__position_history) / len(self.__position_history)

        self.__previous_value = self.__current_value


class ServoEx(Servo):
    INIT_POS_FILE = "servo_init_pos.json"
    COUNTS_PER_REVOLUTION = LAZY_CPR
    POSITION_CENTERING_DEADZONE_ERROR = 0.08
    POSITION_CENTERING_DELAY = 0.25

    def __init__(
        self,
        servo_pin,
        encoder_pin_a,
        encoder_pin_b,
        absolute_encoder_pin,
        initial_value=None,
        pin_factory=None,
        save_on_deactivate=False,
    ):
        try:
            super().__init__(servo_pin, initial_value=initial_value, pin_factory=pin_factory)
            self.encoder = RotaryEncoder(a=encoder_pin_a, b=encoder_pin_b, max_steps=10000000000000, pin_factory=pin_factory)
        except Exception:
            print("ERROR: gpiozero servo could not initialize. Make sure the servos are plugged in to the right pins.")
            raise

        self.absolute_encoder = AbsoluteEncoder(pin=absolute_encoder_pin)
        self.pin = servo_pin
        self.time_position_last_centered = 0.0
        self.save_on_deactivate = save_on_deactivate
        self._last_value = None

        self.__wait_for_active(encoder_pin_a, encoder_pin_b)

        if not os.path.exists(self.INIT_POS_FILE):
            self.save_encoder_position()

        self.load_encoder_position()

    def __wait_for_active(self, encoder_pin_a, encoder_pin_b):
        print(f"servo {self.pin} ready, encoder a={encoder_pin_a} b={encoder_pin_b} abs={self.absolute_encoder.pin} ready")

    def deactivate_and_save(self):
        self.value = None
        self.save_encoder_position()

    def get_position(self):
        return self.encoder.steps / self.COUNTS_PER_REVOLUTION

    def get_position_radians(self):
        return (self.encoder.steps / self.COUNTS_PER_REVOLUTION) * 2.0 * math.pi

    def get_absolute_position(self):
        return self.absolute_encoder.position

    def get_absolute_position_radians(self):
        return self.absolute_encoder.position * 2.0 * math.pi

    def update(self):
        self.update_absolute_encoder()

        if time.time() - self.time_position_last_centered > self.POSITION_CENTERING_DELAY:
            self.center_position_with_absolute_encoder()
            self.time_position_last_centered = time.time()

    def update_absolute_encoder(self):
        self.absolute_encoder.update()

    def center_position_with_absolute_encoder(self):
        if self.get_absolute_position() > (1.0 - self.POSITION_CENTERING_DEADZONE_ERROR) or self.get_absolute_position() < self.POSITION_CENTERING_DEADZONE_ERROR:
            return

        mod_position = self.get_position() % 1.0

        if mod_position > (1.0 - self.POSITION_CENTERING_DEADZONE_ERROR) or mod_position < self.POSITION_CENTERING_DEADZONE_ERROR:
            return

        position_difference = self.get_absolute_position() - mod_position
        self.encoder.steps += position_difference * self.COUNTS_PER_REVOLUTION

    def save_encoder_position(self):
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
        data[str(self.pin)] = self.encoder.steps
        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)

    def load_encoder_position(self):
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
                self.encoder.steps = data.get(str(self.pin), 0)

    def reset_encoder_position(self):
        self.encoder.steps = 0
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
        data[str(self.pin)] = 0
        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)


def main():
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

    roll_reference_deg, pitch_reference_deg = capture_reference_pose(imu, poll_interval)

    factory = PiGPIOFactory()
    mosfet = DigitalOutputDevice(16, pin_factory=factory)
    mosfet.on()
    time.sleep(0.5)

    arm_servo = Servo(15, initial_value=None, pin_factory=factory) if ARM_SERVO_ENABLED else None
    lazy_susan = ServoEx(12, 4, 22, 17, initial_value=None, pin_factory=factory)

    lazy_last_actual = None
    lazy_stalled_since = None

    try:
        if arm_servo is not None:
            arm_servo.value = clamp_strict(ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)
        lazy_susan.value = 0.0
        time.sleep(0.5)

        while True:
            if imu.IMURead():
                data = imu.getIMUData()
                fusion_pose = data["fusionPose"]

                raw_roll = math.degrees(fusion_pose[0])
                raw_pitch = math.degrees(fusion_pose[1])
                roll, pitch = apply_counterbalance(
                    raw_roll,
                    raw_pitch,
                    roll_reference_deg,
                    pitch_reference_deg,
                )
                yaw = math.degrees(fusion_pose[2])

                arm_tgt, lazy_tgt = find_motor_angles(pitch, roll, DESIRED_ANGLE)

                arm_tgt = clamp(arm_tgt, ARM_MIN, ARM_MAX)
                lazy_tgt = clamp_strict(lazy_tgt, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)

                lazy_susan.update()

                lazy_actual_raw = lazy_susan.encoder.steps * 360.0 / LAZY_CPR
                lazy_actual = fold_lazy_susan_degrees(lazy_actual_raw)
                lazy_error = shortest_angle_error(lazy_tgt, lazy_actual)
                if abs(lazy_error) < CONT_DEADBAND_DEG:
                    lazy_cmd = 0.0
                else:
                    lazy_cmd = clamp(lazy_servo_command(lazy_error, lazy_actual), -1.0, 1.0)

                now = time.time()
                if lazy_last_actual is not None and abs(wrap_degrees(lazy_actual - lazy_last_actual)) < ENCODER_STALL_MIN_MOVE_DEG and abs(lazy_cmd) > 0.2:
                    if lazy_stalled_since is None:
                        lazy_stalled_since = now
                    elif now - lazy_stalled_since >= ENCODER_STALL_TIMEOUT_S:
                        lazy_cmd = 0.0
                        log_message = f"Warning: lazy encoder not moving; stopping motor. actual={lazy_actual:+.2f}° target={lazy_tgt:+.2f}°"
                        print(log_message)
                else:
                    lazy_stalled_since = None

                lazy_last_actual = lazy_actual

                arm_cmd = arm_servo_command(arm_tgt)

                lazy_susan.value = lazy_cmd

                if arm_servo is not None:
                    arm_servo.value = arm_cmd

                if DEBUG:
                    print(f"IMU(raw): R={raw_roll:7.1f}° P={raw_pitch:7.1f}°")
                    print(f"IMU(adj): R={roll:7.1f}° P={pitch:7.1f}° Y={yaw:7.1f}°")
                    print(f"Tgt: arm={arm_tgt:+7.2f}° lazy={lazy_tgt:+7.2f}°")
                    print(f"Pos: lazy={lazy_actual:+7.2f}°")
                    print(f"Err: lazy={lazy_error:+7.2f}°")
                    print(f"Cmd: arm={arm_cmd:+.3f} lazy={lazy_cmd:+.3f}")
                    print("-" * 70)

            time.sleep(poll_interval)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if arm_servo is not None:
            arm_servo.value = 0.0
        lazy_susan.value = 0.0
        lazy_susan.deactivate_and_save()
        time.sleep(0.5)
        mosfet.off()


if __name__ == "__main__":
    main()
