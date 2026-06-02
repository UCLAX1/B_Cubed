"""
Open-loop head balance controller.

This version does not read encoders. It assumes the robot starts in a known
vertical pose, then keeps an internal estimate of where the servos have moved
based on the commands it sends.

That makes it useful when the encoder wiring is not trustworthy, or when you
want a simpler controller that only remembers its own commanded motion.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

from head_balance_math import find_motor_angles


DEBUG = True
DESIRED_ANGLE = 0.0
IMU_INIT_TIMEOUT_S = 5.0
NO_IMU_DATA_TIMEOUT_S = 5.0

SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")

ARM_MIN, ARM_MAX = -30.0, 30.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0
HEAD_MIN, HEAD_MAX = -180.0, 180.0

ARM_VERTICAL_OFFSET = 0.0
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5

# Continuous servo response tuning.
LAZY_MAX_SPEED_DEG_S = 60.0
HEAD_MAX_SPEED_DEG_S = 60.0
LAZY_COMMAND_SCALE = 45.0
HEAD_COMMAND_SCALE = 45.0

ARM_SLEW_PER_SEC = 0.8


def log(message):
    if DEBUG:
        print(message, flush=True)


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def normalize_degrees(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def wrap_to_lazy_range(angle_deg):
    angle_deg = normalize_degrees(angle_deg)
    if angle_deg > LAZY_SUSAN_MAX:
        angle_deg -= 180.0
    elif angle_deg < LAZY_SUSAN_MIN:
        angle_deg += 180.0
    return angle_deg


def capture_reference_pose(imu_device, poll_interval):
    log("Capturing flat reference from the first valid IMU sample...")

    while True:
        if imu_device.IMURead():
            data = imu_device.getIMUData()
            fusion_pose = data["fusionPose"]
            roll_reference_deg = math.degrees(fusion_pose[0])
            pitch_reference_deg = math.degrees(fusion_pose[1])
            log(
                f"Flat reference: R={roll_reference_deg:+.1f}°  "
                f"P={pitch_reference_deg:+.1f}°"
            )
            return roll_reference_deg, pitch_reference_deg
        time.sleep(poll_interval)


def apply_counterbalance(roll_deg, pitch_deg, roll_reference_deg, pitch_reference_deg):
    adjusted_roll = normalize_degrees(roll_deg - roll_reference_deg)
    adjusted_pitch = normalize_degrees(pitch_deg - pitch_reference_deg)
    return adjusted_roll, adjusted_pitch


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


class ImuInitTimeout(TimeoutError):
    pass


def _raise_imu_init_timeout(signum, frame):
    raise ImuInitTimeout("IMUInit timed out")


def imu_init_with_timeout(imu_device, timeout_s):
    if timeout_s is None or timeout_s <= 0:
        return imu_device.IMUInit()

    if not hasattr(signal, "SIGALRM") or not hasattr(signal, "setitimer"):
        log("IMU init timeout is not supported on this platform.")
        return imu_device.IMUInit()

    old_handler = signal.getsignal(signal.SIGALRM)
    signal.signal(signal.SIGALRM, _raise_imu_init_timeout)
    signal.setitimer(signal.ITIMER_REAL, timeout_s)
    try:
        return imu_device.IMUInit()
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_handler)


def initialize_imu(init_timeout_s=IMU_INIT_TIMEOUT_S):
    log("Loading RTIMU...")
    import RTIMU  # type: ignore[import-not-found]  # noqa: E402

    log(f"Creating RTIMU settings from {SETTINGS_FILE}...")
    settings = RTIMU.Settings(SETTINGS_FILE)
    imu_device = RTIMU.RTIMU(settings)

    log("Initializing IMU...")
    try:
        initialized = imu_init_with_timeout(imu_device, init_timeout_s)
    except ImuInitTimeout:
        log(f"IMU init timed out after {init_timeout_s:.1f}s")
        return None, None

    if not initialized:
        log("IMU init failed")
        return None, None

    log("IMU initialized.")
    imu_device.setSlerpPower(0.02)
    imu_device.setGyroEnable(True)
    imu_device.setAccelEnable(True)
    imu_device.setCompassEnable(True)
    poll_interval = imu_device.IMUGetPollInterval() / 1000.0
    log(f"IMU poll interval: {poll_interval * 1000:.1f}ms")
    return imu_device, poll_interval


def initialize_servos():
    log("Initializing servos...")
    pin_factory = PiGPIOFactory()
    log("Using pigpio hardware-timed PWM.")

    arm_servo = Servo(15, initial_value=None, pin_factory=pin_factory)
    lazy_susan_servo = Servo(12, initial_value=None, pin_factory=pin_factory)
    head_servo = Servo(20, initial_value=None, pin_factory=pin_factory)
    mosfet = DigitalOutputDevice(16, pin_factory=pin_factory)

    log("Servos initialized.")
    return arm_servo, lazy_susan_servo, head_servo, mosfet


def run_servo_self_test(arm_servo, lazy_susan_servo, head_servo, mosfet):
    log("Running servo self-test...")
    mosfet.on()
    time.sleep(0.5)
    for value in (0.0, 0.15, -0.15, 0.0):
        log(f"Self-test servo command: {value:+.2f}")
        arm_servo.value = value
        lazy_susan_servo.value = value
        head_servo.value = value
        time.sleep(0.7)
    arm_servo.value = 0
    lazy_susan_servo.value = 0
    head_servo.value = 0
    time.sleep(0.3)
    mosfet.off()
    log("Servo self-test complete.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Open-loop balance control without encoders.")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Run for this many seconds, then center servos and exit.",
    )
    parser.add_argument(
        "--updates",
        type=int,
        default=None,
        help="Stop after this many servo updates. Useful for IDE/debug runs.",
    )
    parser.add_argument(
        "--imu-init-timeout",
        type=float,
        default=IMU_INIT_TIMEOUT_S,
        help="Seconds to wait for IMUInit before exiting. Use 0 to disable.",
    )
    parser.add_argument(
        "--servo-self-test",
        action="store_true",
        help="Move each servo briefly, then exit without starting IMU balance.",
    )
    return parser.parse_args()


def main(
    max_runtime_s=None,
    max_servo_updates=None,
    imu_init_timeout_s=IMU_INIT_TIMEOUT_S,
    servo_self_test=False,
):
    log("Starting open-loop head balance servo control.")

    arm_servo, lazy_susan_servo, head_servo, mosfet = initialize_servos()
    if servo_self_test:
        run_servo_self_test(arm_servo, lazy_susan_servo, head_servo, mosfet)
        return 0

    imu, imu_poll_interval = initialize_imu(imu_init_timeout_s)
    if imu is None:
        log("Exiting because IMU did not initialize.")
        return 1

    roll_reference_deg, pitch_reference_deg = capture_reference_pose(imu, imu_poll_interval)

    mosfet.on()
    time.sleep(0.5)

    arm_servo.value = clamp(ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)
    lazy_susan_servo.value = 0.0
    head_servo.value = 0.0

    # Open-loop estimates: we assume we start at vertical and then integrate
    # the motion we command.
    arm_estimated_deg = 0.0
    lazy_estimated_deg = 0.0
    head_estimated_deg = 0.0

    last_update_time = time.monotonic()
    update_count = 0
    start_time = time.monotonic()

    try:
        while True:
            if max_runtime_s is not None and (time.monotonic() - start_time) >= max_runtime_s:
                log("Reached requested runtime limit.")
                break

            if max_servo_updates is not None and update_count >= max_servo_updates:
                log("Reached requested update limit.")
                break

            if not imu.IMURead():
                time.sleep(imu_poll_interval)
                continue

            now = time.monotonic()
            dt = max(now - last_update_time, imu_poll_interval)
            last_update_time = now

            data = imu.getIMUData()
            fusion_pose = data["fusionPose"]

            r2d = math.degrees
            roll = r2d(fusion_pose[0])
            pitch = r2d(fusion_pose[1])
            yaw = r2d(fusion_pose[2])

            adj_roll, adj_pitch = apply_counterbalance(
                roll, pitch, roll_reference_deg, pitch_reference_deg
            )

            arm_tgt, lazy_tgt, head_tgt = find_motor_angles(adj_pitch, adj_roll, DESIRED_ANGLE)

            arm_tgt = clamp(arm_tgt, ARM_MIN, ARM_MAX)
            lazy_tgt = clamp(lazy_tgt, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
            head_tgt = clamp(head_tgt, HEAD_MIN, HEAD_MAX)

            lazy_error = lazy_tgt - lazy_estimated_deg
            head_error = head_tgt - head_estimated_deg

            lazy_cmd = clamp(lazy_error / LAZY_COMMAND_SCALE, -1.0, 1.0)
            head_cmd = clamp(head_error / HEAD_COMMAND_SCALE, -1.0, 1.0)

            arm_cmd = clamp((arm_tgt / 90.0) + ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)

            arm_servo.value = arm_cmd
            lazy_susan_servo.value = lazy_cmd
            head_servo.value = head_cmd

            arm_estimated_deg += arm_cmd * 90.0 * dt * ARM_SLEW_PER_SEC
            lazy_estimated_deg = wrap_to_lazy_range(
                lazy_estimated_deg + lazy_cmd * LAZY_MAX_SPEED_DEG_S * dt
            )
            head_estimated_deg = normalize_degrees(
                head_estimated_deg + head_cmd * HEAD_MAX_SPEED_DEG_S * dt
            )

            update_count += 1

            if DEBUG:
                print(f"IMU: R={roll:7.1f}° P={pitch:7.1f}° Y={yaw:7.1f}°")
                print(
                    f"Ref: roll0={roll_reference_deg:+.1f}° pitch0={pitch_reference_deg:+.1f}°"
                )
                print(
                    f"Adj: R={adj_roll:+7.1f}° P={adj_pitch:+7.1f}°  "
                    f"est: arm={arm_estimated_deg:+7.1f}° "
                    f"lazy={lazy_estimated_deg:+7.1f}° head={head_estimated_deg:+7.1f}°"
                )
                print(
                    f"Tgt: arm={arm_tgt:+7.2f}° lazy={lazy_tgt:+7.2f}° head={head_tgt:+7.2f}°"
                )
                print(
                    f"Err: lazy={lazy_error:+7.2f}° head={head_error:+7.2f}°"
                )
                print(
                    f"Cmd: arm={arm_cmd:+.3f} lazy={lazy_cmd:+.3f} head={head_cmd:+.3f}"
                )
                print("-" * 70)

            time.sleep(imu_poll_interval)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        arm_servo.value = 0.0
        lazy_susan_servo.value = 0.0
        head_servo.value = 0.0
        time.sleep(0.5)
        mosfet.off()

    return 0


if __name__ == "__main__":
    args = parse_args()
    raise SystemExit(
        main(
            max_runtime_s=args.duration,
            max_servo_updates=args.updates,
            imu_init_timeout_s=args.imu_init_timeout,
            servo_self_test=args.servo_self_test,
        )
    )