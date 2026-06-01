"""
Open-loop arm balance controller.

This version controls only the 150kg arm servo (BCM 15). It does not read
encoders — it assumes the robot starts vertical and integrates the arm motion
based on the commands it sends.
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

SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")

ARM_MIN, ARM_MAX = -30.0, 30.0
ARM_VERTICAL_OFFSET = 0.0
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5

# Arm tuning
ARM_SLEW_PER_SEC = 0.8


def log(message):
    if DEBUG:
        print(message, flush=True)


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


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


def initialize_servo():
    log("Initializing arm servo...")
    pin_factory = PiGPIOFactory()
    arm_servo = Servo(15, initial_value=None, pin_factory=pin_factory)
    mosfet = DigitalOutputDevice(16, pin_factory=pin_factory)
    log("Arm servo initialized.")
    return arm_servo, mosfet


def parse_args():
    parser = argparse.ArgumentParser(description="Open-loop arm-only balance.")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Run for this many seconds, then center servo and exit.",
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
        "--assume-vertical",
        action="store_true",
        help="Assume the robot starts vertical and skip waiting for an IMU reference sample.",
    )
    return parser.parse_args()


def main(max_runtime_s=None, max_servo_updates=None, imu_init_timeout_s=IMU_INIT_TIMEOUT_S, assume_vertical: bool = False):
    log("Starting open-loop arm-only controller.")

    arm_servo, mosfet = initialize_servo()

    imu, imu_poll_interval = initialize_imu(imu_init_timeout_s)
    if assume_vertical:
        log("Assuming vertical startup; skipping IMU reference capture.")
        roll_ref, pitch_ref = 0.0, 0.0
        # If IMU isn't available, continue open-loop anyway.
        if imu is None:
            imu_poll_interval = 0.02
    else:
        if imu is None:
            log("Exiting because IMU did not initialize.")
            return 1

        roll_ref, pitch_ref = capture_reference_pose(imu, imu_poll_interval)

    mosfet.on()
    time.sleep(0.5)

    # Start centered
    arm_servo.value = clamp(ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)

    arm_estimated_deg = 0.0
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
            roll = math.degrees(fusion_pose[0])
            pitch = math.degrees(fusion_pose[1])

            adj_roll = (roll - roll_ref + 180.0) % 360.0 - 180.0
            adj_pitch = (pitch - pitch_ref + 180.0) % 360.0 - 180.0

            arm_tgt, _, _ = find_motor_angles(adj_pitch, adj_roll, DESIRED_ANGLE)
            arm_tgt = clamp(arm_tgt, ARM_MIN, ARM_MAX)

            arm_cmd = clamp((arm_tgt / 90.0) + ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)
            arm_servo.value = arm_cmd

            arm_estimated_deg += arm_cmd * 90.0 * dt * ARM_SLEW_PER_SEC

            update_count += 1

            if DEBUG:
                print(f"IMU: R={roll:7.1f}° P={pitch:7.1f}°")
                print(f"Ref: R0={roll_ref:+.1f}° P0={pitch_ref:+.1f}°")
                print(f"Tgt: arm={arm_tgt:+6.2f}° est={arm_estimated_deg:+6.2f}° cmd={arm_cmd:+.3f}")
                print("-" * 60)

            time.sleep(imu_poll_interval)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        arm_servo.value = 0.0
        time.sleep(0.5)
        mosfet.off()

    return 0


if __name__ == "__main__":
    args = parse_args()
    raise SystemExit(
        main(max_runtime_s=args.duration, max_servo_updates=args.updates, imu_init_timeout_s=args.imu_init_timeout)
    )
