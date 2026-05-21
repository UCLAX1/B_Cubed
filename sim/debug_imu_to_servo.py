"""
Debug script: Read IMU, calculate target angles, output servo commands WITHOUT moving servos.
"""

import math
import signal
import sys
import time

from head_balance_math import find_motor_angles

DEBUG = True
DESIRED_ANGLE = 0.0
NO_IMU_DATA_TIMEOUT_S = 5.0
IMU_INIT_TIMEOUT_S = 5.0

SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")

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


def log(message):
    if DEBUG:
        print(message, flush=True)


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def angle_to_servo_value(angle_deg, servo_type="standard"):
    if servo_type == "standard":
        return clamp(angle_deg / 90.0, -1.0, 1.0)
    return clamp(angle_deg / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)


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
    import RTIMU

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


def main(max_runtime_s=None, max_updates=None):
    log("Starting IMU→servo debug (NO MOVEMENT).")

    imu, imu_poll_interval = initialize_imu(IMU_INIT_TIMEOUT_S)
    if imu is None:
        log("Exiting because IMU did not initialize.")
        return 1

    roll_filter = AngleFilter(IMU_ALPHA)
    pitch_filter = AngleFilter(IMU_ALPHA)
    yaw_filter = AngleFilter(IMU_ALPHA)

    arm_tgt_last = 0.0
    lazy_tgt_last = 0.0
    head_tgt_last = 0.0

    try:
        log("Reading IMU. Press Ctrl+C to stop.\n")

        last_print = time.time()
        last_imu_data = time.time()
        start_time = time.time()
        update_count = 0

        running = True
        while running:
            now = time.time()
            if max_runtime_s is not None and now - start_time >= max_runtime_s:
                log(f"Reached {max_runtime_s:.2f}s; stopping.")
                break

            try:
                got_data = imu.IMURead()
            except Exception as exc:
                log(f"IMU read error: {exc}")
                got_data = False

            if got_data:
                now = time.time()
                last_imu_data = now
                data = imu.getIMUData()
                fusion_pose = data["fusionPose"]
                roll = roll_filter.update(math.degrees(fusion_pose[0]))
                pitch = pitch_filter.update(math.degrees(fusion_pose[1]))
                yaw = yaw_filter.update(math.degrees(fusion_pose[2]))

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

                arm_cmd = clamp(
                    angle_to_servo_value(arm_tgt, "standard") + ARM_VERTICAL_OFFSET,
                    -1.0, 1.0
                )
                lazy_cmd = angle_to_servo_value(lazy_tgt, "continuous")
                head_cmd = angle_to_servo_value(head_tgt, "continuous")

                update_count += 1

                if DEBUG and now - last_print >= 0.5:
                    print(
                        f"IMU(filt): R={roll:7.2f}°  P={pitch:7.2f}°  Y={yaw:7.2f}°"
                    )
                    print(
                        f"Tgt(deg): arm={arm_tgt:+7.2f}°  lazy={lazy_tgt:+7.2f}°  head={head_tgt:+7.2f}°"
                    )
                    print(
                        f"Cmd(val): arm={arm_cmd:+.3f}  lazy={lazy_cmd:+.3f}  head={head_cmd:+.3f}"
                    )
                    print("-" * 70)
                    last_print = now

                if max_updates is not None and update_count >= max_updates:
                    log(f"Reached {max_updates} updates; stopping.")
                    break

            elif now - last_imu_data >= NO_IMU_DATA_TIMEOUT_S:
                log(f"No IMU data for {NO_IMU_DATA_TIMEOUT_S:.1f}s; stopping.")
                break

            time.sleep(imu_poll_interval)

    except KeyboardInterrupt:
        log("\nStopping...")
    finally:
        log("Done.")
    return 0


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Debug IMU→servo mapping (no movement).")
    parser.add_argument("--duration", type=float, default=None, help="Run for N seconds.")
    parser.add_argument("--updates", type=int, default=None, help="Stop after N updates.")
    args = parser.parse_args()

    sys.exit(main(max_runtime_s=args.duration, max_updates=args.updates))
