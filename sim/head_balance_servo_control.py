"""
IMU reader with smoothed servo control.

Reads IMU data, calculates target motor angles, and moves the 150kg arm and
lazy-susan servos to balance. The 505 head servo and its encoder are
intentionally ignored. The arm servo uses open-loop PID command damping to
reduce jitter near the target angle.
"""

import argparse
import math
import os
import select
import subprocess
import sys
import time

DEBUG = True
DESIRED_ANGLE = 0.0
NO_IMU_DATA_TIMEOUT_S = 5.0
IMU_INIT_TIMEOUT_S = 5.0

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

ARM_MIN, ARM_MAX = -30.0, 30.0
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90.0, 90.0
HEAD_MIN, HEAD_MAX = -float('inf'), float('inf')

ARM_VERTICAL_OFFSET = 0.0  # servo value at physical vertical
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5

LAZY_ENCODER_A = 4
LAZY_ENCODER_B = 22
LAZY_ENCODER_ABSOLUTE = 17
LAZY_CPR = 8192  # REV-11-1271 quadrature counts per revolution

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


def log(message):
    if DEBUG:
        print(message, flush=True)


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def angle_to_servo_value(angle_deg, servo_type="standard"):
    if servo_type == "standard":
        return clamp(angle_deg / 90.0, -1.0, 1.0)
    return clamp(angle_deg / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)


def slew_limit(current, target, max_step):
    return current + clamp(target - current, -max_step, max_step)


def normalize_degrees(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def capture_reference_pose(
    imu_device,
    poll_interval,
    timeout_s=NO_IMU_DATA_TIMEOUT_S,
):
    log("Capturing flat reference from the first valid IMU sample...")
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        try:
            got_data = imu_device.IMURead()
        except Exception as exc:
            log(f"IMU read error while capturing flat reference: {exc}")
            got_data = False

        if got_data:
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

    log(
        f"No IMU sample received within {timeout_s:.1f}s. "
        "Run `python3 sim/imu_diagnostic.py` to inspect the Sense HAT and I2C connection."
    )
    return None


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


class StreamedImu:
    def __init__(self):
        stream_script = os.path.join(SCRIPT_DIR, "imu_stream.py")
        self.process = subprocess.Popen(
            [sys.executable, "-u", stream_script],
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.data = None

    def IMURead(self):
        if self.process.poll() is not None:
            raise RuntimeError("IMU stream process stopped")

        readable, _, _ = select.select([self.process.stdout], [], [], 0)
        if not readable:
            return False

        line = self.process.stdout.readline().strip()
        if not line:
            return False

        roll, pitch, yaw = (float(value) for value in line.split(","))
        self.data = {"fusionPose": (roll, pitch, yaw)}
        return True

    def getIMUData(self):
        return self.data

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def initialize_imu(init_timeout_s=IMU_INIT_TIMEOUT_S):
    log("Starting isolated RTIMU stream...")
    return StreamedImu(), 0.003


def wait_for_first_imu_sample(imu_device, poll_interval):
    log("Waiting for the first valid IMU sample...")
    deadline = time.time() + NO_IMU_DATA_TIMEOUT_S
    while time.time() < deadline:
        if imu_device.IMURead():
            log("First IMU sample received.")
            return True
        time.sleep(poll_interval)

    log(f"No IMU data for {NO_IMU_DATA_TIMEOUT_S:.1f}s; stopping.")
    return False


def sleep_while_polling_imu(imu_device, poll_interval, duration_s):
    deadline = time.time() + duration_s
    while time.time() < deadline:
        imu_device.IMURead()
        time.sleep(poll_interval)


def initialize_servos():
    from gpiozero import DigitalOutputDevice, Servo  # type: ignore[import-not-found]
    from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore[import-not-found]
    from ServoEx import ServoEx  # type: ignore[import-not-found]

    # Requires: sudo systemctl enable --now pigpiod
    log("Initializing servos...")
    try:
        pin_factory = PiGPIOFactory()
        log("Using pigpio hardware-timed PWM.")
    except Exception as exc:
        log(
            f"pigpio not available ({exc}); falling back to default. "
            "Run `sudo systemctl start pigpiod` to reduce jitter."
        )
        pin_factory = None

    servos = (
        Servo(15, initial_value=None, pin_factory=pin_factory),
        ServoEx(
            12,
            LAZY_ENCODER_A,
            LAZY_ENCODER_B,
            LAZY_ENCODER_ABSOLUTE,
            initial_value=None,
            pin_factory=pin_factory,
        ),
        None,
        DigitalOutputDevice(16),
    )
    log("Arm and lazy-susan servos initialized. 505 head servo is disabled.")
    return servos


def run_servo_self_test(arm_servo, lazy_susan_servo, head_servo, mosfet):
    log("Running arm and lazy-susan servo self-test...")
    mosfet.on()
    time.sleep(0.5)
    for value in (0.0, 0.15, -0.15, 0.0):
        log(f"Self-test servo command: {value:+.2f}")
        arm_servo.value = value
        lazy_susan_servo.value = value
        time.sleep(0.7)
    arm_servo.value = 0
    lazy_susan_servo.value = 0
    time.sleep(0.3)
    mosfet.off()
    log("Servo self-test complete.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Balance servos from IMU pitch/roll data."
    )
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
    log("Starting head balance servo control.")

    if servo_self_test:
        arm_servo, lazy_susan_servo, head_servo, mosfet = initialize_servos()
        run_servo_self_test(arm_servo, lazy_susan_servo, head_servo, mosfet)
        return 0

    imu, imu_poll_interval = initialize_imu(imu_init_timeout_s)
    if imu is None:
        log("Exiting because IMU did not initialize.")
        return 1

    if not wait_for_first_imu_sample(imu, imu_poll_interval):
        imu.close()
        return 1

    from head_balance_math import find_motor_angles
    from servo_control_filters import PIDCommandDamper

    arm_servo, lazy_susan_servo, head_servo, mosfet = initialize_servos()

    log("Skipping flat-reference capture; using raw IMU roll and pitch.")
    roll_reference_deg = 0.0
    pitch_reference_deg = 0.0

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
        log("MOSFET on...")
        mosfet.on()
        sleep_while_polling_imu(imu, imu_poll_interval, 0.5)

        log("Centering servos...")
        arm_servo.value = clamp(ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)
        lazy_susan_servo.value = 0
        arm_damper.reset(0.0)
        sleep_while_polling_imu(imu, imu_poll_interval, 1.0)
        log("Entering balance loop. Press Ctrl+C to stop.")

        last_print = time.time()
        last_servo_update = time.time()
        last_imu_data = time.time()
        start_time = time.time()
        servo_update_count = 0
        print_interval = 0.5
        servo_update_interval = 0.00625

        running = True
        while running:
            now = time.time()
            if max_runtime_s is not None and now - start_time >= max_runtime_s:
                log(f"Reached --duration={max_runtime_s:.2f}s; stopping.")
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
                raw_roll = math.degrees(fusion_pose[0])
                raw_pitch = math.degrees(fusion_pose[1])
                roll, pitch = apply_counterbalance(
                    raw_roll,
                    raw_pitch,
                    roll_reference_deg,
                    pitch_reference_deg,
                )
                roll = roll_filter.update(roll)
                pitch = pitch_filter.update(pitch)
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

                    arm_cmd = clamp(
                        angle_to_servo_value(damped_arm_tgt, "standard") + ARM_VERTICAL_OFFSET,
                        ARM_SERVO_MIN, ARM_SERVO_MAX
                    )

                    lazy_susan_servo.update()
                    lazy_actual = lazy_susan_servo.encoder.steps * 360.0 / LAZY_CPR
                    lazy_cmd = angle_to_servo_value(lazy_tgt - lazy_actual, "continuous")

                    # The 505 head servo and encoder are deliberately ignored.
                    # Keep its calculated target in the debug output only.
                    head_cmd = 0.0

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

                    arm_cmd_last = arm_cmd
                    lazy_cmd_last = lazy_cmd
                    head_cmd_last = head_cmd
                    servo_update_count += 1

                    if (
                        max_servo_updates is not None
                        and servo_update_count >= max_servo_updates
                    ):
                        log(f"Reached --updates={max_servo_updates}; stopping.")
                        running = False

                if DEBUG and now - last_print >= print_interval:
                    print(
                        f"IMU(raw):  R={raw_roll:7.2f}  P={raw_pitch:7.2f}"
                    )
                    print(
                        f"IMU(adj):  R={roll:7.2f}  P={pitch:7.2f}  Y={yaw:7.2f}"
                    )
                    print(
                        f"Cmd: arm={arm_cmd_last:+.3f}  "
                        f"lazy={lazy_cmd_last:+.3f}  head(disabled)={head_tgt_last:+.2f}°"
                    )
                    print("-" * 70)
                    last_print = now
            elif now - last_imu_data >= NO_IMU_DATA_TIMEOUT_S:
                log(f"No IMU data for {NO_IMU_DATA_TIMEOUT_S:.1f}s; stopping.")
                break

            time.sleep(imu_poll_interval)

    except KeyboardInterrupt:
        log("\nStopping...")
    finally:
        log("Centering servos and shutting down.")
        arm_servo.value = 0
        lazy_susan_servo.deactivate_and_save()
        time.sleep(0.5)
        mosfet.off()
        imu.close()
        log("Done. Servo positions saved.")
    return 0


if __name__ == "__main__":
    args = parse_args()
    sys.exit(
        main(
            max_runtime_s=args.duration,
            max_servo_updates=args.updates,
            imu_init_timeout_s=args.imu_init_timeout,
            servo_self_test=args.servo_self_test,
        )
    )
