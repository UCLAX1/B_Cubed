#!/usr/bin/env python3
"""
servo_test_no_imu.py — Test servos WITHOUT IMU / SenseHat.
Moves each servo through its safe range one at a time.
Use this to verify wiring and servo direction before enabling IMU control.

Usage:
  python servo_test_no_imu.py                  # real hardware
  python servo_test_no_imu.py --simulated      # dry run (print only)
  python servo_test_no_imu.py --servo neck     # test only neck tilt
  python servo_test_no_imu.py --servo susan    # test only lazy susan
  python servo_test_no_imu.py --servo twist    # test only head twist
  python servo_test_no_imu.py --interactive    # keyboard control
"""

import sys
import time
import math
import signal
import argparse
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
log = logging.getLogger('servo_test')

# Import from the controller module (same directory)
from head_balance_controller import (
    ServoDriver, ContinuousServoDriver, SimulatedServo,
    NECK_TILT_MIN, NECK_TILT_MAX, NECK_TILT_CENTER,
    LAZY_SUSAN_MIN, LAZY_SUSAN_MAX,
    NECK_TILT_PIN, LAZY_SUSAN_PIN, HEAD_TWIST_PIN,
    clamp,
)


def test_neck_tilt(servo, duration=6.0):
    """Slowly sweep neck tilt through full safe range."""
    log.info(f"--- Neck tilt test: {NECK_TILT_MIN}° to {NECK_TILT_MAX}° ---")
    log.info("Centering...")
    physical_center = NECK_TILT_CENTER
    servo.set_angle(physical_center)
    time.sleep(1.0)

    steps = int(duration * 20)  # 20 Hz
    for i in range(steps):
        # Sinusoidal sweep: 0 → +30 → 0 → -30 → 0
        t = i / steps
        angle = NECK_TILT_MAX * math.sin(2 * math.pi * t)
        physical = NECK_TILT_CENTER + angle
        servo.set_angle(physical)
        log.info(f"  tilt={angle:+6.1f}° (physical={physical:5.1f}°)")
        time.sleep(1.0 / 20)

    log.info("Returning to center...")
    servo.set_angle(physical_center)
    time.sleep(0.5)
    log.info("Neck tilt test DONE")


def test_lazy_susan(servo, duration=6.0):
    """Slowly rotate lazy susan in both directions."""
    log.info("--- Lazy susan test: slow CW then CCW ---")
    log.info("Stopping...")
    servo.stop()
    time.sleep(0.5)

    # Slow forward for half duration
    log.info("Rotating CW (speed=0.3) ...")
    t0 = time.time()
    while time.time() - t0 < duration / 2:
        servo.set_speed(0.3)
        time.sleep(0.05)

    # Slow reverse for half duration
    log.info("Rotating CCW (speed=-0.3) ...")
    t0 = time.time()
    while time.time() - t0 < duration / 2:
        servo.set_speed(-0.3)
        time.sleep(0.05)

    servo.stop()
    log.info("Lazy susan test DONE")


def test_head_twist(servo, duration=4.0):
    """Slowly rotate head twist servo."""
    log.info("--- Head twist test: slow CW then CCW ---")
    servo.stop()
    time.sleep(0.5)

    log.info("Rotating CW (speed=0.3) ...")
    t0 = time.time()
    while time.time() - t0 < duration / 2:
        servo.set_speed(0.3)
        time.sleep(0.05)

    log.info("Rotating CCW (speed=-0.3) ...")
    t0 = time.time()
    while time.time() - t0 < duration / 2:
        servo.set_speed(-0.3)
        time.sleep(0.05)

    servo.stop()
    log.info("Head twist test DONE")


def interactive_mode(neck, susan, twist):
    """Keyboard control: WASD + QE for manual servo testing."""
    log.info("=== INTERACTIVE MODE ===")
    log.info("Controls:")
    log.info("  W/S  — neck tilt up/down")
    log.info("  A/D  — lazy susan left/right")
    log.info("  Q/E  — head twist left/right")
    log.info("  0    — center all")
    log.info("  X    — stop & exit")
    log.info("Press keys then Enter (or use single-char input):")

    tilt_pos = 0.0
    susan_speed = 0.0
    twist_speed = 0.0
    step = 2.0  # degrees per keypress for tilt

    try:
        while True:
            try:
                cmd = input("> ").strip().lower()
            except EOFError:
                break

            if not cmd:
                continue

            for ch in cmd:
                if ch == 'w':
                    tilt_pos = clamp(tilt_pos + step, NECK_TILT_MIN, NECK_TILT_MAX)
                elif ch == 's':
                    tilt_pos = clamp(tilt_pos - step, NECK_TILT_MIN, NECK_TILT_MAX)
                elif ch == 'a':
                    susan_speed = clamp(susan_speed - 0.1, -1.0, 1.0)
                elif ch == 'd':
                    susan_speed = clamp(susan_speed + 0.1, -1.0, 1.0)
                elif ch == 'q':
                    twist_speed = clamp(twist_speed - 0.1, -1.0, 1.0)
                elif ch == 'e':
                    twist_speed = clamp(twist_speed + 0.1, -1.0, 1.0)
                elif ch == '0':
                    tilt_pos = 0.0
                    susan_speed = 0.0
                    twist_speed = 0.0
                elif ch == 'x':
                    raise KeyboardInterrupt

            neck.set_angle(NECK_TILT_CENTER + tilt_pos)
            susan.set_speed(susan_speed)
            twist.set_speed(twist_speed)
            log.info(f"tilt={tilt_pos:+5.1f}° susan_spd={susan_speed:+4.1f} twist_spd={twist_speed:+4.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        log.info("Stopping all servos...")
        neck.stop()
        susan.stop()
        twist.stop()


def main():
    parser = argparse.ArgumentParser(description="Servo test (no IMU required)")
    parser.add_argument('--simulated', action='store_true', help='Dry run, no hardware')
    parser.add_argument('--servo', choices=['neck', 'susan', 'twist'],
                        help='Test only one servo')
    parser.add_argument('--interactive', action='store_true', help='Keyboard control mode')
    parser.add_argument('--duration', type=float, default=6.0, help='Test duration per servo (s)')
    args = parser.parse_args()

    # Build servos
    if args.simulated:
        log.info("=== SIMULATED MODE ===")
        neck = SimulatedServo("neck_tilt", NECK_TILT_MIN, NECK_TILT_MAX)
        susan = SimulatedServo("lazy_susan", LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
        twist = SimulatedServo("head_twist")
    else:
        neck = ServoDriver(NECK_TILT_PIN, min_angle=105.0, max_angle=165.0)
        susan = ContinuousServoDriver(LAZY_SUSAN_PIN, label="lazy_susan")
        twist = ContinuousServoDriver(HEAD_TWIST_PIN, label="head_twist")

    # Graceful exit
    running = True

    def sig_handler(sig, frame):
        nonlocal running
        running = False
        neck.stop()
        susan.stop()
        twist.stop()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        if args.interactive:
            interactive_mode(neck, susan, twist)
        elif args.servo == 'neck':
            test_neck_tilt(neck, args.duration)
        elif args.servo == 'susan':
            test_lazy_susan(susan, args.duration)
        elif args.servo == 'twist':
            test_head_twist(twist, args.duration)
        else:
            # Test all in sequence
            test_neck_tilt(neck, args.duration)
            time.sleep(1.0)
            test_lazy_susan(susan, args.duration)
            time.sleep(1.0)
            test_head_twist(twist, args.duration)
    finally:
        log.info("Closing servos...")
        neck.close()
        susan.close()
        twist.close()
        log.info("All done.")


if __name__ == '__main__':
    main()
