#!/usr/bin/env python3
"""
head_balance_controller.py — BB-8 Head Balance System
Runs on Raspberry Pi with SenseHat (IMU) and gpiozero (servos).

Head mechanism:
  - Neck tilt (Annimos 150KG, 0-270°): ±30° from center (0° = facing up)
  - Lazy susan (DIYMall 70KG continuous): ±180° yaw rotation
  - Head twist (Garosa 5.5kg continuous): unlimited rotation

Usage:
  python head_balance_controller.py                  # normal IMU-driven mode
  python head_balance_controller.py --simulated      # no hardware, print only
"""

import sys
import os
import math
import time
import signal
import argparse
import logging

# Add sim/ to path for head_balance_math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
from head_balance_math import find_motor_angles

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
log = logging.getLogger('head_balance')

# ─── Motor Constraints ───────────────────────────────────────────────
NECK_TILT_MIN = -30.0   # degrees (tilt backward)
NECK_TILT_MAX =  30.0   # degrees (tilt forward)
NECK_TILT_CENTER = 135.0  # Annimos servo center position (0-270° range)

LAZY_SUSAN_MIN = -180.0  # degrees
LAZY_SUSAN_MAX =  180.0  # degrees

# Head twist: unlimited, no clamping needed

# Safety: soft limit buffer — slow down near mechanical stops
SOFT_LIMIT_BUFFER = 5.0  # degrees from hard limit where we start slowing
RATE_LIMIT = 60.0        # max degrees per second per motor
IMU_TIMEOUT = 1.0        # seconds without IMU update before failsafe

# PID defaults
PID_KP = 1.0
PID_KI = 0.0
PID_KD = 0.1

# Update rate
LOOP_HZ = 50
LOOP_DT = 1.0 / LOOP_HZ


# ─── PID Controller ──────────────────────────────────────────────────
class PIDController:
    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        # Anti-windup: clamp integral
        self.integral = max(-100.0, min(100.0, self.integral))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ─── Safety Helpers ───────────────────────────────────────────────────
def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def soft_limit_scale(value, lo, hi, buffer=SOFT_LIMIT_BUFFER):
    """Returns a 0-1 scale factor that reduces to 0 at the hard limits."""
    if buffer <= 0:
        return 1.0
    dist_to_min = value - lo
    dist_to_max = hi - value
    nearest = min(dist_to_min, dist_to_max)
    if nearest >= buffer:
        return 1.0
    if nearest <= 0:
        return 0.0
    return nearest / buffer


def rate_limit_angle(current, target, max_rate, dt):
    """Limit how fast an angle can change per update."""
    max_delta = max_rate * dt
    delta = target - current
    delta = max(-max_delta, min(max_delta, delta))
    return current + delta


# ─── Servo Abstraction ───────────────────────────────────────────────
class ServoDriver:
    """Drives a servo via gpiozero on a Raspberry Pi."""

    def __init__(self, gpio_pin, min_angle, max_angle, inverted=False):
        from gpiozero import Servo as GpioServo
        from gpiozero.pins.pigpio import PiGPIOFactory
        # Use pigpio for hardware PWM (smoother)
        factory = PiGPIOFactory()
        self.servo = GpioServo(gpio_pin, pin_factory=factory)
        self.gpio_pin = gpio_pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.inverted = inverted
        self.current_angle = 0.0
        log.info(f"Servo on GPIO {gpio_pin}: range [{min_angle}, {max_angle}]°")

    def set_angle(self, angle_deg):
        """Set servo to angle in degrees. Clamps to safe range."""
        angle_deg = clamp(angle_deg, self.min_angle, self.max_angle)
        self.current_angle = angle_deg
        # Map angle to gpiozero's -1..+1 range
        mid = (self.min_angle + self.max_angle) / 2.0
        span = (self.max_angle - self.min_angle) / 2.0
        if span == 0:
            value = 0.0
        else:
            value = (angle_deg - mid) / span
        if self.inverted:
            value = -value
        value = max(-1.0, min(1.0, value))
        self.servo.value = value

    def neutral(self):
        self.set_angle(0.0)

    def stop(self):
        """Release PWM signal — servo goes limp (safe shutdown)."""
        self.servo.value = None

    def close(self):
        self.stop()
        self.servo.close()


class ContinuousServoDriver:
    """Drives a continuous rotation servo via gpiozero.
    set_speed(-1..+1) controls direction and speed."""

    def __init__(self, gpio_pin, label="continuous"):
        from gpiozero import Servo as GpioServo
        from gpiozero.pins.pigpio import PiGPIOFactory
        factory = PiGPIOFactory()
        self.servo = GpioServo(gpio_pin, pin_factory=factory)
        self.gpio_pin = gpio_pin
        self.label = label
        log.info(f"Continuous servo '{label}' on GPIO {gpio_pin}")

    def set_speed(self, speed):
        """speed: -1.0 (full reverse) to +1.0 (full forward), 0 = stop."""
        self.servo.value = max(-1.0, min(1.0, speed))

    def stop(self):
        self.servo.value = 0.0

    def close(self):
        self.stop()
        self.servo.close()


class SimulatedServo:
    """For testing without hardware."""

    def __init__(self, name, min_angle=None, max_angle=None):
        self.name = name
        self.current_angle = 0.0
        self.min_angle = min_angle
        self.max_angle = max_angle

    def set_angle(self, angle_deg):
        if self.min_angle is not None and self.max_angle is not None:
            angle_deg = clamp(angle_deg, self.min_angle, self.max_angle)
        self.current_angle = angle_deg

    def set_speed(self, speed):
        self.current_angle = speed  # store speed as value

    def neutral(self):
        self.current_angle = 0.0

    def stop(self):
        self.current_angle = 0.0

    def close(self):
        pass


# ─── IMU Reader ───────────────────────────────────────────────────────
class SenseHatIMU:
    """Reads orientation from SenseHat."""

    def __init__(self):
        from sense_hat import SenseHat
        self.sense = SenseHat()
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.last_update = time.time()
        log.info("SenseHat IMU initialized")

    def update(self):
        orientation = self.sense.get_orientation()
        self.pitch = orientation['pitch']
        self.roll = orientation['roll']
        self.yaw = orientation['yaw']
        self.last_update = time.time()

    @property
    def age(self):
        return time.time() - self.last_update


class SimulatedIMU:
    """For testing without SenseHat hardware."""

    def __init__(self):
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.last_update = time.time()

    def update(self):
        self.last_update = time.time()

    @property
    def age(self):
        return time.time() - self.last_update


# ─── Main Controller ─────────────────────────────────────────────────
class HeadBalanceController:
    """
    Reads IMU, computes balance angles, drives servos with safety.

    Motors:
      neck_tilt  — standard servo, ±30° (Annimos 150KG, 0-270° range)
      lazy_susan — continuous servo, ±180° tracked position
      head_twist — continuous servo, unlimited
    """

    def __init__(self, imu, neck_tilt_servo, lazy_susan_servo, head_twist_servo,
                 desired_head_angle=0.0):
        self.imu = imu
        self.neck_tilt = neck_tilt_servo
        self.lazy_susan = lazy_susan_servo
        self.head_twist = head_twist_servo
        self.desired_head_angle = desired_head_angle

        # PID controllers for each axis
        self.pid_tilt = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.pid_susan = PIDController(kp=0.8, ki=0.0, kd=0.1)
        self.pid_twist = PIDController(kp=0.5, ki=0.0, kd=0.05)

        # Current commanded positions (for rate limiting)
        self.current_tilt = 0.0
        self.current_susan = 0.0
        self.current_twist = 0.0

        # Tracking for continuous servos
        self.susan_position = 0.0  # estimated position of lazy susan
        self.twist_position = 0.0  # estimated position of head twist

        self.running = False
        self.failsafe_active = False

    def _enter_failsafe(self, reason):
        """Stop all motors and hold position."""
        if not self.failsafe_active:
            log.warning(f"FAILSAFE: {reason}")
            self.failsafe_active = True
        self.neck_tilt.stop()
        self.lazy_susan.stop()
        self.head_twist.stop()

    def _exit_failsafe(self):
        if self.failsafe_active:
            log.info("FAILSAFE cleared — resuming control")
            self.failsafe_active = False
            self.pid_tilt.reset()
            self.pid_susan.reset()
            self.pid_twist.reset()

    def update(self, dt):
        """Single control loop iteration."""
        # Read IMU
        try:
            self.imu.update()
        except Exception as e:
            self._enter_failsafe(f"IMU read error: {e}")
            return

        # Check IMU freshness
        if self.imu.age > IMU_TIMEOUT:
            self._enter_failsafe("IMU data stale")
            return

        if self.failsafe_active:
            self._exit_failsafe()

        pitch = self.imu.pitch
        roll = self.imu.roll

        # Compute desired motor angles from balance math
        arm_angle, susan_angle, twist_angle = find_motor_angles(
            pitch, roll, self.desired_head_angle
        )

        # --- Neck tilt (standard servo) ---
        target_tilt = clamp(arm_angle, NECK_TILT_MIN, NECK_TILT_MAX)
        # Apply soft limit scaling near edges
        scale = soft_limit_scale(target_tilt, NECK_TILT_MIN, NECK_TILT_MAX)
        tilt_error = target_tilt - self.current_tilt
        tilt_cmd = self.pid_tilt.update(tilt_error, dt) * scale
        self.current_tilt = rate_limit_angle(
            self.current_tilt, self.current_tilt + tilt_cmd, RATE_LIMIT, dt
        )
        self.current_tilt = clamp(self.current_tilt, NECK_TILT_MIN, NECK_TILT_MAX)
        # Map ±30° to physical servo position on 0-270° servo
        physical_tilt = NECK_TILT_CENTER + self.current_tilt
        self.neck_tilt.set_angle(physical_tilt)

        # --- Lazy susan (continuous, tracked ±180°) ---
        target_susan = clamp(susan_angle, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
        scale = soft_limit_scale(target_susan, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
        susan_error = target_susan - self.susan_position
        susan_cmd = self.pid_susan.update(susan_error, dt) * scale
        # Convert PID output to speed command for continuous servo
        speed = max(-1.0, min(1.0, susan_cmd / 90.0))  # normalize
        self.lazy_susan.set_speed(speed)
        # Estimate position change (rough, since no encoder)
        self.susan_position += speed * RATE_LIMIT * dt
        self.susan_position = clamp(self.susan_position, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)

        # --- Head twist (continuous, unlimited) ---
        twist_error = twist_angle - self.twist_position
        twist_cmd = self.pid_twist.update(twist_error, dt)
        speed = max(-1.0, min(1.0, twist_cmd / 90.0))
        self.head_twist.set_speed(speed)
        self.twist_position += speed * RATE_LIMIT * dt

    def stop(self):
        """Emergency stop — all servos to neutral/stop."""
        self.running = False
        log.info("Stopping all servos")
        try:
            self.neck_tilt.stop()
        except Exception:
            pass
        try:
            self.lazy_susan.stop()
        except Exception:
            pass
        try:
            self.head_twist.stop()
        except Exception:
            pass

    def close(self):
        """Release all hardware resources."""
        self.stop()
        try:
            self.neck_tilt.close()
        except Exception:
            pass
        try:
            self.lazy_susan.close()
        except Exception:
            pass
        try:
            self.head_twist.close()
        except Exception:
            pass

    def run(self):
        """Main control loop."""
        self.running = True
        log.info("Head balance controller started (Ctrl+C to stop)")
        log.info(f"  Neck tilt: ±{NECK_TILT_MAX}° | Lazy susan: ±{LAZY_SUSAN_MAX}° | Head twist: unlimited")

        prev_time = time.time()
        try:
            while self.running:
                now = time.time()
                dt = now - prev_time
                prev_time = now

                self.update(dt)

                # Telemetry
                if int(now * 2) % 2 == 0:  # ~every 0.5s
                    log.info(
                        f"IMU p={self.imu.pitch:6.1f} r={self.imu.roll:6.1f} | "
                        f"tilt={self.current_tilt:5.1f}° susan={self.susan_position:6.1f}° "
                        f"twist={self.twist_position:6.1f}°"
                    )

                # Maintain loop rate
                elapsed = time.time() - now
                sleep_time = LOOP_DT - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            log.info("KeyboardInterrupt received")
        finally:
            self.close()


# ─── GPIO Pin Assignments (CHANGE THESE TO MATCH YOUR WIRING) ────────
# These are BCM GPIO numbers
NECK_TILT_PIN = 12    # Annimos 150KG servo
LAZY_SUSAN_PIN = 13   # DIYMall 70KG continuous servo
HEAD_TWIST_PIN = 18   # Garosa 5.5kg continuous servo


def build_hardware():
    """Create real hardware drivers."""
    imu = SenseHatIMU()
    neck = ServoDriver(NECK_TILT_PIN, min_angle=105.0, max_angle=165.0)
    # 105° = center(135) - 30, 165° = center(135) + 30
    susan = ContinuousServoDriver(LAZY_SUSAN_PIN, label="lazy_susan")
    twist = ContinuousServoDriver(HEAD_TWIST_PIN, label="head_twist")
    return imu, neck, susan, twist


def build_simulated():
    """Create simulated drivers for testing without hardware."""
    imu = SimulatedIMU()
    neck = SimulatedServo("neck_tilt", min_angle=NECK_TILT_MIN, max_angle=NECK_TILT_MAX)
    susan = SimulatedServo("lazy_susan", min_angle=LAZY_SUSAN_MIN, max_angle=LAZY_SUSAN_MAX)
    twist = SimulatedServo("head_twist")
    return imu, neck, susan, twist


def main():
    parser = argparse.ArgumentParser(description="BB-8 Head Balance Controller")
    parser.add_argument('--simulated', action='store_true',
                        help='Run without hardware (print-only mode)')
    parser.add_argument('--desired-angle', type=float, default=0.0,
                        help='Desired head facing angle in degrees (default: 0)')
    args = parser.parse_args()

    if args.simulated:
        log.info("=== SIMULATED MODE (no hardware) ===")
        imu, neck, susan, twist = build_simulated()
    else:
        imu, neck, susan, twist = build_hardware()

    controller = HeadBalanceController(imu, neck, susan, twist,
                                       desired_head_angle=args.desired_angle)

    # Graceful shutdown on signals
    def signal_handler(sig, frame):
        log.info(f"Signal {sig} received, shutting down...")
        controller.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    controller.run()


if __name__ == '__main__':
    main()
