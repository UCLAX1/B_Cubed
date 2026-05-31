"""
Slowly moves each servo to both limits then returns to center/stop.

150kg arm (BCM 13): standard servo, vertical = -0.66, limits ±30° (±0.333 servo units)
70kg lazy susan (BCM 12): continuous servo, spins each direction slowly then stops
5.5kg head (BCM 18): continuous servo, spins each direction slowly then stops
"""

import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

MOSFET_PIN = 16

ARM_PIN       = 13
ARM_VERTICAL  = -0.66
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5
ARM_LIMIT     = 30 / 90  # 30 degrees in servo units

LAZY_PIN      = 12
HEAD_PIN      = 18
CONT_SPEED    = 0.3   # spin speed for continuous servos
HEAD_DURATION = 4.0   # seconds to spin head each direction
LAZY_DURATION = 3.0   # seconds to spin lazy susan to ~90 degrees

STEP       = 0.005
STEP_DELAY = 0.04

factory = PiGPIOFactory()
mosfet  = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

arm       = Servo(ARM_PIN,  initial_value=None, pin_factory=factory)
lazy      = Servo(LAZY_PIN, initial_value=None, pin_factory=factory)
head      = Servo(HEAD_PIN, initial_value=None, pin_factory=factory)


def ramp(servo, from_val, to_val):
    current = from_val
    servo.value = current
    step = STEP if to_val > from_val else -STEP
    while (step > 0 and current < to_val) or (step < 0 and current > to_val):
        current += step
        current = round(current, 4)
        servo.value = current
        print(f"  -> {current:+.4f}", flush=True)
        time.sleep(STEP_DELAY)
    servo.value = to_val


def spin(servo, name, speed, duration):
    print(f"  {name} spinning at {speed:+.2f} for {duration:.1f}s...")
    servo.value = speed
    time.sleep(duration)
    servo.value = 0.0
    print(f"  {name} stopped.")
    time.sleep(1.0)


# ── 150kg ARM ────────────────────────────────────────────────
print("\n=== ARM SERVO (150kg) ===")
print("Setting to vertical (arm should already be there)...")
arm.value = max(min(ARM_VERTICAL, ARM_SERVO_MAX), ARM_SERVO_MIN)
time.sleep(1.0)

pos_limit = max(min(ARM_VERTICAL + ARM_LIMIT, ARM_SERVO_MAX), ARM_SERVO_MIN)
neg_limit = max(min(ARM_VERTICAL - ARM_LIMIT, ARM_SERVO_MAX), ARM_SERVO_MIN)

print(f"Ramping to positive limit ({pos_limit:+.3f})...")
ramp(arm, ARM_VERTICAL, pos_limit)
time.sleep(0.5)

print("Returning to vertical...")
ramp(arm, pos_limit, ARM_VERTICAL)
time.sleep(0.5)

print(f"Ramping to negative limit ({neg_limit:+.3f})...")
ramp(arm, ARM_VERTICAL, neg_limit)
time.sleep(0.5)

print("Returning to vertical...")
ramp(arm, neg_limit, ARM_VERTICAL)
time.sleep(1.0)

# ── 70kg LAZY SUSAN ──────────────────────────────────────────
print("\n=== LAZY SUSAN (70kg) ===")
spin(lazy, "lazy susan", -CONT_SPEED, LAZY_DURATION)  # left ~90 degrees
spin(lazy, "lazy susan", +CONT_SPEED, LAZY_DURATION)  # return to center
print("  lazy susan centered (stopped).")

# ── 5.5kg HEAD ───────────────────────────────────────────────
print("\n=== HEAD SERVO (5.5kg) ===")
spin(head, "head", +CONT_SPEED, HEAD_DURATION)
spin(head, "head", -CONT_SPEED, HEAD_DURATION)
print("  head centered (stopped).")

# ── DONE ─────────────────────────────────────────────────────
print("\nAll done. Powering off.")
arm.value  = None
lazy.value = None
head.value = None
time.sleep(0.5)
mosfet.off()
