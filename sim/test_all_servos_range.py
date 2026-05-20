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
ARM_LIMIT     = 30 / 90  # 30 degrees in servo units

LAZY_PIN      = 12
HEAD_PIN      = 18
CONT_SPEED    = 0.15  # slow spin speed for continuous servos
CONT_DURATION = 2.0   # seconds to spin each direction

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
    time.sleep(0.5)


# ── 150kg ARM ────────────────────────────────────────────────
print("\n=== ARM SERVO (150kg) ===")
print("Ramping to vertical...")
ramp(arm, 0.0, ARM_VERTICAL)
time.sleep(0.5)

pos_limit = ARM_VERTICAL + ARM_LIMIT
neg_limit = ARM_VERTICAL - ARM_LIMIT

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
spin(lazy, "lazy susan", +CONT_SPEED, CONT_DURATION)
spin(lazy, "lazy susan", -CONT_SPEED, CONT_DURATION)
print("  lazy susan centered (stopped).")

# ── 5.5kg HEAD ───────────────────────────────────────────────
print("\n=== HEAD SERVO (5.5kg) ===")
spin(head, "head", +CONT_SPEED, CONT_DURATION)
spin(head, "head", -CONT_SPEED, CONT_DURATION)
print("  head centered (stopped).")

# ── DONE ─────────────────────────────────────────────────────
print("\nAll done. Powering off.")
arm.value  = None
lazy.value = None
head.value = None
time.sleep(0.5)
mosfet.off()
