"""
Resets all three servos to their forward/vertical positions.
  - Arm (150kg, BCM 13): moves to hardcoded vertical (-0.66)
  - Lazy susan (70kg, BCM 12): moves to saved forward position (0 steps)
  - Head (5.5kg, BCM 18): moves to saved forward position (0 steps)
"""

import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

MOSFET_PIN    = 16
ARM_PIN       = 13
ARM_VERTICAL  = -0.66

LAZY_SERVO    = 12
LAZY_A, LAZY_B, LAZY_ABS = 26, 6, 5
LAZY_CPR      = 1493

HEAD_SERVO    = 18
HEAD_A, HEAD_B, HEAD_ABS = 4, 22, 17
HEAD_CPR      = 2048

MOVE_SPEED = 0.3
MIN_SPEED  = 0.12
SLOW_ZONE  = 150
DEADBAND   = 8

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()

print("Initializing servos...")
arm  = Servo(ARM_PIN, initial_value=None, pin_factory=factory)
lazy = ServoEx(LAZY_SERVO, LAZY_A, LAZY_B, LAZY_ABS, initial_value=None, pin_factory=factory)
head = ServoEx(HEAD_SERVO, HEAD_A, HEAD_B, HEAD_ABS, initial_value=None)

print(f"Arm:        will move to servo value {ARM_VERTICAL}")
print(f"Lazy susan: loaded at {lazy.encoder.steps} steps ({lazy.encoder.steps * 360.0 / LAZY_CPR:.1f} deg from saved forward)")
print(f"Head:       loaded at {head.encoder.steps} steps ({head.encoder.steps * 360.0 / HEAD_CPR:.1f} deg from saved forward)")


def wait_for_stable(enc):
    prev = enc.encoder.steps
    consec = 0
    deadline = time.time() + 2.0
    while time.time() < deadline:
        time.sleep(0.05)
        cur = enc.encoder.steps
        if abs(cur - prev) <= 1:
            consec += 1
            if consec >= 4:
                return
        else:
            consec = 0
        prev = cur


def detect_direction(servo):
    pre = servo.encoder.steps
    servo.value = MOVE_SPEED
    time.sleep(0.5)
    servo.value = 0.0
    time.sleep(0.3)
    delta = servo.encoder.steps - pre
    servo.encoder.steps = pre  # restore — don't lose the loaded saved position
    return 1 if delta >= 0 else -1


def move_to_zero(servo, name, direction_sign):
    print(f"\nMoving {name} to forward (0 steps)...")
    initial_error_sign = None
    while True:
        current = servo.encoder.steps
        error = -current  # target is always 0
        if abs(error) <= DEADBAND:
            print(f"  STOP  steps={current}  error={error}")
            break
        if initial_error_sign is None:
            initial_error_sign = 1 if error > 0 else -1
        elif (1 if error > 0 else -1) != initial_error_sign:
            print(f"  OVERSHOOT  steps={current}  error={error}")
            break
        ratio = min(1.0, abs(error) / SLOW_ZONE)
        speed = MIN_SPEED + (MOVE_SPEED - MIN_SPEED) * ratio
        servo.value = direction_sign * speed if error > 0 else -direction_sign * speed
        time.sleep(0.02)
    servo.value = 0.0
    wait_for_stable(servo)
    print(f"  {name} resting at {servo.encoder.steps} steps")


# ── Arm ───────────────────────────────────────────────────────
print("\nMoving arm to vertical...")
arm.value = ARM_VERTICAL
time.sleep(1.5)  # standard servo, just wait for it to reach position
print(f"  arm at {ARM_VERTICAL}")

# ── Lazy susan ────────────────────────────────────────────────
print("Detecting lazy susan direction...")
lazy_dir = detect_direction(lazy)
move_to_zero(lazy, "lazy susan", lazy_dir)

# ── Head ──────────────────────────────────────────────────────
print("Detecting head direction...")
head_dir = detect_direction(head)
move_to_zero(head, "head", head_dir)

print("\nAll servos at home position.")

arm.value = None
lazy.value = None
head.value = None
time.sleep(0.2)
mosfet.off()
print("Done.")
