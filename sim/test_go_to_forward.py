"""
Moves the lazy susan and head to their saved forward positions (0 steps).
Run this to confirm save_forward_position.py saved correctly.
"""

import time
from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

MOSFET_PIN    = 16
LAZY_SERVO    = 12
LAZY_A, LAZY_B, LAZY_ABS = 26, 6, 5
HEAD_SERVO    = 20
HEAD_A, HEAD_B, HEAD_ABS = 4, 22, 17

LAZY_CPR = 1493
HEAD_CPR = 2048

MOVE_SPEED = 0.3
MIN_SPEED  = 0.12
SLOW_ZONE  = 150
DEADBAND   = 8

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()

print("Initializing lazy susan...")
lazy = ServoEx(LAZY_SERVO, LAZY_A, LAZY_B, LAZY_ABS, initial_value=None, pin_factory=factory)
print("Initializing head...")
head = ServoEx(HEAD_SERVO, HEAD_A, HEAD_B, HEAD_ABS, initial_value=None)

print(f"Lazy susan loaded position: {lazy.encoder.steps} steps ({lazy.encoder.steps * 360.0 / LAZY_CPR:.1f} deg)")
print(f"Head loaded position:       {head.encoder.steps} steps ({head.encoder.steps * 360.0 / HEAD_CPR:.1f} deg)")


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


def move_to_zero(servo, name, direction_sign):
    target = 0
    initial_error_sign = None
    print(f"\nMoving {name} to forward (0 steps)...")
    while True:
        current = servo.encoder.steps
        error = target - current
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


# detect direction for each servo
def detect_direction(servo):
    pre = servo.encoder.steps
    servo.value = MOVE_SPEED
    time.sleep(0.5)
    servo.value = 0.0
    time.sleep(0.3)
    delta = servo.encoder.steps - pre
    servo.encoder.steps = pre  # restore — don't lose the loaded saved position
    return 1 if delta >= 0 else -1


print("\nDetecting lazy susan direction...")
lazy_dir = detect_direction(lazy)
print(f"  direction_sign = {lazy_dir}")

print("Detecting head direction...")
head_dir = detect_direction(head)
print(f"  direction_sign = {head_dir}")

move_to_zero(lazy, "lazy susan", lazy_dir)
move_to_zero(head, "head", head_dir)

print("\nDone. Both servos at forward position.")

lazy.value = None
head.value = None
time.sleep(0.2)
mosfet.off()
