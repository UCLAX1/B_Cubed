"""
One-time calibration for the 70kg lazy susan (BCM 12).
Uses encoder position (steps) for all movement — no time-based spinning.
Press d/a to nudge by 10 degrees, s to save forward as zero, q to quit.

Enc 0 pins (BCM): A=26 (physical 37), B=6 (physical 31), ABS=5 (physical 29)
2048 counts per revolution -> 1 degree = 5.69 steps
"""

import sys
import time
import tty
import termios
from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

SERVO_PIN     = 12
MOSFET_PIN    = 16
ENCODER_PIN_A = 26
ENCODER_PIN_B = 6
ABS_PIN       = 5

NUDGE_DEG    = 10.0
MEASURED_CPR = 1493
DEG_TO_STEPS = MEASURED_CPR / 360.0
MOVE_SPEED = 0.3
MIN_SPEED  = 0.08  # minimum speed to overcome friction
SLOW_ZONE  = 150   # steps — start scaling down speed within this range
DEADBAND   = 8     # steps (~2 deg at 4 steps/deg)

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()
print("Initializing encoder...")
enc = ServoEx(
    servo_pin=SERVO_PIN,
    encoder_pin_a=ENCODER_PIN_A,
    encoder_pin_b=ENCODER_PIN_B,
    absolute_encoder_pin=ABS_PIN,
    initial_value=None,
    pin_factory=factory,
)
enc.reset_encoder_position()

# ── detect direction ──────────────────────────────────────────
print("Detecting motor direction (spinning briefly)...")
enc.value = MOVE_SPEED
time.sleep(0.5)
enc.value = 0.0
time.sleep(0.3)
test_steps = enc.encoder.steps
direction_sign = 1 if test_steps >= 0 else -1
enc.encoder.steps = 0
print(f"Direction sign: {direction_sign} (detected from {test_steps} steps)")

print(f"Using CPR={MEASURED_CPR} (hardcoded). 1 degree = {DEG_TO_STEPS:.2f} steps")
print("Ready. Current position = 0 steps.")
print("d/RIGHT = +10 deg  |  a/LEFT = -10 deg  |  s = save as forward  |  q = quit\n")


def set_goal(degrees):
    return int(degrees * DEG_TO_STEPS)


def move_to(target_steps):
    while True:
        current = enc.encoder.steps
        error = target_steps - current
        print(f"  steps={current}  target={target_steps}  error={error}", flush=True)
        if abs(error) <= DEADBAND:
            break
        ratio = min(1.0, abs(error) / SLOW_ZONE)
        speed = MIN_SPEED + (MOVE_SPEED - MIN_SPEED) * ratio
        enc.value = direction_sign * speed if error > 0 else -direction_sign * speed
        time.sleep(0.02)
    enc.value = 0.0


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(2)
            return ch + ch2
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


target = 0

try:
    while True:
        current_steps = enc.encoder.steps
        current_deg = current_steps / DEG_TO_STEPS

        ch = getch()

        if ch in ('q', '\x03'):
            print("\nQuit without saving.")
            break
        elif ch == 's':
            enc.save_encoder_position()
            print(f"\nSaved! Forward = {current_deg:.1f} deg ({current_steps} steps)")
            break
        elif ch in ('\x1b[C', 'd'):
            target += set_goal(NUDGE_DEG)
        elif ch in ('\x1b[D', 'a'):
            target -= set_goal(NUDGE_DEG)
        else:
            continue

        print(f"Moving to {target / DEG_TO_STEPS:+.1f} deg ({target} steps)...")
        move_to(target)
        print(f"pos = {enc.encoder.steps / DEG_TO_STEPS:+.1f} deg  ({enc.encoder.steps} steps)", flush=True)

except KeyboardInterrupt:
    pass
finally:
    enc.value = 0.0
    time.sleep(0.5)
    mosfet.off()
    print("Done.")
