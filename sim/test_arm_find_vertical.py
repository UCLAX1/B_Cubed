"""
Interactive fine-tuning for the 150kg arm servo (BCM 13, physical 33).
Use arrow keys or +/- to nudge the servo and find the vertical position.

Controls:
  RIGHT / d / + : nudge positive (extend arm)
  LEFT  / a / - : nudge negative (retract arm)
  f             : fine mode (smaller step, 0.005)
  c             : coarse mode (larger step, 0.02)
  s             : save current position as vertical reference
  0             : go to center (0.0)
  p             : print current value
  q             : quit
"""

import sys
import time
import tty
import termios
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

SERVO_PIN  = 13
MOSFET_PIN = 16

FINE_STEP   = 0.005
COARSE_STEP = 0.02

ARM_VERTICAL_OFFSET = -0.66  # The vertical reference point

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()
servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory)
current = ARM_VERTICAL_OFFSET  # Start at vertical
step = FINE_STEP

print(f"150kg ARM servo ready. Starting at vertical: {current:.4f}")
print("RIGHT/d/+ = nudge +  |  LEFT/a/- = nudge -  |  f=fine  c=coarse  s=save  0=center  p=print  q=quit\n")

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

try:
    while True:
        ch = getch()

        if ch in ('q', '\x03'):
            break
        elif ch in ('\x1b[C', 'd', '+'):  # right arrow, d, +
            current = min(1.0, current + step)
        elif ch in ('\x1b[D', 'a', '-'):  # left arrow, a, -
            current = max(-1.0, current - step)
        elif ch == '0':
            current = 0.0
        elif ch == 'f':
            step = FINE_STEP
            print(f"Fine mode (step={FINE_STEP})")
            continue
        elif ch == 'c':
            step = COARSE_STEP
            print(f"Coarse mode (step={COARSE_STEP})")
            continue
        elif ch == 'p':
            print(f"Current value: {current:.4f}  (offset from vertical: {current - ARM_VERTICAL_OFFSET:+.4f})")
            continue
        elif ch == 's':
            print(f"\n✓ Saved vertical position: {current:.4f}")
            print(f"  Update ARM_VERTICAL_OFFSET = {current:.4f} in head_balance_servo_control_slow.py")
            continue
        else:
            continue

        servo.value = current
        print(f"arm = {current:+.4f}  (offset: {current - ARM_VERTICAL_OFFSET:+.4f})", flush=True)

except KeyboardInterrupt:
    pass
finally:
    print(f"\nFinal value: {current:+.4f}")
    servo.value = 0.0
    time.sleep(0.5)
    mosfet.off()
    print("Done.")
