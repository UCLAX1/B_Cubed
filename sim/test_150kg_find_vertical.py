"""
Interactive fine-tuning for the 150kg standard servo (BCM 13, physical 33).
Use arrow keys or +/- to nudge the servo and find the vertical position.

Controls:
  RIGHT / d / + : nudge positive
  LEFT  / a / - : nudge negative
  f             : fine mode (smaller step)
  c             : coarse mode (larger step)
  0             : go to center (0.0)
  p             : print current value
  q             : quit
"""

import sys
import time
import tty
import termios
from gpiozero import DigitalOutputDevice, Servo

SERVO_PIN  = 13
MOSFET_PIN = 16

FINE_STEP   = 0.005
COARSE_STEP = 0.02

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

servo = Servo(SERVO_PIN, initial_value=None)
current = 0.0
step = FINE_STEP

print("150kg servo ready. Current value: 0.000")
print("RIGHT/d/+ = nudge +  |  LEFT/a/- = nudge -  |  f=fine  c=coarse  0=center  p=print  q=quit\n")

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
            print(f"Current value: {current:.4f}")
            continue
        else:
            continue

        servo.value = current
        print(f"servo = {current:+.4f}")

except KeyboardInterrupt:
    pass
finally:
    print(f"\nFinal value: {current:+.4f}  <-- save this as your vertical point")
    servo.value = 0.0
    time.sleep(0.5)
    mosfet.off()
    print("Done.")
