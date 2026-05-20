"""
One-time calibration for the 5.5kg head servo (BCM 18).
Spin it to the forward-facing position, then press 's' to save that as zero.

Enc 1 pins (BCM): A=4 (physical 7), B=22 (physical 15), ABS=17 (physical 11)

Controls:
  RIGHT / d : spin clockwise
  LEFT  / a : spin counter-clockwise
  SPACE     : stop
  s         : save current position as zero (forward) and quit
  q         : quit without saving
"""

import sys
import time
import tty
import termios
from gpiozero import DigitalOutputDevice
from ServoEx import ServoEx

SERVO_PIN      = 18
MOSFET_PIN     = 16
ENCODER_PIN_A  = 4
ENCODER_PIN_B  = 22
ABS_PIN        = 17
SPIN_SPEED     = 0.15

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

print("Initializing encoder...")
enc = ServoEx(
    servo_pin=SERVO_PIN,
    encoder_pin_a=ENCODER_PIN_A,
    encoder_pin_b=ENCODER_PIN_B,
    absolute_encoder_pin=ABS_PIN,
    initial_value=None,
)
servo = enc
enc.reset_encoder_position()
print("Encoder zeroed at current position.")
print("\nSpin to forward-facing position, then press 's' to save.")
print("RIGHT/d = clockwise  |  LEFT/a = counter-clockwise  |  SPACE = stop  |  s = save  |  q = quit\n")

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
        enc.update()
        pos_deg = enc.get_position() * 360.0

        ch = getch()

        if ch in ('q', '\x03'):
            print("\nQuit without saving.")
            break
        elif ch == 's':
            enc.save_encoder_position()
            print(f"\nSaved! Forward = {pos_deg:.1f} deg ({enc.get_position():.4f} rot)")
            print("Run test_head_reset.py to return here in future.")
            break
        elif ch in ('\x1b[C', 'd'):
            servo.value = SPIN_SPEED
        elif ch in ('\x1b[D', 'a'):
            servo.value = -SPIN_SPEED
        elif ch == ' ':
            servo.value = 0.0

        print(f"pos = {pos_deg:+.1f} deg  ({enc.get_position():+.4f} rot)", flush=True)

except KeyboardInterrupt:
    pass
finally:
    servo.value = 0.0
    time.sleep(0.5)
    mosfet.off()
    print("Done.")
