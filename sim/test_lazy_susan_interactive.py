"""
Interactive lazy susan movement with encoder feedback.
Mimics head_balance proportional speed control.

Controls:
  + / d     : increase target (move left/positive)
  - / a     : decrease target (move right/negative)
  0         : reset target to center
  p         : print current position & target
  q / Ctrl+C: quit
"""

import sys
import time
import tty
import termios
from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

SERVO_PIN  = 12
MOSFET_PIN = 16

LAZY_CPR = 1493
CONT_MAX_ANGLE_SPEED = 0.5

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()
lazy = ServoEx(12, 26, 6, 5, initial_value=None, pin_factory=factory)
print(f"Lazy susan initialized. Encoder: {lazy.encoder.steps} steps")

target_deg = 0.0
current_deg = 0.0
step_size = 5.0  # degrees per keypress

print(f"Lazy susan ready. Starting at center (target={target_deg}°)")
print("+ = increase target  |  - = decrease target  |  0 = center  |  p = print  |  q = quit\n")

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

def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)

def angle_to_servo_value(angle_deg, servo_type="standard"):
    if servo_type == "standard":
        return clamp(angle_deg / 90.0, -1.0, 1.0)
    return clamp(angle_deg / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)

try:
    while True:
        # Read current position from encoder
        current_deg = lazy.encoder.steps * 360.0 / LAZY_CPR

        # Calculate error
        error_deg = target_deg - current_deg

        # Proportional speed control (like head_balance)
        cmd = angle_to_servo_value(error_deg, "continuous")

        # Send to servo
        lazy.value = cmd

        # Try to get keypress (non-blocking)
        ch = getch()

        if ch in ('q', '\x03'):
            break
        elif ch in ('+', 'd'):
            target_deg = min(90.0, target_deg + step_size)
            print(f"Target: {target_deg:+.1f}°  Current: {current_deg:+.1f}°  Error: {error_deg:+.1f}°  Cmd: {cmd:+.3f}")
        elif ch in ('-', 'a'):
            target_deg = max(-90.0, target_deg - step_size)
            print(f"Target: {target_deg:+.1f}°  Current: {current_deg:+.1f}°  Error: {error_deg:+.1f}°  Cmd: {cmd:+.3f}")
        elif ch == '0':
            target_deg = 0.0
            print(f"Target reset to 0°")
        elif ch == 'p':
            print(f"Target: {target_deg:+.1f}°  Current: {current_deg:+.1f}°  Error: {error_deg:+.1f}°  Cmd: {cmd:+.3f}  Steps: {lazy.encoder.steps}")
        else:
            continue

        time.sleep(0.05)  # 20 Hz update rate

except KeyboardInterrupt:
    pass
finally:
    print(f"\nStopping.")
    lazy.value = 0.0
    lazy.deactivate_and_save()
    time.sleep(0.5)
    mosfet.off()
    print(f"Final position saved: {lazy.encoder.steps} steps ({current_deg:+.1f}°)")
    print("Done.")
