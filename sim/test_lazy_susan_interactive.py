"""
Interactive lazy susan movement with encoder feedback (encoder 0 = 70kg).
Mimics head_balance proportional speed control. Outputs values continuously.

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

SERVO_PIN  = 12  # Lazy susan servo
MOSFET_PIN = 16

# Encoder 0 pins (70kg lazy susan)
ENC_A = 26
ENC_B = 6
ENC_ABS = 5

LAZY_CPR = 1493
CONT_MAX_ANGLE_SPEED = 0.5

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()
lazy = ServoEx(SERVO_PIN, ENC_A, ENC_B, ENC_ABS, initial_value=None, pin_factory=factory)
print(f"Lazy susan (enc 0) initialized. Encoder: {lazy.encoder.steps} steps")

target_deg = 0.0
cmd_last = 0.0
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

def angle_to_servo_value(angle_deg):
    return clamp(angle_deg / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)

last_print = time.time()
print_interval = 0.2  # Print every 0.2 seconds

try:
    while True:
        now = time.time()

        # Read current position from encoder
        current_deg = lazy.encoder.steps * 360.0 / LAZY_CPR

        # Calculate error
        error_deg = target_deg - current_deg

        # Proportional speed control (like head_balance)
        cmd = angle_to_servo_value(error_deg)

        # Send to servo
        lazy.value = cmd
        cmd_last = cmd

        # Print values periodically
        if now - last_print >= print_interval:
            print(
                f"Tgt: {target_deg:+7.2f}°  "
                f"Cur: {current_deg:+7.2f}°  "
                f"Err: {error_deg:+7.2f}°  "
                f"Cmd: {cmd:+.3f}  "
                f"Steps: {lazy.encoder.steps}"
            )
            last_print = now

        # Non-blocking keypress
        try:
            ch = getch()

            if ch in ('q', '\x03'):
                break
            elif ch in ('+', 'd'):
                target_deg = min(90.0, target_deg + step_size)
            elif ch in ('-', 'a'):
                target_deg = max(-90.0, target_deg - step_size)
            elif ch == '0':
                target_deg = 0.0
            elif ch == 'p':
                print(f"\n[Manual print] Tgt: {target_deg:+7.2f}°  Cur: {current_deg:+7.2f}°  Err: {error_deg:+7.2f}°  Cmd: {cmd:+.3f}  Steps: {lazy.encoder.steps}\n")
        except:
            pass

        time.sleep(0.02)  # 50 Hz control rate

except KeyboardInterrupt:
    pass
finally:
    print(f"\nStopping.")
    lazy.value = 0.0
    lazy.deactivate_and_save()
    time.sleep(0.5)
    mosfet.off()
    current_deg = lazy.encoder.steps * 360.0 / LAZY_CPR
    print(f"Final position saved: {lazy.encoder.steps} steps ({current_deg:+.1f}°)")
    print("Done.")
