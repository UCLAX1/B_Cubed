"""
Interactive debug script for both continuous-servo encoders.

Use this to verify that the lazy susan and head encoders both respond while
you manually nudge the motors. The script prints relative encoder steps,
relative degrees, and absolute encoder readings on every update.

Controls:
  l / L : nudge lazy susan forward / backward
  h / H : nudge head forward / backward
  space : stop both motors
  s     : save the current encoder positions and exit
  q     : quit without saving
"""

import sys
import time
import tty
import termios
import pigpio

from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx


MOSFET_PIN = 16

LAZY_SERVO = 12
LAZY_A, LAZY_B, LAZY_ABS = 26, 6, 5
LAZY_CPR = 8192

HEAD_SERVO = 20
HEAD_A, HEAD_B, HEAD_ABS = 4, 22, 17
HEAD_CPR = 8192

NUDGE_SPEED = 0.2
HEAD_NUDGE_SPEED = 0.35
NUDGE_TIME_S = 0.6
ABS_VALID_MIN = 0.0
ABS_VALID_MAX = 1.0


class RawPinMonitor:
    def __init__(self, pins):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod is not available")

        self.edge_counts = {pin: 0 for pin in pins}
        self.callbacks = [
            self.pi.callback(pin, pigpio.EITHER_EDGE, self._count_edge)
            for pin in pins
        ]

    def _count_edge(self, pin, level, _tick):
        if level in (0, 1):
            self.edge_counts[pin] += 1

    def snapshot(self):
        return {
            pin: (self.pi.read(pin), self.edge_counts[pin])
            for pin in self.edge_counts
        }

    def close(self):
        for callback in self.callbacks:
            callback.cancel()
        self.pi.stop()


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_axis_debug(servo, cpr):
    servo.update()
    steps = servo.encoder.steps
    rel_deg = steps * 360.0 / cpr
    abs_value = servo.get_absolute_position()
    abs_deg = abs_value * 360.0
    abs_valid = ABS_VALID_MIN < abs_value < ABS_VALID_MAX
    return steps, rel_deg, abs_value, abs_deg, abs_valid


mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()
raw_pins = RawPinMonitor([LAZY_A, LAZY_B, LAZY_ABS, HEAD_A, HEAD_B, HEAD_ABS])

print("Initializing lazy susan encoder...")
lazy = ServoEx(LAZY_SERVO, LAZY_A, LAZY_B, LAZY_ABS, initial_value=None, pin_factory=factory)
print("Initializing head encoder...")
head = ServoEx(HEAD_SERVO, HEAD_A, HEAD_B, HEAD_ABS, initial_value=None, pin_factory=factory)

lazy.value = 0.0
head.value = 0.0
time.sleep(0.5)

print("\nEncoder debug ready.")
print("  l/L = lazy forward/backward")
print("  h/H = head forward/backward")
print("  space = stop motors")
print("  s = save current encoder positions and exit")
print("  q = quit without saving\n")


def print_status():
    lazy_steps, lazy_rel_deg, lazy_abs_value, lazy_abs_deg, lazy_abs_valid = read_axis_debug(lazy, LAZY_CPR)
    head_steps, head_rel_deg, head_abs_value, head_abs_deg, head_abs_valid = read_axis_debug(head, HEAD_CPR)

    print(
        f"lazy: steps={lazy_steps:8d} rel={lazy_rel_deg:8.2f}° "
        f"abs={lazy_abs_value:5.3f} ({lazy_abs_deg:7.2f}°) "
        f"{'OK' if lazy_abs_valid else 'BAD'}"
    )
    print(
        f"head: steps={head_steps:8d} rel={head_rel_deg:8.2f}° "
        f"abs={head_abs_value:5.3f} ({head_abs_deg:7.2f}°) "
        f"{'OK' if head_abs_valid else 'BAD'}"
    )
    print("-" * 78)


def print_raw_delta(before, after):
    def describe(pin):
        level, edges = after[pin]
        return f"{pin}={level} edges={edges - before[pin][1]:+d}"

    print(
        "raw delta: "
        f"lazy[A {describe(LAZY_A)}, B {describe(LAZY_B)}, ABS {describe(LAZY_ABS)}] "
        f"head[A {describe(HEAD_A)}, B {describe(HEAD_B)}, ABS {describe(HEAD_ABS)}]"
    )


def nudge(servo, direction):
    servo.value = clamp(direction * NUDGE_SPEED, -1.0, 1.0)


def nudge_head(servo, direction):
    servo.value = clamp(direction * HEAD_NUDGE_SPEED, -1.0, 1.0)


try:
    print_status()
    while True:
        ch = getch()

        if ch in ("q", "\x03"):
            print("\nQuit without saving.")
            break
        if ch == "s":
            lazy.save_encoder_position()
            head.save_encoder_position()
            print("\nSaved current encoder positions.")
            break
        if ch == " ":
            lazy.value = 0.0
            head.value = 0.0
            print_status()
            continue

        if ch in ("l", "L", "h", "H"):
            raw_before = raw_pins.snapshot()
            head_before = head.encoder.steps
            lazy_before = lazy.encoder.steps
            if ch == "l":
                print(f"\nNudging lazy susan at {NUDGE_SPEED:+.2f}")
                nudge(lazy, +1.0)
            elif ch == "L":
                print(f"\nNudging lazy susan at {-NUDGE_SPEED:+.2f}")
                nudge(lazy, -1.0)
            elif ch == "h":
                print(f"\nNudging head at {HEAD_NUDGE_SPEED:+.2f}")
                nudge_head(head, +1.0)
            elif ch == "H":
                print(f"\nNudging head at {-HEAD_NUDGE_SPEED:+.2f}")
                nudge_head(head, -1.0)

            time.sleep(NUDGE_TIME_S)
            lazy.value = 0.0
            head.value = 0.0
            print_status()
            print(
                f"delta: lazy_steps={lazy.encoder.steps - lazy_before:+d} "
                f"head_steps={head.encoder.steps - head_before:+d}"
            )
            print_raw_delta(raw_before, raw_pins.snapshot())

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    lazy.value = 0.0
    head.value = 0.0
    lazy.deactivate_and_save()
    head.deactivate_and_save()
    raw_pins.close()
    factory.close()
    time.sleep(0.2)
    mosfet.off()
