"""Continuously print lazy susan encoder step count and derived angle."""

import time
from gpiozero import RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory

LAZY_CPR = 2048

factory = PiGPIOFactory()
encoder = RotaryEncoder(a=26, b=6, max_steps=10_000_000, pin_factory=factory)

print("Monitoring lazy susan encoder (BCM26/6). Ctrl-C to stop.\n")
print(f"{'steps':>10}  {'angle_deg':>10}")

try:
    while True:
        steps = encoder.steps
        angle = steps * 360.0 / LAZY_CPR
        print(f"{steps:>10d}  {angle:>+10.2f}°", flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nDone.")
