"""
Read and print encoder values for lazy susan (BCM 12) and head (BCM 20).
"""

import time
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

factory = PiGPIOFactory()

print("Initializing encoders...")
lazy = ServoEx(12, 26, 6, 5, initial_value=None, pin_factory=factory)
head = ServoEx(20, 4, 22, 17, initial_value=None, pin_factory=factory)

print(f"Lazy susan encoder loaded: {lazy.encoder.steps} steps")
print(f"Head encoder loaded: {head.encoder.steps} steps\n")

LAZY_CPR = 1493
HEAD_CPR = 2048

print("Reading encoders (Ctrl+C to stop)...\n")
print("Time(s)  | Lazy(steps) | Lazy(deg) | Head(steps) | Head(deg)")
print("-" * 65)

start = time.time()

try:
    while True:
        elapsed = time.time() - start
        lazy_steps = lazy.encoder.steps
        lazy_deg = lazy_steps * 360.0 / LAZY_CPR
        head_steps = head.encoder.steps
        head_deg = head_steps * 360.0 / HEAD_CPR

        print(f"{elapsed:7.2f}  | {lazy_steps:11d} | {lazy_deg:9.2f} | {head_steps:11d} | {head_deg:9.2f}")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nDone.")
finally:
    lazy.deactivate_and_save()
    head.deactivate_and_save()
