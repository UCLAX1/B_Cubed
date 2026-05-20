"""
Slowly moves the 150kg arm servo to the known vertical position (-0.66).
Starts from center (0.0) and ramps there in tiny steps.
"""

import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

SERVO_PIN   = 13
MOSFET_PIN  = 16
VERTICAL    = -0.66

STEP        = 0.005   # servo units per step
STEP_DELAY  = 0.05    # seconds between steps

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()
servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory)
time.sleep(0.5)

print(f"Moving slowly to vertical ({VERTICAL})...")
current = 0.0  # arm rests at ~0.0 when unpowered/forward

while current > VERTICAL:
    current = max(VERTICAL, current - STEP)
    servo.value = current
    print(f"servo = {current:+.4f}", flush=True)
    time.sleep(STEP_DELAY)

print(f"\nReached vertical ({VERTICAL}). Press Ctrl+C to release.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    mosfet.off()
    print("Done.")
