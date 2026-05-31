"""
Moves the 150kg arm servo instantly to the known vertical position (-0.66).
"""

import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

SERVO_PIN  = 13
MOSFET_PIN = 16
VERTICAL   = -0.66
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()
servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory)
time.sleep(0.5)

print(f"Moving to vertical ({VERTICAL})... (clamped to [{ARM_SERVO_MIN},{ARM_SERVO_MAX}])")
servo.value = max(min(VERTICAL, ARM_SERVO_MAX), ARM_SERVO_MIN)
print("Done. Press Ctrl+C to release.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    mosfet.off()
    print("Done.")
