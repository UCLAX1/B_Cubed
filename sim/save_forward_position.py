"""
Save the current physical position of the lazy susan and head as forward (0 degrees).
Run this with both servos physically facing forward before running head_balance_servo_control.py.
"""

import time
from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

MOSFET_PIN    = 16
LAZY_SERVO    = 12
LAZY_A, LAZY_B, LAZY_ABS = 26, 6, 5
HEAD_SERVO    = 18
HEAD_A, HEAD_B, HEAD_ABS = 4, 22, 17

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()

print("Initializing lazy susan encoder...")
lazy = ServoEx(LAZY_SERVO, LAZY_A, LAZY_B, LAZY_ABS, initial_value=None, pin_factory=factory)
print("Initializing head encoder...")
head = ServoEx(HEAD_SERVO, HEAD_A, HEAD_B, HEAD_ABS, initial_value=None)

lazy.reset_encoder_position()
head.reset_encoder_position()

print(f"Saved: lazy susan (pin {LAZY_SERVO}) forward = 0 steps")
print(f"Saved: head       (pin {HEAD_SERVO}) forward = 0 steps")

lazy.value = None
head.value = None
time.sleep(0.2)
mosfet.off()
print("Done.")
