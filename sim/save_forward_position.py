"""
Save the current absolute physical position of the lazy susan and head as the
shared startup home pose.

Run this with both servos physically facing forward before running the head
balance controller. The balance controller will return to this saved absolute
pose on every startup.
"""

import json
import os
import time
from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from ServoEx import ServoEx

MOSFET_PIN    = 16
LAZY_SERVO    = 12
LAZY_A, LAZY_B, LAZY_ABS = 26, 6, 5
HEAD_SERVO    = 20
HEAD_A, HEAD_B, HEAD_ABS = 4, 22, 17

HOME_FILE = "servo_home_absolute.json"

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

factory = PiGPIOFactory()

print("Initializing lazy susan encoder...")
lazy = ServoEx(LAZY_SERVO, LAZY_A, LAZY_B, LAZY_ABS, initial_value=None, pin_factory=factory)
print("Initializing head encoder...")
head = ServoEx(HEAD_SERVO, HEAD_A, HEAD_B, HEAD_ABS, initial_value=None, pin_factory=factory)

for _ in range(5):
	lazy.update()
	head.update()
	time.sleep(0.05)

lazy_home_deg = lazy.get_absolute_position() * 360.0
head_home_deg = head.get_absolute_position() * 360.0

home_data = {
	"lazy_susan_deg": lazy_home_deg,
	"head_deg": head_home_deg,
}

with open(HOME_FILE, "w") as file_handle:
	json.dump(home_data, file_handle, indent=2)

print(f"Saved absolute home pose to {HOME_FILE}")
print(f"  lazy susan (pin {LAZY_SERVO}) = {lazy_home_deg:.1f}°")
print(f"  head       (pin {HEAD_SERVO}) = {head_home_deg:.1f}°")

lazy.value = None
head.value = None
time.sleep(0.2)
mosfet.off()
print("Done.")
