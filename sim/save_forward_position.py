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

lazy_abs = lazy.get_absolute_position()
head_abs = head.get_absolute_position()
lazy_home_deg = lazy_abs * 360.0
head_home_deg = head_abs * 360.0
lazy_relative_deg = lazy.get_position() * 360.0
head_relative_deg = head.get_position() * 360.0

if lazy_abs in (0.0, 1.0):
	print(f"WARNING: lazy absolute encoder appears saturated at {lazy_abs:.1f}")
if head_abs in (0.0, 1.0):
	print(f"WARNING: head absolute encoder appears saturated at {head_abs:.1f}")

home_data = {
	"lazy_susan_deg": lazy_home_deg,
	"head_deg": head_home_deg,
	"lazy_susan_steps": lazy.encoder.steps,
	"head_steps": head.encoder.steps,
}

with open(HOME_FILE, "w") as file_handle:
	json.dump(home_data, file_handle, indent=2)

print(f"Saved absolute home pose to {HOME_FILE}")
print(f"  lazy susan (pin {LAZY_SERVO}) = {lazy_home_deg:.1f}°")
print(f"  head       (pin {HEAD_SERVO}) = {head_home_deg:.1f}°")
print(f"  lazy raw absolute = {lazy_abs:.3f}, relative = {lazy_relative_deg:.1f}°")
print(f"  head raw absolute = {head_abs:.3f}, relative = {head_relative_deg:.1f}°")

lazy.value = None
head.value = None
time.sleep(0.2)
mosfet.off()
print("Done.")
