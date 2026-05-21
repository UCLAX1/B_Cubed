"""
Temporary test: Use 70kg servo GPIO pin (BCM 12) to control 5.5kg servo physically plugged in.
Tests if the 70kg servo's GPIO signal source works by driving the head servo wired to it.
"""

import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

LAZY_SUSAN_PIN = 27  # 70kg servo GPIO - using this as signal source
MOSFET_PIN = 16
SPIN_SPEED = 0.3
DURATION = 2.0

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(3)

try:
    factory = PiGPIOFactory()
    servo = Servo(LAZY_SUSAN_PIN, initial_value=None, pin_factory=factory)
    print(f"Using 70kg GPIO pin (BCM {LAZY_SUSAN_PIN}) to control 5.5kg servo.")

    print(f"Spinning right at {SPIN_SPEED} for {DURATION}s...")
    servo.value = SPIN_SPEED
    time.sleep(DURATION)

    print(f"Spinning left at {-SPIN_SPEED} for {DURATION}s...")
    servo.value = -SPIN_SPEED
    time.sleep(DURATION)

    print("Stopping...")
    servo.value = 0.0
    time.sleep(0.5)

    print("If servo moved, BCM 12 (70kg GPIO) is working!")

finally:
    servo.value = None
    time.sleep(0.2)
    mosfet.off()
