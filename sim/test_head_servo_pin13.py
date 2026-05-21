"""
Temporary test: Use 70kg servo GPIO pin (BCM 12) to control 5.5kg servo physically plugged in.
Tests if the 70kg servo's GPIO signal source works by driving the head servo wired to it.
"""

import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

SERVO_PIN = 18  # Physical pin 32 - previously working
MOSFET_PIN = 16
SPIN_SPEED = 0.3
DURATION = 10.0

mosfet = DigitalOutputDevice(MOSFET_PIN, pin_factory = PiGPIOFactory())
mosfet.on()
time.sleep(0.5)

try:
    factory = PiGPIOFactory()
    servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory)
    print(f"Using GPIO pin BCM {SERVO_PIN} to control servo.")
    print(f"Initial servo.value: {servo.value}")

    print(f"Spinning right at {SPIN_SPEED} for {DURATION}s...")
    servo.value = SPIN_SPEED
    print(f"  Set servo.value to {servo.value}")
    time.sleep(DURATION)

    print(f"Spinning left at {-SPIN_SPEED} for {DURATION}s...")
    servo.value = -SPIN_SPEED
    print(f"  Set servo.value to {servo.value}")
    time.sleep(DURATION)

    print("Stopping...")
    servo.value = 0.0
    print(f"  Set servo.value to {servo.value}")
    time.sleep(0.5)

    print(f"If servo moved, BCM {SERVO_PIN} is working!")

finally:
    servo.value = None
    time.sleep(0.2)
    mosfet.off()
