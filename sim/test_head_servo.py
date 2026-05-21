"""
Simple test to verify the 5.5kg head servo (BCM 18) responds to commands.
Spins left and right without using encoders.
"""

import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

SERVO_PIN  = 18
MOSFET_PIN = 16
SPIN_SPEED = 0.3
DURATION   = 2.0

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

try:
    factory = PiGPIOFactory()
    servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory)
    print(f"Head servo (BCM {SERVO_PIN}) initialized.")

    print(f"Spinning right at {SPIN_SPEED} for {DURATION}s...")
    servo.value = SPIN_SPEED
    time.sleep(DURATION)

    print(f"Spinning left at {-SPIN_SPEED} for {DURATION}s...")
    servo.value = -SPIN_SPEED
    time.sleep(DURATION)

    print("Stopping...")
    servo.value = 0.0
    time.sleep(0.5)

    print("Head servo test complete. If servo moved, it works.")

finally:
    servo.value = None
    time.sleep(0.2)
    mosfet.off()
