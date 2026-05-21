import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

MOSFET_PIN = 16

mosfet = DigitalOutputDevice(MOSFET_PIN)

try:
    mosfet.on()
    print("turned on")

finally:
    print("not on :(")