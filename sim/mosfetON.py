import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

MOSFET_PIN = 16

mosfet = DigitalOutputDevice(MOSFET_PIN, pin_factory = PiGPIOFactory())

try:
    mosfet.on()
    print("turned on")

except:
    print("not on :(")