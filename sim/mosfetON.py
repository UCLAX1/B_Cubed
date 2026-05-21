import time
from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

MOSFET_PIN = 16

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
