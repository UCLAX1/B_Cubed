from gpiozero import Servo, DigitalOutputDevice, PWMOutputDevice
from time import sleep

PIN = 19

servo = Servo(PIN, initial_value=None)  # Continuous (Physical Pin 12)

MOSFET = DigitalOutputDevice(16)  # MOSFET control (Physical Pin 36)

try:
    print("Turning MOSFET ON (pin 36)")
    MOSFET.on()

    print("Testing Servo")
    servo.value = 0.1 # Move to center position
    sleep(20)

    print("Turning MOSFET OFF (pin 36)")
    MOSFET.off()

except KeyboardInterrupt:
    print("Program stopped.")
