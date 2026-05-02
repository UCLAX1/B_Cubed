from gpiozero import Servo, DigitalOutputDevice
from time import sleep

# Define the servos using their BCM (GPIO) numbers, NOT physical pins!
# Adjust min_pulse_width and max_pulse_width if your servos don't spin fully
garosa_5kg = Servo(18, initial_value=None)  # Continuous (Physical Pin 12)
diymall_70kg = Servo(12, initial_value=None)  # Continuous (Physical Pin 32)
annimos_150kg = Servo(13, initial_value=None)  # Standard (Physical Pin 33)
MOSFET = DigitalOutputDevice(16)  # MOSFET control (Physical Pin 36)

try:
    print("Turning MOSFET ON (pin 36)")
    MOSFET.on()

    # print("Testing Standard Servo (150KG)")
    # annimos_150kg.value = 0 # Move to center position
    # sleep(2)
    
    # print("Testing Continuous Servo (70KG)")
    # diymall_70kg.value = 0.1 # Spin forward at 50% speed
    # sleep(2)
    # diymall_70kg.value = 0 # Stop
    
    # print("Testing Continuous Servo (5.5KG)")
    # garosa_5kg.value = 0.1 # Spin in reverse at 100% speed
    # sleep(2)
    # garosa_5kg.value = 0 # Stop

    print("Turning MOSFET OFF (pin 36)")
    MOSFET.off()

    
except KeyboardInterrupt:
    print("Program stopped.")