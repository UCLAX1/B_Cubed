from gpiozero import Servo
from time import sleep

# Define the servos using their BCM (GPIO) numbers, NOT physical pins!
# Adjust min_pulse_width and max_pulse_width if your servos don't spin fully
garosa_5kg = Servo(23)  # Continuous (Physical Pin 16)
diymall_70kg = Servo(12)  # Continuous (Physical Pin 32)
annimos_150kg = Servo(13)  # Standard (Physical Pin 33)

try:
    # print("Testing Standard Servo (150KG)")
    # annimos_150kg.value = 0 # Move to center position
    # sleep(2)
    
    print("Testing Continuous Servo (70KG)")
    diymall_70kg.value = 0.1 # Spin forward at 50% speed
    sleep(2)
    diymall_70kg.value = 0 # Stop
    
    # print("Testing Continuous Servo (5.5KG)")
    # garosa_5kg.value = 0.1 # Spin in reverse at 100% speed
    # sleep(2)
    # garosa_5kg.value = 0 # Stop
    
except KeyboardInterrupt:
    print("Program stopped.")