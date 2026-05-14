"""
test_arm_range.py

Test the range of motion of the 150kg arm servo (annimos).
Moves slowly (1 second per step) through full range, showing encoder position.
Uses ServoEx for encoder feedback.
"""

import sys
import time
from gpiozero import DigitalOutputDevice

sys.path.append("/usr/lib/python3/dist-packages")
from ServoEx import ServoEx

# ============================================================================
# SERVO INITIALIZATION (150kg arm on GPIO 13)
# ============================================================================

print("Initializing 150kg arm servo...")
try:
    # physical pins: A=37, B=31, ABS=29 -> BCM: A=26, B=6, ABS=5
    arm_servo = ServoEx(servo_pin=13, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)
except Exception as e:
    print(f"ERROR initializing arm servo: {e}")
    sys.exit(1)

MOSFET = DigitalOutputDevice(16)  # MOSFET control

print("Turning MOSFET ON...")
MOSFET.on()
time.sleep(0.5)

print("Centering arm...")
arm_servo.value = 0
time.sleep(1)

# ============================================================================
# TEST RANGE OF MOTION
# ============================================================================

print("\n" + "=" * 80)
print("TESTING ARM RANGE OF MOTION (150kg servo)")
print("=" * 80)
print("\nMoving 1 step every 1 second from full reverse to full forward.\n")

try:
    # Test points: -1.0 (full reverse) to 1.0 (full forward) in 0.1 increments
    test_points = [-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 
                   0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    
    for step_num, servo_value in enumerate(test_points, 1):
        print(f"Step {step_num:2d}/21: servo.value = {servo_value:5.1f}", end=" | ")
        
        # Set servo
        arm_servo.value = servo_value
        
        # Wait 1 second for servo to settle
        time.sleep(1.0)
        
        # Update encoder readings
        arm_servo.update()
        
        # Get position
        encoder_pos = arm_servo.get_position()  # in rotations (0-1)
        absolute_pos = arm_servo.get_absolute_position()
        
        # Convert to degrees (assuming 360° per rotation)
        encoder_deg = encoder_pos * 360.0
        absolute_deg = absolute_pos * 360.0
        
        print(f"Encoder: {encoder_deg:7.1f}°  |  Absolute: {absolute_deg:7.1f}°")
    
    print("\n" + "=" * 80)
    print("TEST COMPLETE")
    print("=" * 80)
    print("\nNow testing SLOW oscillation (±0.5 servo value)...")
    print("Press Ctrl+C to stop.\n")
    
    # Oscillate at ±0.5 servo value
    step_count = 0
    while True:
        for servo_value in [0.5, -0.5]:
            step_count += 1
            print(f"Step {step_count}: servo.value = {servo_value:5.1f}", end=" | ")
            
            arm_servo.value = servo_value
            time.sleep(1.0)
            
            arm_servo.update()
            encoder_pos = arm_servo.get_position()
            absolute_pos = arm_servo.get_absolute_position()
            
            encoder_deg = encoder_pos * 360.0
            absolute_deg = absolute_pos * 360.0
            
            print(f"Encoder: {encoder_deg:7.1f}°  |  Absolute: {absolute_deg:7.1f}°")

except KeyboardInterrupt:
    print("\n\nStopping test...")

finally:
    print("Centering arm...")
    arm_servo.value = 0
    time.sleep(0.5)
    
    print("Turning MOSFET OFF...")
    MOSFET.off()
    print("Done!")
