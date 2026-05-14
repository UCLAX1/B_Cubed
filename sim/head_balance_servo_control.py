"""
IMU Reader with Servo Control
Reads IMU data, calculates target motor angles, and moves servos to balance.
Uses gpiozero Servo for direct control (matches servo_sample.py).
"""

import sys
import time
import math
from gpiozero import Servo, DigitalOutputDevice
from head_balance_math import find_motor_angles

# ============================================================================
# DEBUG FLAG - Set to True to see print output, False to suppress
# ============================================================================
DEBUG = True # Change to False to silence all output

# ============================================================================
# IMU INITIALIZATION (egg.py style)
# ============================================================================
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU

settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)

if not imu.IMUInit():
    if DEBUG:
        print("IMU init failed")
    sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

imu_poll_interval = imu.IMUGetPollInterval() / 1000.0

if DEBUG:
    print(f"IMU initialized. Poll interval: {imu_poll_interval*1000:.1f}ms\n")

# ============================================================================
# SERVO INITIALIZATION (match servo_sample.py)
# ============================================================================
arm_servo = Servo(13, initial_value=None)           # annimos_150kg (Standard, Physical Pin 33)
lazy_susan_servo = Servo(12, initial_value=None)   # diymall_70kg (Continuous, Physical Pin 32)
head_servo = Servo(18, initial_value=None)         # garosa_5kg (Continuous, Physical Pin 12)
MOSFET = DigitalOutputDevice(16)                   # MOSFET control (Physical Pin 36)

# Angle limits (degrees) to prevent mechanical damage
ARM_MIN = -120
ARM_MAX = 120
LAZY_SUSAN_MIN = -90
LAZY_SUSAN_MAX = 90

def angle_to_servo_value(angle_deg, servo_type='standard'):
    """
    Convert target angle (degrees) to servo control value (-1 to 1).
    
    For standard servos (arm): -1 = full left, 0 = center, 1 = full right
    For continuous servos (lazy susan, head): -1 = full reverse, 0 = stop, 1 = full forward
    
    Args:
        angle_deg: Target angle in degrees
        servo_type: 'standard' or 'continuous'
    
    Returns:
        Servo value between -1 and 1
    """
    if servo_type == 'standard':
        # Standard servo: assume ±90° maps to ±1.0
        return max(min(angle_deg / 90.0, 1.0), -1.0)
    else:  # continuous
        # For continuous servos, we use the angle to determine speed/direction
        # Proportionally map angle to velocity
        # ULTRA-SLOW: max comfortable speed is 5°/sec (9x slower than normal 45°/sec)
        max_angle_speed = 5.0
        return max(min(angle_deg / max_angle_speed, 1.0), -1.0)


# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    try:
        if DEBUG:
            print("Turning MOSFET ON...")
        MOSFET.on()
        time.sleep(0.5)
        
        if DEBUG:
            print("Centering all servos...")
        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        time.sleep(1)
        
        if DEBUG:
            print("Reading IMU data and moving servos. Press Ctrl+C to stop.\n")
        
        last_print = time.time()
        last_servo_update = time.time()
        print_interval = 0.5     # Print every 500ms
        servo_update_interval = 0.5  # Update servos every 500ms (ultra-slow)
        
        while True:
            # Read IMU at its natural poll interval
            if imu.IMURead():
                current_time = time.time()
                
                # Print IMU and target data periodically
                if current_time - last_print >= print_interval:
                    data = imu.getIMUData()
                    fusionPose = data["fusionPose"]
                    
                    # Convert to degrees
                    roll = math.degrees(fusionPose[0])
                    pitch = math.degrees(fusionPose[1])
                    yaw = math.degrees(fusionPose[2])
                    
                    # Calculate target motor angles
                    arm_target, lazy_susan_target, head_target = find_motor_angles(pitch, roll, 0.0)
                    
                    if DEBUG:
                        print(f"IMU: Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°")
                        print(f"Targets: Arm={arm_target:7.2f}°  Lazy Susan={lazy_susan_target:7.2f}°  Head={head_target:7.2f}°")
                        print("-" * 80)
                    
                    last_print = current_time
                
                # Update servo positions at controlled rate
                if current_time - last_servo_update >= servo_update_interval:
                    data = imu.getIMUData()
                    fusionPose = data["fusionPose"]
                    
                    roll = math.degrees(fusionPose[0])
                    pitch = math.degrees(fusionPose[1])
                    
                    # Calculate target angles
                    arm_target, lazy_susan_target, head_target = find_motor_angles(pitch, roll, 0.0)
                    
                    # Clamp to safe limits
                    arm_target = max(min(arm_target, ARM_MAX), ARM_MIN)
                    lazy_susan_target = max(min(lazy_susan_target, LAZY_SUSAN_MAX), LAZY_SUSAN_MIN)
                    
                    # Convert angles to servo values and move
                    arm_servo.value = angle_to_servo_value(arm_target, 'standard')
                    lazy_susan_servo.value = angle_to_servo_value(lazy_susan_target, 'continuous')
                    head_servo.value = angle_to_servo_value(head_target, 'continuous')
                    
                    last_servo_update = current_time
            
            # Sleep for RTIMU's recommended poll interval
            time.sleep(imu_poll_interval)
    
    except KeyboardInterrupt:
        if DEBUG:
            print("\n\nStopping...")
    
    finally:
        if DEBUG:
            print("Centering servos...")
        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        time.sleep(0.5)
        
        if DEBUG:
            print("Turning MOSFET OFF...")
        MOSFET.off()
        if DEBUG:
            print("Done!")


if __name__ == "__main__":
    main()
