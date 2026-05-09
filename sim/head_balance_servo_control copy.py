"""
Minimal IMU data reader - just get IMU angles and print them.
No servo control, no complexity.
"""

import sys
import time
import math
from head_balance_math import find_motor_angles

# ============================================================================
# IMU INITIALIZATION (egg.py style)
# ============================================================================
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU

settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)

if not imu.IMUInit():
    print("IMU init failed")
    sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

imu_poll_interval = imu.IMUGetPollInterval() / 1000.0

print(f"IMU initialized. Poll interval: {imu_poll_interval*1000:.1f}ms\n")


# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    try:
        print("Reading IMU data. Press Ctrl+C to stop.\n")
        
        last_print = time.time()
        print_interval = 0.2  # Print every 200ms
        
        while True:
            # Read IMU at its natural poll interval
            if imu.IMURead():
                current_time = time.time()
                
                # Print every 200ms to see real-time changes
                if current_time - last_print >= print_interval:
                    data = imu.getIMUData()
                    fusionPose = data["fusionPose"]
                    accel = data["accel"]
                    gyro = data["gyro"]
                    compass = data["compass"]
                    
                    # Convert to degrees
                    roll = math.degrees(fusionPose[0])
                    pitch = math.degrees(fusionPose[1])
                    yaw = math.degrees(fusionPose[2])
                    
                    # Calculate target motor angles
                    arm_target, lazy_susan_target, head_target = find_motor_angles(pitch, roll, 0.0)
                    
                    print(f"Orientation: Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°")
                    print(f"Target Angles: Arm={arm_target:7.2f}°  Lazy Susan={lazy_susan_target:7.2f}°  Head={head_target:7.2f}°")
                    
                    last_print = current_time
            
            # Sleep for RTIMU's recommended poll interval
            time.sleep(imu_poll_interval)
    
    except KeyboardInterrupt:
        print("\n\nStopped!")


if __name__ == "__main__":
    main()
