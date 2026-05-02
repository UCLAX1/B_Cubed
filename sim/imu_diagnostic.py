#!/usr/bin/env python3
"""
Comprehensive diagnostic for IMU zero-output issue
Checks all layers: I2C hardware, Sense HAT, RTIMU initialization, and data reading
"""

import sys
import os
import time

print("=" * 60)
print("B_CUBED IMU DIAGNOSTIC TOOL")
print("=" * 60)

# Step 1: Check RTIMU library availability
print("\n[1] CHECKING RTIMU LIBRARY...")
sys.path.append("/usr/lib/python3/dist-packages")
try:
    import RTIMU
    print("    ✓ RTIMU imported successfully")
except ImportError as e:
    print(f"    ✗ RTIMU import FAILED: {e}")
    sys.exit(1)

# Step 2: Check RTIMULib settings file
print("\n[2] CHECKING SETTINGS FILE...")
SETTINGS_FILE = "RTIMULib"
if os.path.exists(SETTINGS_FILE):
    print(f"    ✓ Settings file '{SETTINGS_FILE}' exists")
    with open(SETTINGS_FILE, 'r') as f:
        content = f.read()
        lines = len(content.split('\n'))
        print(f"    - File size: {len(content)} bytes ({lines} lines)")
        # Check if it has real config or is empty
        if len(content) < 100:
            print("    ⚠ WARNING: Settings file appears very small, may be incomplete")
        else:
            print("    - Settings file appears to have content")
else:
    print(f"    ✗ Settings file '{SETTINGS_FILE}' NOT FOUND")
    print("    - CRITICAL: RTIMU needs this file in working directory")
    print("    - Current directory:", os.getcwd())
    print("    - Files in current directory:")
    for f in os.listdir('.'):
        print(f"      - {f}")

# Step 3: Try RTIMU Settings initialization
print("\n[3] INITIALIZING RTIMU SETTINGS...")
try:
    settings = RTIMU.Settings(SETTINGS_FILE)
    print("    ✓ RTIMU.Settings object created")
    print(f"    - Settings file used: {SETTINGS_FILE}")
except Exception as e:
    print(f"    ✗ Settings initialization FAILED: {e}")
    sys.exit(1)

# Step 4: Try RTIMU IMU object creation
print("\n[4] CREATING RTIMU IMU OBJECT...")
try:
    imu = RTIMU.RTIMU(settings)
    print("    ✓ RTIMU.RTIMU object created")
except Exception as e:
    print(f"    ✗ IMU object creation FAILED: {e}")
    sys.exit(1)

# Step 5: Try IMU Initialization
print("\n[5] INITIALIZING IMU HARDWARE...")
init_result = imu.IMUInit()
print(f"    - IMUInit() returned: {init_result}")
if not init_result:
    print("    ✗ IMU INITIALIZATION FAILED")
    print("    - This suggests:")
    print("      • Sense HAT not detected on I2C bus")
    print("      • I2C interface not enabled on Raspberry Pi")
    print("      • Hardware connection issue")
    print("      • Wrong settings file/calibration")
else:
    print("    ✓ IMU initialized successfully")

# Step 6: Enable sensors
print("\n[6] ENABLING SENSORS...")
try:
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)
    print("    ✓ All sensors enabled")
except Exception as e:
    print(f"    ✗ Sensor enable FAILED: {e}")

# Step 7: Get poll interval
print("\n[7] CHECKING POLL INTERVAL...")
try:
    poll_interval_ms = imu.IMUGetPollInterval()
    poll_interval_s = poll_interval_ms / 1000.0
    print(f"    ✓ Poll interval: {poll_interval_ms}ms ({poll_interval_s}s)")
except Exception as e:
    print(f"    ✗ Poll interval check FAILED: {e}")

# Step 8: Attempt to read IMU data
print("\n[8] ATTEMPTING DATA READS...")
print("    Reading 10 samples...")

zero_count = 0
for i in range(10):
    read_result = imu.IMURead()
    print(f"    [{i+1}] IMURead() returned: {read_result}", end="")
    
    if read_result:
        try:
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
            accel = data["accel"]
            gyro = data["gyro"]
            compass = data["compass"]
            
            print(f" | Pose: ({fusionPose[0]:.4f}, {fusionPose[1]:.4f}, {fusionPose[2]:.4f})")
            print(f"         | Accel: {accel} | Gyro: {gyro}")
            
            # Check if all zeros
            if (fusionPose == (0, 0, 0) and 
                accel == [0.0, 0.0, 0.0] and 
                gyro == [0.0, 0.0, 0.0]):
                zero_count += 1
        except Exception as e:
            print(f" ✗ getIMUData() FAILED: {e}")
    else:
        print(" | No data available")
    
    time.sleep(poll_interval_s)

print(f"\n    ⚠ Zero samples: {zero_count}/10")

# Summary
print("\n" + "=" * 60)
print("DIAGNOSTIC SUMMARY")
print("=" * 60)

if init_result and zero_count == 10:
    print("⚠ CRITICAL FINDING:")
    print("  - IMU initialized successfully (hardware IS detected)")
    print("  - BUT all sensor reads return zeros")
    print("  - LIKELY CAUSES:")
    print("    1. Calibration data missing/incorrect in RTIMULib settings")
    print("    2. Sensor hardware not responding")
    print("    3. Sense HAT mounted incorrectly")
    print("  - NEXT STEPS:")
    print("    • Check physical Sense HAT connection")
    print("    • Try to regenerate RTIMULib calibration")
    print("    • Test with direct I2C tools")
elif not init_result:
    print("✗ HARDWARE DETECTION ISSUE:")
    print("  - IMU initialization failed")
    print("  - LIKELY CAUSES:")
    print("    1. I2C interface not enabled (raspi-config)")
    print("    2. Sense HAT not physically installed")
    print("    3. I2C device not responding")
    print("  - DIAGNOSTIC COMMANDS TO RUN:")
    print("    • i2cdetect -y 1  # Check I2C bus")
    print("    • lsmod | grep sense  # Check drivers")
    print("    • ls -la /dev/i2c*  # Check I2C devices")
else:
    print("✓ IMU appears to be working correctly")

print("=" * 60)
