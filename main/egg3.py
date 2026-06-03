import sys, os, time
from gpiozero import Servo, DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import numpy as np
from CRServoEx import CRServoEx
from ServoEx import ServoEx

# input angles in radians
def find_motor_angles(pitch, roll, desired_angle):
    lazy_susan = 0
    arm = 0

    THRESHOLD = np.deg2rad(2.5)
    # if abs(roll) < THRESHOLD:
    if abs(roll) < THRESHOLD and abs(pitch) < THRESHOLD:
        roll = 0
        pitch = 0

    # n1 = np.sin(pitch)
    # n2 = -np.cos(pitch) * np.sin(roll)
    # n3 = np.cos(pitch) * np.cos(roll)
    
    n1 = np.sin(pitch) * np.cos(roll)
    n2 = -np.sin(roll)
    n3 = np.cos(pitch) * np.cos(roll)
    
    arm = -np.arccos(n3)
    
    if pitch > 0:
        arm = -arm
    
    if roll != 0:
        lazy_susan = np.arccos(n1/np.sqrt(n1**2 + n2**2))
    
    if roll > 0:
        lazy_susan = -lazy_susan


    if (abs(lazy_susan) > np.pi / 2):
        # arm = -arm
        if(lazy_susan > 0):
            lazy_susan = lazy_susan - np.pi
        else:
            lazy_susan = lazy_susan + np.pi

    head = desired_angle - lazy_susan

    arm = np.clip(arm, np.deg2rad(-30), np.deg2rad(30))

    # print("Motor Angles:" , arm, lazy_susan, head)


    return(arm, lazy_susan, head)

# Define the servos using their BCM (GPIO) numbers, NOT physical pins!
# Adjust min_pulse_width and max_pulse_width if your servos don't spin fully
# garosa_5kg = Servo(18, initial_value=None)  # Continuous (Physical Pin 16)
# diymall_70kg = Servo(12, initial_value=None)  # Continuous (Physical Pin 32)
# annimos_150kg = Servo(13, initial_value=None)  # Standard (Physical Pin 33)
mosfet = DigitalOutputDevice(16)  # mosfet control (Physical Pin 36, if supported)

# RTIMULib needs a settings file
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

poll_interval = imu.IMUGetPollInterval() / 1000.0

lazy_susan = CRServoEx(12, 4, 22, 17, kP=4.0, kI=0.01, kD=0.9)
arm_servo = ServoEx(15, 270, 0.5) # can only go from -30 to 30

mosfet.on()

start_time = time.time()
current_time = start_time
previous_time = current_time
time_elapsed = current_time - previous_time

current_pos = 0
previous_pos = 0

try:
    while True:
        current_time = time.time()
        dt = current_time - previous_time
        time_elapsed = current_time - start_time
        previous_time = current_time

        if imu.IMURead():
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
            # accel = data["accel"]
            # gyro = data["gyro"]
            # compass = data["compass"]

        arm_angle, lazy_susan_angle, head_angle = find_motor_angles(fusionPose[1], fusionPose[0], 0.0)

        print(f"Orientation: R={np.rad2deg(fusionPose[0]):6.1f} P={np.rad2deg(fusionPose[1]):6.1f} Y={np.rad2deg(fusionPose[2]):6.1f}, arm: {np.rad2deg(arm_angle):10.2f}")
        
        # print(f"lazy: {np.rad2deg(lazy_susan_angle):10.2f}")
        # print(f"arm: {np.rad2deg(arm_angle):10.2f}")

        arm_val = arm_angle / np.deg2rad(30)


        lazy_susan_val = lazy_susan_angle / (2.0 * np.pi)
        lazy_susan_val = (lazy_susan_val + np.pi) % (2.0 * np.pi) - np.pi
    
        lazy_susan.set_position(lazy_susan_val)
        arm_servo.set_position(arm_val)
        # current_pos = lazy_susan.get_position()
        if current_pos != previous_pos:
            pass
            # print(f"{current_pos:10.8f}")
        previous_pos = current_pos


        lazy_susan.update(dt)
except KeyboardInterrupt as e:
    print("keyboard interrupt")
    print(e)
except Exception as e:
    print("exception")
    print(e)
finally:
    mosfet.off()
    exit(1)

