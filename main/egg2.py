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
    
    n1 = np.sin(pitch) * np.cos(roll)
    n2 = -np.sin(roll)
    n3 = np.cos(pitch) * np.cos(roll)
    
    arm = -math.degrees(math.acos(n3))
    
    if pitch < 0:
        arm = -arm
    
    if roll != 0:
        lazy_susan = np.acos(n1/math.sqrt(n1**2 + n2**2))
    
    if roll < 0:
        lazy_susan = -lazy_susan
        arm = -arm

    if (abs(lazy_susan) > np.pi / 2):
        arm = -arm
        if(lazy_susan > 0):
            lazy_susan = lazy_susan - np.pi
        else:
            lazy_susan = lazy_susan + np.pi

    head = desired_angle - lazy_susan

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

lazy_susan = CRServoEx(12, 4, 22, 17)
# lazy_susan = Servo(12, pin_factory=PiGPIOFactory())

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

            # print(f"Orientation: R={np.rad2deg(fusionPose[0]):6.1f} P={np.rad2deg(fusionPose[1]):6.1f} Y={np.rad2deg(fusionPose[2]):6.1f}")
        
        lazy_susan.set_position(0.5)
        # lazy_susan.set_velocity(0.2)
        # lazy_susan.set_velocity(0.15)
        # print(f"{lazy_susan.absolute_encoder.current_raw_position*1000:.8f}")
        current_pos = lazy_susan.absolute_encoder.position
        if current_pos != previous_pos:
            pass
            # print(f"{lazy_susan.absolute_encoder.position*4096:10.8f}")
        previous_pos = current_pos

        # lazy_susan.set_position(0.0)
        # print(lazy_susan.get_absolute_position())
        # if time_elapsed <= 1.0:
        #     lazy_susan.set_velocity(0.2)
        # elif time_elapsed <= 2.0:
        #     lazy_susan.set_velocity(-0.2)
        # elif time_elapsed <= 3.0:
        #     break

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

