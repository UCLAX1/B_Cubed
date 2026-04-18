import time
from sense_hat import SenseHat

sense = SenseHat()

print("Pressure, Temp, Roll, Pitch, Yaw")

while True:
    # Get orientation in degrees
    orientation = sense.get_orientation()
    pitch = orientation['pitch']
    roll = orientation['roll']
    yaw = orientation['yaw']
    
    # Print to terminal (your computer will see this via SSH)
    print(f"P: {pitch:.2f}, R: {roll:.2f}, Y: {yaw:.2f}")
    
    time.sleep(0.1)
