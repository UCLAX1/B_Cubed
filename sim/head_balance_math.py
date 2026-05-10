import math 
import numpy as np

def find_motor_angles(pitch, roll, desired_angle):
    pitch = pitch * np.pi/180
    roll = roll * np.pi/180
    desired_angle = desired_angle * np.pi/180
    Lazy_Susan = 0
    Arm = 0
    
    n1 = math.sin(pitch) * math.cos(roll)
    n2 = -math.sin(roll)
    n3 = math.cos(pitch) * math.cos(roll)
    
    Arm = -math.acos(n3) * 180/np.pi
    
    if pitch < 0:
        Arm = -Arm
    
    if roll != 0:
        Lazy_Susan = math.acos(n1/math.sqrt(n1**2 + n2**2)) * 180/np.pi
    
    if roll < 0:
        Lazy_Susan = -Lazy_Susan
        Arm = -Arm

    if (abs(Lazy_Susan) > 90):
        Arm = -Arm
        if(Lazy_Susan > 0):
            Lazy_Susan = Lazy_Susan - 180
        else:
            Lazy_Susan = Lazy_Susan + 180

    head = desired_angle - Lazy_Susan

    print("Motor Angles:" , Arm, Lazy_Susan, head)

    return(Arm, Lazy_Susan, head)

