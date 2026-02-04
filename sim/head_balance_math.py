import math 
import numpy as np

def find_motor_angles(pitch, roll):
    pitch = pitch * np.pi/180
    roll = roll * np.pi/180
    Lazy_Susan = 0
    Arm = 0
    
    n1 = math.sin(pitch) * math.cos(roll)
    n2 = -math.sin(roll)
    n3 = math.cos(pitch) * math.cos(roll)
    
    Arm = math.acos(n3) * 180/np.pi
    # Determine sign of Arm based on n1 (forward/backward direction)
    if n1 < 0:
        Arm = -Arm
    
    if roll != 0:
        Lazy_Susan = math.acos(n1/math.sqrt(n1**2 + n2**2)) * 180/np.pi
    
    if pitch > np.pi: 
        Arm = -Arm
    if roll > np.pi: 
        Lazy_Susan = - Lazy_Susan
    
    return(-Arm, Lazy_Susan)

