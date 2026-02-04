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
        # unsigned angle between x-axis and projection (0..180)
        raw = math.degrees(math.acos(n1/math.sqrt(n1**2 + n2**2)))
        # fold into acute angle to avoid 180° flips when pitch sign changes
        if raw > 90:
            raw = 180 - raw
        # sign convention: if pitch is near zero, use roll direction; otherwise use pitch sign
        if abs(math.sin(pitch)) < 1e-8:
            sign = -1 if n2 < 0 else 1
        else:
            sign = 1 if math.sin(pitch) > 0 else -1
        Lazy_Susan = sign * raw
    else:
        Lazy_Susan = 0.0
    if pitch > np.pi: 
        Arm = -Arm
    if roll > np.pi: 
        Lazy_Susan = - Lazy_Susan
    
    return(Arm, Lazy_Susan)

