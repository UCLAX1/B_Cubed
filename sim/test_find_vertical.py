from head_balance_math import find_motor_angles
import numpy as np

# Test cases: (pitch, roll) in degrees
# These are chosen to check when the arm should be vertical (0 deg, 20 deg, -20 deg, 90 deg, etc)
tests = [
    (0, 0),        # perfectly upright
    (20, 0),       # 20 deg pitch forward
    (-20, 0),      # 20 deg pitch backward
    (0, 20),       # 20 deg roll right
    (0, -20),      # 20 deg roll left
    (90, 0),       # pitch fully forward
    (0, 90),       # roll fully right
    (45, 45),      # diagonal
    (0, 180),      # upside down roll
    (180, 0),      # upside down pitch
]

print("pitch, roll  =>  arm, lazy_susan")
for pitch, roll in tests:
    arm, lazy_susan = find_motor_angles(pitch, roll)
    print(f"{pitch:>5}, {roll:>5}  =>  {arm:>7.2f}, {lazy_susan:>10.2f}")
