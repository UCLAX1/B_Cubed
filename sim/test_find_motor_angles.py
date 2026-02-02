from head_balance_math import find_motor_angles

# Test cases: (pitch, roll) in degrees
inputs = [
    (0, 0),
    (10, 0),
    (0, 10),
    (10, 10),
    (45, 0),
    (0, 45),
    (45, 45),
    (90, 0),
    (0, 90),
    (90, 90),
    (-10, 0),
    (0, -10),
    (-10, -10),
    (180, 0),
    (0, 180),
]

for pitch, roll in inputs:
    arm, lazy_susan = find_motor_angles(pitch, roll)
    print(f"Input: pitch={pitch}°, roll={roll}°  =>  arm={arm:.2f}°, lazy_susan={lazy_susan:.2f}°")
