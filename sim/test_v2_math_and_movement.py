"""
Test head_balance_servo_control_v2 math and movement (no numpy required).
"""

import math

def mock_find_motor_angles(pitch, roll, desired_angle):
    """
    Simplified mock based on actual head_balance_math.py logic.
    Tests proportional control without full trigonometry.
    """
    pitch_rad = pitch * math.pi / 180
    roll_rad = roll * math.pi / 180
    desired_rad = desired_angle * math.pi / 180

    n1 = math.sin(pitch_rad) * math.cos(roll_rad)
    n2 = -math.sin(roll_rad)
    n3 = math.cos(pitch_rad) * math.cos(roll_rad)

    arm = -math.acos(n3) * 180 / math.pi

    if pitch < 0:
        arm = -arm

    if roll != 0:
        lazy_susan = math.acos(n1 / math.sqrt(n1**2 + n2**2)) * 180 / math.pi
    else:
        lazy_susan = 0

    if roll < 0:
        lazy_susan = -lazy_susan
        arm = -arm

    if abs(lazy_susan) > 90:
        arm = -arm
        if lazy_susan > 0:
            lazy_susan = lazy_susan - 180
        else:
            lazy_susan = lazy_susan + 180

    head = desired_angle - lazy_susan

    return (arm, lazy_susan, head)

LAZY_CPR = 1493
HEAD_CPR = 2048
CONT_MAX_ANGLE_SPEED = 0.5
ARM_VERTICAL_OFFSET = -0.66
ARM_SERVO_MIN = -0.5
ARM_SERVO_MAX = 0.5

def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)

def angle_to_servo_cmd_continuous(error_deg):
    """Convert error degrees to servo command (-1 to 1)"""
    return clamp(error_deg / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)

def angle_to_servo_cmd_arm(angle_deg):
    """Convert angle degrees to servo command with vertical offset"""
    return clamp((angle_deg / 90.0) + ARM_VERTICAL_OFFSET, ARM_SERVO_MIN, ARM_SERVO_MAX)

print("=" * 80)
print("PHASE 1: MATH VALIDATION")
print("=" * 80)

test_cases = [
    (0, 0, "No tilt - should be near zero"),
    (10, 0, "10° pitch forward - arm compensates"),
    (0, 10, "10° roll right - lazy susan rotates"),
    (10, 10, "10° forward and right tilt"),
    (45, 45, "Large tilt in both axes"),
    (-10, -10, "Backward and left tilt"),
]

print("\nIMU Input → Motor Target Angles\n")
all_pass = True
for pitch, roll, description in test_cases:
    arm_tgt, lazy_tgt, head_tgt = mock_find_motor_angles(pitch, roll, 0.0)
    print(f"{description}")
    print(f"  Input: pitch={pitch:+6.1f}° roll={roll:+6.1f}°")
    print(f"  Output: arm={arm_tgt:+7.2f}° lazy={lazy_tgt:+7.2f}° head={head_tgt:+7.2f}°")

    # Validation checks
    arm_ok = abs(arm_tgt) <= 30.0
    lazy_ok = abs(lazy_tgt) <= 90.0

    status_arm = "✓" if arm_ok else "✗"
    status_lazy = "✓" if lazy_ok else "✗"

    print(f"  {status_arm} Arm in range [-30, 30]: {arm_ok}")
    print(f"  {status_lazy} Lazy in range [-90, 90]: {lazy_ok}")

    all_pass = all_pass and arm_ok and lazy_ok
    print()

print("=" * 80)
print("PHASE 2: SERVO COMMAND CALCULATION")
print("=" * 80)

print("\nProportional Speed Control: error_degrees / CONT_MAX_ANGLE_SPEED\n")

error_tests = [
    (0, "At target - no movement"),
    (1, "1° error - slow movement"),
    (0.5, "0.5° error - slower movement"),
    (0.25, "0.25° error - very slow"),
    (5, "5° error - fast movement"),
    (10, "10° error - max speed (clamped)"),
]

cmd_pass = True
for error, description in error_tests:
    cmd = angle_to_servo_cmd_continuous(error)
    expected_speed = error / CONT_MAX_ANGLE_SPEED
    is_clamped = abs(expected_speed) > 1.0
    clamped_text = "CLAMPED" if is_clamped else "ok"

    print(f"{description}")
    print(f"  Error: {error:+6.2f}°  Raw: {expected_speed:+.3f}  Clamped: {cmd:+.3f}  {clamped_text}")

    # Movement verification
    if cmd == 0.0:
        print(f"  ✓ Servo stopped (at target)")
    elif abs(cmd) < 0.3:
        print(f"  ✓ Slow movement ({abs(cmd):.3f})")
    elif abs(cmd) < 0.7:
        print(f"  ✓ Medium movement ({abs(cmd):.3f})")
    else:
        print(f"  ✓ Fast movement ({abs(cmd):.3f})")

    # Validate command is in [-1, 1]
    if ARM_SERVO_MIN <= cmd <= ARM_SERVO_MAX:
        print(f"  ✓ Command in valid range [-1.0, 1.0]")
    else:
        print(f"  ✗ COMMAND OUT OF RANGE")
        cmd_pass = False
    print()

print("=" * 80)
print("PHASE 3: END-TO-END TEST")
print("=" * 80)

print("\nSimulated Robot Tilting → Servo Responses\n")

# Simulate a sequence where robot tilts forward over time
tilt_sequence = [
    (0, 0, "T=0s: Neutral"),
    (5, 0, "T=1s: Tilting forward 5°"),
    (10, 0, "T=2s: Tilting forward 10°"),
    (15, 5, "T=3s: Tilting forward 15° + right 5°"),
    (10, 0, "T=4s: Back to 10° forward"),
    (0, 0, "T=5s: Back to neutral"),
]

e2e_pass = True
for pitch, roll, description in tilt_sequence:
    print(f"{description}")
    print(f"  IMU: P={pitch:+6.1f}° R={roll:+6.1f}°", end="")

    arm_tgt, lazy_tgt, head_tgt = mock_find_motor_angles(pitch, roll, 0.0)

    arm_tgt_clamped = clamp(arm_tgt, -30.0, 30.0)
    lazy_tgt_clamped = clamp(lazy_tgt, -90.0, 90.0)

    # Simulate encoder at neutral (0 degrees)
    lazy_pos = 0.0
    head_pos = 0.0

    lazy_error = lazy_tgt_clamped - lazy_pos
    head_error = head_tgt - head_pos

    lazy_cmd = angle_to_servo_cmd_continuous(lazy_error)
    head_cmd = angle_to_servo_cmd_continuous(head_error)
    arm_cmd = angle_to_servo_cmd_arm(arm_tgt_clamped)

    print(f"  → Targets: arm={arm_tgt_clamped:+7.2f}° lazy={lazy_tgt_clamped:+7.2f}° head={head_tgt:+7.2f}°")
    print(f"    Commands: arm={arm_cmd:+.3f} lazy={lazy_cmd:+.3f} head={head_cmd:+.3f}")

    # Predict movement direction
    directions = []
    if lazy_cmd > 0.1:
        directions.append("lazy→CW")
    elif lazy_cmd < -0.1:
        directions.append("lazy→CCW")
    else:
        directions.append("lazy→STOP")

    if head_cmd > 0.1:
        directions.append("head→CW")
    elif head_cmd < -0.1:
        directions.append("head→CCW")
    else:
        directions.append("head→STOP")

    if arm_cmd > (ARM_VERTICAL_OFFSET + 0.05):
        directions.append("arm↑")
    elif arm_cmd < (ARM_VERTICAL_OFFSET - 0.05):
        directions.append("arm↓")
    else:
        directions.append("arm→neutral")

    print(f"    Movement: {' | '.join(directions)}")

    # Validate all commands in range
    if not ( -1.0 <= lazy_cmd <= 1.0 and -1.0 <= head_cmd <= 1.0 and ARM_SERVO_MIN <= arm_cmd <= ARM_SERVO_MAX):
        print(f"    ✗ COMMAND OUT OF RANGE")
        e2e_pass = False
    print()

print("=" * 80)
print("VALIDATION SUMMARY")
print("=" * 80)

if all_pass and cmd_pass and e2e_pass:
    print("""
✅ ALL TESTS PASSED

Phase 1 (Math): ✓ Angles within mechanical limits
Phase 2 (Commands): ✓ Servo commands valid and clamped correctly
Phase 3 (Flow): ✓ End-to-end simulation works

NEXT: Run actual head_balance_servo_control_v2.py with DEBUG=True:
  1. Check printed output matches predictions above
  2. Verify lazy susan rotates toward target
  3. Verify head rotates toward target
  4. Verify arm adjusts for pitch tilt
  5. Verify encoder readings make sense
""")
else:
    print("""
❌ SOME TESTS FAILED - CHECK ABOVE
""")
