"""
IMU Reader with Servo Control (smoothed, no PID)
"""

import sys
import time
import math
from gpiozero import Servo, DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory  # hardware-timed PWM
from head_balance_math import find_motor_angles
from servo_control_filters import PDCommandDamper, angle_to_servo_value, clamp, slew_limit

DEBUG = True

# ============================================================================
# IMU INIT
# ============================================================================
SETTINGS_FILE = "RTIMULib"
sys.path.append("/usr/lib/python3/dist-packages")
import RTIMU

settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)
if not imu.IMUInit():
    if DEBUG: print("IMU init failed")
    sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
imu_poll_interval = imu.IMUGetPollInterval() / 1000.0

# ============================================================================
# SERVO INIT  -- use pigpio factory to remove software-PWM jitter
# ============================================================================
# Requires:  sudo systemctl enable --now pigpiod
try:
    pin_factory = PiGPIOFactory()
except Exception as e:
    if DEBUG: print(f"pigpio not available ({e}); falling back to default. "
                    "Run `sudo systemctl start pigpiod` to reduce jitter.")
    pin_factory = None

arm_servo        = Servo(13, initial_value=None, pin_factory=pin_factory)
lazy_susan_servo = Servo(12, initial_value=None, pin_factory=pin_factory)
head_servo       = Servo(18, initial_value=None, pin_factory=pin_factory)
MOSFET           = DigitalOutputDevice(16)

ARM_MIN, ARM_MAX = -120, 120
LAZY_SUSAN_MIN, LAZY_SUSAN_MAX = -90, 90
<<<<<<< Updated upstream
=======
HEAD_MIN, HEAD_MAX = -90, 90

# Servo range: assume ±90° maps to ±1.0 on the gpiozero Servo value.
# If your servos have a wider/narrower range, adjust SERVO_RANGE_DEG accordingly.
SERVO_RANGE_DEG = 90.0

# Slew-rate limit: max change in servo value per second (-1..1 range).
# 2.0 = full sweep in 1 second. Lower = smoother but slower to respond.
SERVO_SLEW_RATE = 2.0

# Arm jitter damping. This is open-loop because this script does not read the
# arm encoder; the controller damps the commanded angle near the setpoint.
ARM_PD_KP = 8.0
ARM_PD_KD = 0.35
ARM_PD_MAX_VELOCITY_DEG_S = 160.0
ARM_PD_DEADBAND_DEG = 1.0
ARM_TARGET_NOISE_DEG = 0.35
ARM_TARGET_NOISE_RATE_DEG_S = 12.0

>>>>>>> Stashed changes

# ============================================================================
# SMOOTHING / DEADBAND / SLEW PARAMETERS  (tune these)
# ============================================================================
# Exponential moving average on the IMU angles. Lower = smoother but laggier.
IMU_ALPHA = 0.15

# Deadband: ignore target changes smaller than this (degrees). Kills micro-buzz.
ARM_DEADBAND_DEG     = 1.0
CONT_DEADBAND_DEG    = 2.0   # continuous servos need a bigger deadband

# Slew limit on the commanded servo value (-1..1 units per second).
# Smaller = smoother, slower response.
ARM_SLEW_PER_SEC     = 0.8
CONT_SLEW_PER_SEC    = 0.6

# Continuous-servo speed scaling. Bigger denominator = gentler response.
CONT_MAX_ANGLE_SPEED = 15.0   # was 5.0 -- 5 was so small everything saturated

def clamp(x, lo, hi): return max(lo, min(hi, x))

def angle_to_servo_value(angle_deg, servo_type='standard'):
    if servo_type == 'standard':
        return clamp(angle_deg / 90.0, -1.0, 1.0)
    return clamp(angle_deg / CONT_MAX_ANGLE_SPEED, -1.0, 1.0)

# ============================================================================
# MAIN
# ============================================================================
def main():
<<<<<<< Updated upstream
    # Filter state
    roll_f = pitch_f = 0.0
    have_filter = False

    # Last commanded servo values (for slew limiting + deadband comparison)
    arm_cmd_last  = 0.0
    lazy_cmd_last = 0.0
    head_cmd_last = 0.0

    # Last target angles (for deadband on the *angle*, not the servo value)
    arm_tgt_last  = 0.0
    lazy_tgt_last = 0.0
    head_tgt_last = 0.0
=======
    # Track last commanded servo values for slew limiting
    arm_cmd = 0.0
    lazy_susan_cmd = 0.0
    head_cmd = 0.0
    arm_damper = PDCommandDamper(
        initial_angle=0.0,
        kp=ARM_PD_KP,
        kd=ARM_PD_KD,
        max_velocity_deg_s=ARM_PD_MAX_VELOCITY_DEG_S,
        deadband_deg=ARM_PD_DEADBAND_DEG,
        target_noise_deg=ARM_TARGET_NOISE_DEG,
        target_noise_rate_deg_s=ARM_TARGET_NOISE_RATE_DEG_S,
    )
>>>>>>> Stashed changes

    try:
        if DEBUG: print("MOSFET on...")
        MOSFET.on()
        time.sleep(0.5)

        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        arm_damper.reset(0.0)
        time.sleep(1)

        last_print = time.time()
        last_servo_update = time.time()
        print_interval = 0.5
        # Faster updates make slew limiting work properly. 50 Hz is plenty.
        servo_update_interval = 0.00625

        while True:
            if imu.IMURead():
                now = time.time()
                data = imu.getIMUData()
                fp = data["fusionPose"]
                roll_raw  = math.degrees(fp[0])
                pitch_raw = math.degrees(fp[1])
                yaw_raw   = math.degrees(fp[2])

                # --- Low-pass filter the IMU angles ---
                if not have_filter:
                    roll_f, pitch_f = roll_raw, pitch_raw
                    have_filter = True
                else:
                    roll_f  += IMU_ALPHA * (roll_raw  - roll_f)
                    pitch_f += IMU_ALPHA * (pitch_raw - pitch_f)

                # --- Servo update ---
                if now - last_servo_update >= servo_update_interval:
                    dt = now - last_servo_update
                    last_servo_update = now

<<<<<<< Updated upstream
                    arm_tgt, lazy_tgt, head_tgt = find_motor_angles(pitch_f, roll_f, 0.0)
=======
                # Clamp to safe mechanical limits
                arm_target = clamp(arm_target, ARM_MIN, ARM_MAX)
                lazy_susan_target = max(
                    min(lazy_susan_target, LAZY_SUSAN_MAX), LAZY_SUSAN_MIN
                )
                head_target = clamp(head_target, HEAD_MIN, HEAD_MAX)
>>>>>>> Stashed changes

                    # Deadband on target angle: hold previous target if change is tiny.
                    if abs(arm_tgt  - arm_tgt_last)  < ARM_DEADBAND_DEG:  arm_tgt  = arm_tgt_last
                    if abs(lazy_tgt - lazy_tgt_last) < CONT_DEADBAND_DEG: lazy_tgt = lazy_tgt_last
                    if abs(head_tgt - head_tgt_last) < CONT_DEADBAND_DEG: head_tgt = head_tgt_last
                    arm_tgt_last, lazy_tgt_last, head_tgt_last = arm_tgt, lazy_tgt, head_tgt

<<<<<<< Updated upstream
                    # Clamp
                    arm_tgt  = clamp(arm_tgt,  ARM_MIN, ARM_MAX)
                    lazy_tgt = clamp(lazy_tgt, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX)
=======
                # Update servos at controlled rate with slew limiting
                dt = current_time - last_servo_update
                if dt >= servo_update_interval:
                    damped_arm_target = arm_damper.update(arm_target, dt)

                    arm_raw = angle_to_servo_value(
                        damped_arm_target, SERVO_RANGE_DEG
                    )
                    lazy_raw = angle_to_servo_value(
                        lazy_susan_target, SERVO_RANGE_DEG
                    )
                    head_raw = angle_to_servo_value(head_target, SERVO_RANGE_DEG)
>>>>>>> Stashed changes

                    # Convert to servo command
                    arm_cmd  = angle_to_servo_value(arm_tgt,  'standard')
                    lazy_cmd = angle_to_servo_value(lazy_tgt, 'continuous')
                    head_cmd = angle_to_servo_value(head_tgt, 'continuous')

                    # Slew-limit each command
                    arm_max_step  = ARM_SLEW_PER_SEC  * dt
                    cont_max_step = CONT_SLEW_PER_SEC * dt
                    arm_cmd  = arm_cmd_last  + clamp(arm_cmd  - arm_cmd_last,  -arm_max_step,  arm_max_step)
                    lazy_cmd = lazy_cmd_last + clamp(lazy_cmd - lazy_cmd_last, -cont_max_step, cont_max_step)
                    head_cmd = head_cmd_last + clamp(head_cmd - head_cmd_last, -cont_max_step, cont_max_step)

                    arm_servo.value        = arm_cmd
                    lazy_susan_servo.value = lazy_cmd
                    head_servo.value       = head_cmd

                    arm_cmd_last, lazy_cmd_last, head_cmd_last = arm_cmd, lazy_cmd, head_cmd

                if DEBUG and now - last_print >= print_interval:
                    print(f"IMU(filt): R={roll_f:7.2f}  P={pitch_f:7.2f}  Y={yaw_raw:7.2f}")
                    print(f"Cmd: arm={arm_cmd_last:+.3f}  lazy={lazy_cmd_last:+.3f}  head={head_cmd_last:+.3f}")
                    print("-" * 70)
                    last_print = now

            time.sleep(imu_poll_interval)

    except KeyboardInterrupt:
        if DEBUG: print("\nStopping...")
    finally:
        arm_servo.value = 0
        lazy_susan_servo.value = 0
        head_servo.value = 0
        time.sleep(0.5)
        MOSFET.off()
        if DEBUG: print("Done.")

if __name__ == "__main__":
    main()