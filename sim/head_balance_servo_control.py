"""
head_balance_servo_control.py

Implements head balance using actual servo hardware with safe, slow movements.
Uses I2C IMU data to calculate motor angles and smoothly moves servos to target positions.
Integrates ServoEx for encoder-based position feedback.

Uses egg.py-style RTIMU initialization and polling pattern.
"""

import sys
import time
import math
from gpiozero import DigitalOutputDevice
from ServoEx import ServoEx
from head_balance_math import find_motor_angles

# ============================================================================
# IMU INITIALIZATION (egg.py style)
# ============================================================================
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

imu_poll_interval = imu.IMUGetPollInterval() / 1000.0

# ============================================================================
# SERVO CONFIGURATION WITH ENCODERS
# ============================================================================

# ServoEx initialization: ServoEx(servo_pin, encoder_pin_a, encoder_pin_b, absolute_encoder_pin)
# Adjust encoder pins based on your hardware wiring
try:
    lazy_susan_servo = ServoEx(servo_pin=18, encoder_pin_a=24, encoder_pin_b=25, absolute_encoder_pin=27)  # Continuous (Physical Pin 12)
    arm_servo = ServoEx(servo_pin=12, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)           # Continuous (Physical Pin 32)
    head_servo = ServoEx(servo_pin=13, encoder_pin_a=19, encoder_pin_b=20, absolute_encoder_pin=21)        # Standard (Physical Pin 33)
except Exception as e:
    print(f"ERROR initializing servos: {e}")
    print("Make sure all encoder pins are connected correctly")
    exit(1)

MOSFET = DigitalOutputDevice(16)  # MOSFET control (Physical Pin 36)

# inital variables
MAX_VELOCITY_LAZY_SUSAN = 15.0  # deg/sec - rotation servo (slow to avoid slip)
MAX_VELOCITY_ARM = 20.0          # deg/sec - arm servo
MAX_VELOCITY_HEAD = 25.0         # deg/sec - head servo (fastest, least likely to damage)

# Angle limits (degrees) to prevent mechanical damage
LAZY_SUSAN_MIN = -90
LAZY_SUSAN_MAX = 90
ARM_MIN = -120
ARM_MAX = 120
# HEAD_MIN = -180
# HEAD_MAX = 180

# Control loop timing
CONTROL_LOOP_HZ = 20  # 50ms update rate
CONTROL_LOOP_DT = 1.0 / CONTROL_LOOP_HZ


# SERVO VALUE CONVERSION
def velocity_to_servo_value(velocity_deg_per_sec, max_velocity):
    """
    Convert desired angular velocity (deg/sec) to servo control value (-1 to 1).
    Clamps velocity to max allowed.
    """
    velocity_deg_per_sec = max(min(velocity_deg_per_sec, max_velocity), -max_velocity)
    servo_value = velocity_deg_per_sec / max_velocity
    return servo_value


def angle_to_servo_value(angle_deg, max_velocity):
    """
    For standard servos, convert angle to servo value.
    For our purposes, assuming standard servo maps -90 to 90 degrees to -1 to 1.
    """
    # Clamp to servo range
    angle_deg = max(min(angle_deg, 90), -90)
    servo_value = angle_deg / 90.0
    return servo_value




# HEAD BALANCE CONTROLLER WITH ENCODER FEEDBACK
class HeadBalanceController:
    def __init__(self):
        """
        Initialize the head balance controller.
        Uses global IMU object (egg.py pattern).
        """
        
        # Current servo positions (from encoders, in degrees)
        self.lazy_susan_angle = 0.0
        self.arm_angle = 0.0
        self.head_angle = 0.0
        
        # Desired target angles (degrees)
        self.lazy_susan_target = 0.0
        self.arm_target = 0.0
        self.head_target = 0.0
        
        # Desired head angle (global orientation)
        self.desired_head_angle = 0.0
        
        # Timestamps for velocity calculation
        self.last_update_time = time.time()
        
        # Stall detection
        self.prev_encoder_pos = [0.0, 0.0, 0.0]
        self.stall_count = [0, 0, 0]  # lazy_susan, arm, head
        self.STALL_THRESHOLD = 3  # counts before triggering stall
    
    def get_imu_angles(self):
        """Get pitch, roll, yaw from global IMU (egg.py pattern)"""
        if imu.IMURead():
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
            roll = math.degrees(fusionPose[0])
            pitch = math.degrees(fusionPose[1])
            yaw = math.degrees(fusionPose[2])
            return pitch, roll, yaw
        return 0.0, 0.0, 0.0  # Fallback if no data
    
    def clamp_angle(self, angle, min_angle, max_angle):
        """Clamp angle to safe limits"""
        return max(min(angle, max_angle), min_angle)
    
    def smooth_move(self, current, target, max_velocity, dt):
        """
        Calculate next angle value moving toward target at max velocity.
        
        Args:
            current: Current angle (degrees)
            target: Target angle (degrees)
            max_velocity: Maximum velocity (degrees/second)
            dt: Time delta (seconds)
        
        Returns:
            Next angle value
        """
        max_step = max_velocity * dt
        error = target - current
        
        if abs(error) <= max_step:
            return target
        elif error > 0:
            return current + max_step
        else:
            return current - max_step
    
    def detect_stall(self, servo_id, current_pos):
        """
        Detect if servo is stalled (not moving when it should be).
        
        Args:
            servo_id: 0=lazy_susan, 1=arm, 2=head
            current_pos: Current encoder position
        """
        target = [self.lazy_susan_target, self.arm_target, self.head_target][servo_id]
        current = [self.lazy_susan_angle, self.arm_angle, self.head_angle][servo_id]
        
        # If target and current are far apart but position hasn't moved
        if abs(target - current) > 5.0 and abs(current_pos - self.prev_encoder_pos[servo_id]) < 0.1:
            self.stall_count[servo_id] += 1
            if self.stall_count[servo_id] >= self.STALL_THRESHOLD:
                servo_names = ["Lazy Susan", "Arm", "Head"]
                print(f"[WARNING] Potential stall detected on {servo_names[servo_id]}")
                return True
        else:
            self.stall_count[servo_id] = 0
        
        self.prev_encoder_pos[servo_id] = current_pos
        return False
    
    def update(self):
        """Update controller state and calculate servo commands"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Get IMU angles (uses global IMU from egg.py pattern)
        pitch, roll, yaw = self.get_imu_angles()
        
        # Calculate target motor angles using head balance math
        arm_target, lazy_susan_target, head_target = find_motor_angles(
            pitch, roll, self.desired_head_angle
        )
        
        # Store targets
        self.lazy_susan_target = arm_target
        self.arm_target = lazy_susan_target
        self.head_target = head_target
        
        # Clamp targets to safe ranges
        self.lazy_susan_target = self.clamp_angle(
            self.lazy_susan_target, LAZY_SUSAN_MIN, LAZY_SUSAN_MAX
        )
        self.arm_target = self.clamp_angle(
            self.arm_target, ARM_MIN, ARM_MAX
        )
        # Head has no angle limits
        
        # Update servo encoder readings
        lazy_susan_pos = lazy_susan_servo.get_position() * 360.0  # Convert rotations to degrees
        arm_pos = arm_servo.get_position() * 360.0
        head_pos = head_servo.get_position() * 360.0
        
        # Detect stalls
        self.detect_stall(0, lazy_susan_pos)
        self.detect_stall(1, arm_pos)
        self.detect_stall(2, head_pos)
        
        # Use encoder positions instead of calculated positions for accuracy
        self.lazy_susan_angle = lazy_susan_pos
        self.arm_angle = arm_pos
        self.head_angle = head_pos
    
    def set_servo_positions(self):
        """Send current servo targets to hardware"""
        try:
            # For continuous servos, calculate velocity needed
            lazy_susan_error = self.lazy_susan_target - self.lazy_susan_angle
            arm_error = self.arm_target - self.arm_angle
            head_error = self.head_target - self.head_angle
            
            # Convert to servo values
            # For ServoEx continuous servos: -1.0 = full reverse, 0.0 = stop, 1.0 = full forward
            lazy_susan_servo.value = velocity_to_servo_value(
                lazy_susan_error / CONTROL_LOOP_DT, MAX_VELOCITY_LAZY_SUSAN
            )
            arm_servo.value = velocity_to_servo_value(
                arm_error / CONTROL_LOOP_DT, MAX_VELOCITY_ARM
            )
            head_servo.value = angle_to_servo_value(self.head_target, MAX_VELOCITY_HEAD)
            
            # Update encoders
            lazy_susan_servo.update()
            arm_servo.update()
            head_servo.update()
            
        except Exception as e:
            print(f"Error setting servo positions: {e}")
    
    def print_status(self):
        """Print current status"""
        print(f"\n{'='*70}")
        print(f"Head Balance Status")
        print(f"{'='*70}")
        pitch, roll, yaw = self.get_imu_angles()
        print(f"IMU Data:")
        print(f"  Pitch: {pitch:7.2f}°  |  Roll: {roll:7.2f}°  |  Yaw: {yaw:7.2f}°")
        print(f"\nServo Angles (Current -> Target):")
        print(f"  Lazy Susan: {self.lazy_susan_angle:7.2f}° -> {self.lazy_susan_target:7.2f}°")
        print(f"  Arm:        {self.arm_angle:7.2f}° -> {self.arm_target:7.2f}°")
        print(f"  Head:       {self.head_angle:7.2f}° -> {self.head_target:7.2f}°")
        print(f"{'='*70}")


# MAIN LOOP
def main():
    print("Initializing Head Balance Servo Control with I2C IMU...\n")
    print("Using egg.py-style RTIMU initialization and polling pattern\n")
    
    # Initialize controller
    controller = HeadBalanceController()
    
    try:
        # Enable power
        print("Turning MOSFET ON...")
        MOSFET.on()
        time.sleep(0.5)
        
        # Center all servos first
        print("Centering servos...\n")
        lazy_susan_servo.value = 0
        arm_servo.value = 0
        head_servo.value = 0
        time.sleep(1)
        
        print("Head balance control active! Press Ctrl+C to stop.\n")
        
        # Main control loop (egg.py pattern with dual-rate servo control)
        last_status_print = time.time()
        last_servo_update = time.time()
        status_print_interval = 2.0  # Print status every 2 seconds
        servo_update_interval = CONTROL_LOOP_DT  # Servo updates at 20Hz for safety
        
        while True:
            # Poll IMU at RTIMU's recommended interval (egg.py pattern)
            if imu.IMURead():
                current_time = time.time()
                
                # Update servo positions at controlled rate
                if current_time - last_servo_update >= servo_update_interval:
                    controller.update()
                    controller.set_servo_positions()
                    last_servo_update = current_time
                
                # Print status periodically
                if current_time - last_status_print >= status_print_interval:
                    controller.print_status()
                    last_status_print = current_time
            
            # Sleep for RTIMU's recommended poll interval (egg.py uses this)
            time.sleep(imu_poll_interval)
    
    except KeyboardInterrupt:
        print("\n\nStopping head balance control...")
    
    finally:
        # Safe shutdown
        print("Centering servos...")
        lazy_susan_servo.value = 0
        arm_servo.value = 0
        head_servo.value = 0
        time.sleep(0.5)
        
        print("Turning MOSFET OFF...")
        MOSFET.off()
        print("Done!")


if __name__ == "__main__":
    main()
