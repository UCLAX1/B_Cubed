"""
3D Real-time IMU and Motor Angle Visualizer
Shows a 3D representation of the robot tilting with IMU data and target motor angles.
"""

import sys
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
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
print(f"IMU initialized. Poll interval: {imu_poll_interval*1000:.1f}ms\n")

# ============================================================================
# DATA STORAGE
# ============================================================================
imu_data = {
    'pitch': 0.0,
    'roll': 0.0,
    'yaw': 0.0,
    'accel': [0.0, 0.0, 0.0],
    'gyro': [0.0, 0.0, 0.0],
    'mag': [0.0, 0.0, 0.0],
    'arm': 0.0,
    'lazy_susan': 0.0,
    'head': 0.0,
}

def read_imu():
    """Continuously read IMU data in background"""
    global imu_data
    while True:
        if imu.IMURead():
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
            
            imu_data['roll'] = math.degrees(fusionPose[0])
            imu_data['pitch'] = math.degrees(fusionPose[1])
            imu_data['yaw'] = math.degrees(fusionPose[2])
            
            imu_data['accel'] = data["accel"]
            imu_data['gyro'] = data["gyro"]
            imu_data['mag'] = data["compass"]
            
            # Calculate target angles
            arm, lazy_susan, head = find_motor_angles(imu_data['pitch'], imu_data['roll'], 0.0)
            imu_data['arm'] = arm
            imu_data['lazy_susan'] = lazy_susan
            imu_data['head'] = head
        
        time.sleep(imu_poll_interval)

# Start background IMU thread
import threading
imu_thread = threading.Thread(target=read_imu, daemon=True)
imu_thread.start()
time.sleep(0.5)  # Let thread start

# ============================================================================
# 3D VISUALIZATION
# ============================================================================

def create_box(center, size, rotation_matrix):
    """Create a box with vertices"""
    # Box vertices (before rotation)
    vertices = np.array([
        [-size[0]/2, -size[1]/2, -size[2]/2],
        [size[0]/2, -size[1]/2, -size[2]/2],
        [size[0]/2, size[1]/2, -size[2]/2],
        [-size[0]/2, size[1]/2, -size[2]/2],
        [-size[0]/2, -size[1]/2, size[2]/2],
        [size[0]/2, -size[1]/2, size[2]/2],
        [size[0]/2, size[1]/2, size[2]/2],
        [-size[0]/2, size[1]/2, size[2]/2],
    ])
    
    # Apply rotation
    rotated = vertices @ rotation_matrix.T
    rotated += center
    return rotated

def rotation_matrix_xyz(roll_deg, pitch_deg, yaw_deg):
    """Create rotation matrix from roll, pitch, yaw (in degrees)"""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    
    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    
    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    
    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx

def draw_robot_box(ax, center, size, roll, pitch, yaw, color='blue', alpha=0.7):
    """Draw a rotated box representing the robot"""
    R = rotation_matrix_xyz(roll, pitch, yaw)
    vertices = create_box(center, size, R)
    
    # Define the 12 edges of a cube
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom
        (4, 5), (5, 6), (6, 7), (7, 4),  # top
        (0, 4), (1, 5), (2, 6), (3, 7),  # vertical
    ]
    
    for edge in edges:
        points = vertices[list(edge)]
        ax.plot3D(*points.T, color=color, linewidth=2)
    
    # Draw center point
    ax.scatter(*center, color=color, s=100)

def draw_arrow(ax, start, direction, length=2, color='red', label=''):
    """Draw an arrow to show a direction"""
    end = start + direction / np.linalg.norm(direction) * length
    ax.quiver(*start, *(end - start), color=color, arrow_length_ratio=0.3, linewidth=2, label=label)

# Create figure
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(121, projection='3d')
ax_info = fig.add_subplot(222)
ax_targets = fig.add_subplot(224)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim([-4, 4])
ax.set_ylim([-4, 4])
ax.set_zlim([-4, 4])

update_count = [0]

def update_frame(frame_num):
    """Update 3D visualization"""
    ax.clear()
    ax_info.clear()
    ax_targets.clear()
    
    # Draw the robot as a box tilted by IMU angles
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-4, 4])
    ax.set_ylim([-4, 4])
    ax.set_zlim([-4, 4])
    ax.set_title(f'Robot Orientation (Live IMU Data)', fontsize=12, fontweight='bold')
    
    # Draw gravity vector (always pointing down)
    ax.quiver(0, 0, 0, 0, 0, -3, color='green', arrow_length_ratio=0.2, linewidth=3, label='Gravity')
    
    # Draw robot box
    draw_robot_box(ax, [0, 0, 0], [1.5, 1.0, 1.0], 
                   imu_data['roll'], imu_data['pitch'], imu_data['yaw'],
                   color='blue', alpha=0.8)
    
    # Draw reference axes
    ax.quiver(0, 0, 0, 2, 0, 0, color='red', arrow_length_ratio=0.2, linewidth=2, label='X (Roll axis)')
    ax.quiver(0, 0, 0, 0, 2, 0, color='blue', arrow_length_ratio=0.2, linewidth=2, label='Y (Pitch axis)')
    ax.quiver(0, 0, 0, 0, 0, 2, color='black', arrow_length_ratio=0.2, linewidth=2, label='Z')
    
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True)
    
    # ======================== INFO PANEL ========================
    ax_info.axis('off')
    info_text = f"""
    IMU READINGS (degrees)
    ─────────────────────
    Pitch: {imu_data['pitch']:7.2f}°
    Roll:  {imu_data['roll']:7.2f}°
    Yaw:   {imu_data['yaw']:7.2f}°
    
    ACCELEROMETER (g)
    ─────────────────────
    X: {imu_data['accel'][0]:7.4f}
    Y: {imu_data['accel'][1]:7.4f}
    Z: {imu_data['accel'][2]:7.4f}
    """
    ax_info.text(0.1, 0.95, info_text, transform=ax_info.transAxes,
                fontsize=10, verticalalignment='top', family='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # ======================== TARGETS PANEL ========================
    ax_targets.axis('off')
    targets_text = f"""
    TARGET MOTOR ANGLES
    ─────────────────────
    Arm:         {imu_data['arm']:7.2f}°
    Lazy Susan:  {imu_data['lazy_susan']:7.2f}°
    Head:        {imu_data['head']:7.2f}°
    """
    ax_targets.text(0.1, 0.95, targets_text, transform=ax_targets.transAxes,
                   fontsize=11, verticalalignment='top', family='monospace',
                   bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    plt.tight_layout()

# Create animation
ani = FuncAnimation(fig, update_frame, interval=100, cache_frame_data=False)

plt.suptitle('B-Cubed IMU 3D Visualizer', fontsize=14, fontweight='bold')
plt.show()
