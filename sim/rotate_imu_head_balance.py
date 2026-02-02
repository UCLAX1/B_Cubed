"""
rotate_imu_head_balance.py

This script loads the imu_head_balance.xml MuJoCo model and subscribes to a ROS2 topic that publishes pitch, roll, and yaw (from a Pi IMU). It rotates the base accordingly and uses find_motor_angles to move a1 (Lazy_Susan) and a2 (Arm).

Requirements:
- ROS2 (rclpy, std_msgs)
- mujoco, glfw, numpy
- head_balance_math.py in the same directory or in PYTHONPATH

Run:
  python rotate_imu_head_balance.py
"""

import os
import sys
import time
import numpy as np
from threading import Thread

# MuJoCo and GLFW imports for simulation and rendering
from mujoco.glfw import glfw
import mujoco as mj

# ROS2 imports for subscribing to IMU data
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Import the function to compute motor angles from pitch/roll
from head_balance_math import find_motor_angles

# Path setup for imu_head_balance.xml (relative to this script)
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODEL_PATH = os.path.join(ROOT_DIR, "urdf", "imu_head_balance.xml")

# Shared variable for IMU data (pitch, roll, yaw)
global_imu_angles = np.zeros(3)

# ROS2 callback: receives IMU data as [pitch, roll, yaw] (degrees)
def imu_callback(msg):
    global global_imu_angles
    # Expecting msg.data = [pitch, roll, yaw] in degrees, convert to radians
    if len(msg.data) == 3:
        global_imu_angles = np.array(msg.data) * np.pi / 180.0
        print(f"[ROS2 Callback] Received: {msg.data}")

# ROS2 Node that subscribes to the IMU topic
class ImuListener(Node):
    def __init__(self, topic_name="/sense_hat/raw"):
        super().__init__('imu_listener')
        print(f"[ROS2] Subscribing to topic: {topic_name}")
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name,
            imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        print("[ROS2] Subscription created successfully")

# Start the ROS2 node in a background thread
def start_ros2_node():
    node = ImuListener()
    rclpy.spin(node)
    
def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 node for IMU subscription
    imu_node = ImuListener()
    
    # Load the MuJoCo model from XML file
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)
    
    # Debug: print gravity
    print(f"Gravity: {model.opt.gravity}")
    print(f"Model bodies: {[mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]}")
    print(f"Model actuators: {[mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]}")

    # Find actuator IDs for a1 (Lazy_Susan) and a2 (Arm)
    a1_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a1_motor")
    a2_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a2_motor")
    print(f"Actuator IDs - a1: {a1_id}, a2: {a2_id}")
    
    # Test: Set motors to 45 degrees on startup
    if a1_id != -1:
        data.ctrl[a1_id] = 45 * np.pi/180
    if a2_id != -1:
        data.ctrl[a2_id] = 45 * np.pi/180
    print("Test: Setting motors to 45 degrees")
    # Note: base orientation quaternion is at qpos[3:7]

    # Setup the MuJoCo viewer window and camera
    glfw.init()
    window = glfw.create_window(720, 540, "IMU Head Balance", None, None)
    glfw.make_context_current(window)
    cam = mj.MjvCamera()
    cam.distance = 2
    cam.azimuth = 45
    cam.elevation = -35
    cam.orthographic = 1
    opt = mj.MjvOption()
    mj.mjv_defaultOption(opt)
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    # Simulation and rendering timing setup
    sim_dt = 0.001
    elapsed_time = 0.0
    time_prev = time.perf_counter()
    target_fps = 60
    frame_dt = 1.0 / target_fps
    prev_render_t = time_prev
    
    # Debug counter
    debug_counter = 0

    # Main simulation loop
    while not glfw.window_should_close(window):
        c_time = time.perf_counter()
        frame_time = c_time - time_prev
        time_prev = c_time
        elapsed_time += frame_time
# Spin ROS2 to receive callbacks (non-blocking)
        rclpy.spin_once(imu_node, timeout_sec=0.0)
        
        
        # Physics steps (advance simulation)
        while elapsed_time >= sim_dt:
            # Get latest roll, yaw, pitch from IMU (in radians)
            roll, yaw, pitch = global_imu_angles
            # Don't set base orientation - let it stay fixed
            # Instead, only use IMU data to calculate motor positions
            
            # Debug output every 1000 steps
            debug_counter += 1
            if debug_counter % 1000 == 0:
                print(f"IMU (deg): roll={roll*180/np.pi:.2f}, yaw={yaw*180/np.pi:.2f}, pitch={pitch*180/np.pi:.2f}")
                print(f"Motor angles (deg): arm={arm:.2f}, lazy_susan={lazy_susan:.2f}")
                print(f"Ctrl values: a1={lazy_susan*np.pi/180:.4f}, a2={arm*np.pi/180:.4f}")
            
            
            # Use find_motor_angles to compute a1 (Lazy_Susan) and a2 (Arm) angles (expects degrees)
            arm, lazy_susan = find_motor_angles(pitch * 180/np.pi, roll * 180/np.pi)
            # Set actuator controls (convert degrees to radians)
            if a1_id != -1:
                data.ctrl[a1_id] = lazy_susan * np.pi/180
            if a2_id != -1:
                data.ctrl[a2_id] = arm * np.pi/180
            # Step the simulation
            mj.mj_step(model, data)
            elapsed_time -= sim_dt

        # Rendering (update the viewer window)
        if c_time - prev_render_t >= frame_dt:
            prev_render_t = c_time
            cam.lookat[0:3] = data.qpos[0:3]
            viewport_w, viewport_h = glfw.get_framebuffer_size(window)
            viewport = mj.MjrRect(0, 0, viewport_w, viewport_h)
            mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
            mj.mjr_render(viewport, scene, context)
            glfw.swap_buffers(window)
            glfw.poll_events()
    rclpy.shutdown()

    # Clean up and close the window
    glfw.terminate()

# Entry point
if __name__ == "__main__":
    main()
