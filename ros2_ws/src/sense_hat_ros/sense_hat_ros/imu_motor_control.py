"""
imu_motor_control.py

This script loads the bb8_hl.xml MuJoCo model and subscribes to a ROS2 IMU topic.
It uses the IMU's angular velocity (z) to control the robot's rotation motor in simulation.

Requirements:
- ROS2 (rclpy, sensor_msgs)
- mujoco, glfw, numpy

Run:
  python imu_motor_control.py
"""

import os
import sys
import time
import numpy as np
from threading import Thread

from mujoco.glfw import glfw
import mujoco as mj

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Path setup for bb8_hl.xml
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODEL_PATH = os.path.join(ROOT_DIR, "urdf", "bb8_hl.xml")

# Shared variable for IMU data
global_imu_angular_z = 0.0

def imu_callback(msg):
    global global_imu_angular_z
    # Use angular velocity z (yaw rate)
    global_imu_angular_z = msg.angular_velocity.z

class ImuListener(Node):
    def __init__(self, topic_name="sense_hat/raw"):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            topic_name,
            imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning

def start_ros2_node():
    rclpy.init()
    node = ImuListener()
    rclpy.spin(node)
    rclpy.shutdown()

def main():
    # Start ROS2 node in a separate thread
    ros_thread = Thread(target=start_ros2_node, daemon=True)
    ros_thread.start()

    # Load MuJoCo model
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    # Find rotation motor actuator (br_motor)
    body_r_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "br_motor")
    if body_r_id == -1:
        print("[ERROR] br_motor actuator not found in model.")
        return

    # Setup window
    glfw.init()
    window = glfw.create_window(720, 540, "IMU Motor Control", None, None)
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

    # Main loop
    sim_dt = 0.001
    elapsed_time = 0.0
    time_prev = time.perf_counter()
    target_fps = 60
    frame_dt = 1.0 / target_fps
    prev_render_t = time_prev

    while not glfw.window_should_close(window):
        c_time = time.perf_counter()
        frame_time = c_time - time_prev
        time_prev = c_time
        elapsed_time += frame_time

        # Physics steps
        while elapsed_time >= sim_dt:
            # Use IMU angular velocity to set rotation motor
            data.ctrl[body_r_id] = global_imu_angular_z * 10000  # scale as needed
            mj.mj_step(model, data)
            elapsed_time -= sim_dt

        # Rendering
        if c_time - prev_render_t >= frame_dt:
            prev_render_t = c_time
            cam.lookat[0:3] = data.qpos[0:3]
            viewport_w, viewport_h = glfw.get_framebuffer_size(window)
            viewport = mj.MjrRect(0, 0, viewport_w, viewport_h)
            mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
            mj.mjr_render(viewport, scene, context)
            glfw.swap_buffers(window)
            glfw.poll_events()

    glfw.terminate()

if __name__ == "__main__":
    main()
