from mujoco.glfw import glfw
import mujoco as mj
import time
import os
import sys

# Add project root to path for imports
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import keyboard_control
from enums import CameraControl
from control_state import state
from input_management import update_input_matrix
from state_machine import (
    KeyboardInputSource,
    MuJoCoOutputSink,
    RobotStateMachine,
)

# load model & set up data and camera
# Use absolute path so script can be run from anywhere
modelPath = os.path.join(ROOT_DIR, "urdf", "bb8_fixed.xml")
model = mj.MjModel.from_xml_path(modelPath)
data = mj.MjData(model)

# visualization settings
cam = mj.MjvCamera()
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1

opt = mj.MjvOption()

# Get base_link body for camera tracking (base_link is fixed, no freejoint)
base_link_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base_link")

# Get actuator IDs for joint control only (no wheels in fixed version)
# Position actuators for joints:
# - a1_motor: controls a1 joint (first arm segment)
# - a2_motor: controls a2 joint (second arm segment)  
# - h_motor: controls h joint (head)
a1_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a1_motor")
a2_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a2_motor")
h_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "h_motor")
w1_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w1_motor")
w2_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w2_motor")
w3_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w3_motor")
w4_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w4_motor")

actuator_ids = {
    "a1_motor": a1_motor_id,
    "a2_motor": a2_motor_id,
    "h_motor": h_motor_id,
    "w1_motor": w1_motor_id,
    "w2_motor": w2_motor_id,
    "w3_motor": w3_motor_id,
    "w4_motor": w4_motor_id,
}

# Adapter wiring point:
# - replace KeyboardInputSource() with VisionInputSource(...) for CV inputs
# - replace MuJoCoOutputSink(...) with RealMotorOutputSink(...) for hardware
state_machine = RobotStateMachine(
    state=state,
    input_source=KeyboardInputSource(),
    output_sink=MuJoCoOutputSink(actuator_ids),
)

# Get joint IDs and qpos addresses for joint position tracking
a1_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "a1")
a2_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "a2")
h_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "h")

a1_qpos_addr = model.jnt_qposadr[a1_joint_id]
a2_qpos_addr = model.jnt_qposadr[a2_joint_id]
h_qpos_addr = model.jnt_qposadr[h_joint_id]

# create sim window & set scene
glfw.init()
window = glfw.create_window(720,540,"B_Cubed", None, None)
glfw.make_context_current(window)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# set simulation timers
sim_dt = 0.001
elapsed_time = 0.0
time_prev = time.perf_counter()

# set render timers
target_fps = 60 # You can change this if you want
frame_dt = 1.0/ target_fps
prev_render_t = time_prev

glfw.set_key_callback(window, keyboard_control.keyboard_callback)

# Open window
while not glfw.window_should_close(window):
    # get current time
    c_time = time.perf_counter()
    frame_time = c_time - time_prev
    time_prev = c_time
    elapsed_time += frame_time

    # simulation loop
    while elapsed_time >= sim_dt:

        state_machine.step(sim_dt, output_context={"data": data})

        # Handle camera controls (matching mujoco_test.py behavior)
        if state.cam_control == CameraControl.UP:
            cam.elevation += 2  # Move camera up
            state.cam_control = CameraControl.NONE  # Reset after single adjustment
        elif state.cam_control == CameraControl.DOWN:
            cam.elevation -= 2  # Move camera down
            state.cam_control = CameraControl.NONE
        elif state.cam_control == CameraControl.LEFT:
            cam.azimuth -= 2  # Rotate camera left
            state.cam_control = CameraControl.NONE
        elif state.cam_control == CameraControl.RIGHT:
            cam.azimuth += 2  # Rotate camera right
            state.cam_control = CameraControl.NONE
        elif state.cam_control == CameraControl.ZOOM_IN:
            cam.distance = max(0.5, cam.distance - 0.1)  # Zoom in (min distance 0.5)
            state.cam_control = CameraControl.NONE
        elif state.cam_control == CameraControl.ZOOM_OUT:
            cam.distance = min(10.0, cam.distance + 0.1)  # Zoom out (max distance 10)
            state.cam_control = CameraControl.NONE
        elif state.cam_control == CameraControl.RESET_CAMERA:
            # Reset camera to default position
            cam.distance = 2
            cam.azimuth = 45
            cam.elevation = -35
            state.cam_control = CameraControl.NONE
            print("Camera reset to default position")

        # update physics sim
        # update the shared input matrix (body speeds / head state)
        try:
            update_input_matrix()
        except Exception:
            pass
        mj.mj_step(model, data)
        elapsed_time -= sim_dt

    # Rendering Steps
    if c_time - prev_render_t >= frame_dt:
        prev_render_t = c_time

        # Camera tracking: follow base_link position
        # Since base_link is fixed (no freejoint), use its body position directly
        base_link_pos = data.xpos[base_link_body_id]
        cam.lookat[0:3] = base_link_pos  # [x, y, z] position of base_link
        
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)
        glfw.swap_buffers(window)
        glfw.poll_events()

glfw.terminate()