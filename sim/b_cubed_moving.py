from mujoco.glfw import glfw
import mujoco as mj
import time
import keyboard_control
from enums import (
    BotControl,
    CameraControl,
    AngularVelocityControl,
    JointControl,
)  # For use in control logic
from control_state import state

# load model & set up data and camera
modelPath = "../urdf/bb8_hl.xml"  # Updated to use bb8_car_converted.xml
model = mj.MjModel.from_xml_path(modelPath)
data = mj.MjData(model)

# visualization settings
cam = mj.MjvCamera()
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1

opt = mj.MjvOption()

# define base_link body and free joint for position tracking
base_link_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base_link")
# Get the free joint (freejoint is the first joint of base_link body)
base_link_joint_id = model.body_jntadr[base_link_body_id]
qpos_addr = model.jnt_qposadr[base_link_joint_id]

# Get actuator IDs for motor control
# Position actuators for joints:
# - a1_motor: controls a1 joint (first arm segment)
# - a2_motor: controls a2 joint (second arm segment)
# - h_motor: controls h joint (head)
# Motor actuators for wheels:
# - w1_motor, w2_motor, w3_motor, w4_motor: control wheel rotation
a1_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a1_motor")
a2_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a2_motor")
h_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "h_motor")
w1_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w1_motor")
w2_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w2_motor")
w3_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w3_motor")
w4_motor_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "w4_motor")

body_x_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "bd_motor")
body_y_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "bf_motor")
body_r_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "br_motor")


# Debug: Print motor IDs to verify they're found
print(
    f"Motor IDs: w1={w1_motor_id}, w2={w2_motor_id}, w3={w3_motor_id}, w4={w4_motor_id}"
)
if w1_motor_id == -1 or w2_motor_id == -1 or w3_motor_id == -1 or w4_motor_id == -1:
    print("WARNING: One or more wheel motor IDs not found!")

# Get joint IDs and qpos addresses for joint position tracking
a1_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "a1")
a2_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "a2")
h_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "h")

a1_qpos_addr = model.jnt_qposadr[a1_joint_id]
a2_qpos_addr = model.jnt_qposadr[a2_joint_id]
h_qpos_addr = model.jnt_qposadr[h_joint_id]

# Target positions are now stored in state (control_state.py)
# We'll use state.target_a1_pos, state.target_a2_pos, state.target_h_pos
# Wheel speeds are also stored in state (w1_speed, w2_speed, w3_speed, w4_speed)

# create sim window & set scene
glfw.init()
window = glfw.create_window(720, 540, "B_Cubed", None, None)
glfw.make_context_current(window)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# set simulation timers
sim_dt = 0.001
elapsed_time = 0.0
time_prev = time.perf_counter()

# set render timers
target_fps = 60  # You can change this if you want
frame_dt = 1.0 / target_fps
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

        # ============================================
        # WHEEL CONTROL SECTION
        # ============================================
        # Wheel speeds are incremented by 0.1 per key press in keyboard_control.py
        # Forward: w1 positive, w3 negative
        # Backward: w1 negative, w3 positive
        # Left: w2 positive, w3 negative
        # Right: w2 negative, w3 positive
        # Apply wheel speeds from state (already clamped to [-1, 1])
        if w1_motor_id != -1:
            data.ctrl[w1_motor_id] = state.w1_speed
        if w2_motor_id != -1:
            data.ctrl[w2_motor_id] = state.w2_speed
        if w3_motor_id != -1:
            data.ctrl[w3_motor_id] = state.w3_speed
        if w4_motor_id != -1:
            data.ctrl[w4_motor_id] = state.w4_speed

        if body_x_id != -1:
            data.ctrl[body_x_id] = state.body_x_speed
        if body_y_id != -1:
            data.ctrl[body_y_id] = state.body_y_speed
        if body_r_id != -1:
            data.ctrl[body_r_id] = state.body_r_speed    
            
        # ============================================

        # ============================================
        # ANGULAR VELOCITY CONTROL SECTION
        # ============================================
        # TODO: Implement angular velocity control for base_link rotation
        # Left arrow key: state.angular_vel_control == AngularVelocityControl.LEFT
        # Right arrow key: state.angular_vel_control == AngularVelocityControl.RIGHT
        #
        # This should apply angular velocity to the base_link body
        # Example approach:
        #   if state.angular_vel_control == AngularVelocityControl.LEFT:
        #       # Set angular velocity for left rotation
        #       # You may need to access the base_link's angular velocity directly
        #       # data.qvel[qpos_addr + 3:qpos_addr + 6] = [wx, wy, wz]  # Angular velocity components
        #       # Or use a motor/actuator if one exists for base rotation
        #
        #   elif state.angular_vel_control == AngularVelocityControl.RIGHT:
        #       # Set angular velocity for right rotation
        #       # Similar to above but opposite direction
        #
        # Note: The base_link has a freejoint, so you'll be setting angular velocity
        # in the base_link's body frame or world frame depending on your implementation

        # Reset angular velocity control after applying

        # if state.angular_vel_control == AngularVelocityControl.LEFT:
        #     # Apply left rotation angular velocity
        #     # data.qvel[qpos_addr + 0 : qpos_addr + 2] = 0.0
        #     # data.qvel[qpos_addr + 3] = 0.05  # Set some angular velocity around x-axis

        # if state.angular_vel_control == AngularVelocityControl.RIGHT:
        #     # Apply right rotation angular velocity
        #     # data.qvel[qpos_addr + 0 : qpos_addr + 2] = 0.0
        #     # data.qvel[qpos_addr + 3] = -0.05  # Set some angular velocity around x-axis
            

        # ============================================

        # ============================================
        # JOINT POSITION CONTROL SECTION
        # ============================================
        # Individual joint position control (Z, X, C, V, B, N keys)
        # Positions are updated one-time per key press in keyboard_control.py
        # Just apply the current target positions from state
        data.ctrl[a1_motor_id] = state.target_a1_pos
        data.ctrl[a2_motor_id] = state.target_a2_pos
        data.ctrl[h_motor_id] = state.target_h_pos
        # ============================================

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
        mj.mj_step(model, data)
        elapsed_time -= sim_dt

    # Rendering Steps
    if c_time - prev_render_t >= frame_dt:
        prev_render_t = c_time

        # Camera tracking: follow base_link position
        # The new XML structure has base_link as the main body instead of bb8
        cam.lookat[0:3] = data.qpos[
            qpos_addr : qpos_addr + 3
        ]  # [x, y, z] position of base_link

        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        mj.mjv_updateScene(
            model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene
        )
        mj.mjr_render(viewport, scene, context)
        glfw.swap_buffers(window)
        glfw.poll_events()


glfw.terminate()