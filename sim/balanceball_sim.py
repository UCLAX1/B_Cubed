from mujoco.glfw import glfw
import numpy as np
import mujoco as mj
import time
import os
import keyboard_control
# import sim.enums as states
from control_state import state

# load model & set up data and camera
# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
modelPath = os.path.join(script_dir, "balance_non_collidable_joint.xml")
model = mj.MjModel.from_xml_path(modelPath)
body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "sphere_body")
head_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "box_body")
data = mj.MjData(model) 

# visualization settings
cam = mj.MjvCamera()
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1

opt = mj.MjvOption()

data = mj.MjData(model)

# define sphere joint
joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "sphere_free")
qpos_addr = model.jnt_qposadr[joint_id]  # Position address (for free joint: [x, y, z, qw, qx, qy, qz])
qvel_addr = model.jnt_dofadr[joint_id]   # Velocity address (for free joint: [vx, vy, vz, wx, wy, wz])

# create sim window & set scene
glfw.init()
window = glfw.create_window(720,540,"Balanceball", None, None)
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

key_states = {
    'W': False,  # Forward perturbation (force +5 in x)
    'A': False,  # Left perturbation (force +5 in y)
    'S': False,  # Backward perturbation (force -5 in x)
    'D': False   # Right perturbation (force -5 in y)
}

def keyboard_callback(window, key, scancode, action, mods):
    global key_states
    
    # Handle key press and release for WASD keys (perturbations only)
    if action == glfw.PRESS:
        if key == glfw.KEY_W:
            key_states['W'] = True
        elif key == glfw.KEY_A:
            key_states['A'] = True
        elif key == glfw.KEY_S:
            key_states['S'] = True
        elif key == glfw.KEY_D:
            key_states['D'] = True
    elif action == glfw.RELEASE:
        if key == glfw.KEY_W:
            key_states['W'] = False
        elif key == glfw.KEY_A:
            key_states['A'] = False
        elif key == glfw.KEY_S:
            key_states['S'] = False
        elif key == glfw.KEY_D:
            key_states['D'] = False


# PID gains for horizontal displacement control
Kp = 8.0   # Proportional gain
Kd = 1.5  # Derivative gain (damping)
KI = 1.0   # Integral gain
IMax = 0.3  # Maximum integral term

# Velocity control parameters
max_velocity = 1.5  # Maximum desired velocity (m/s)
max_force = 25.0  # Maximum perturbation force (N)

# PID controller for 2D horizontal error
def PID_2D(error_xy, prev_error_xy, total_sum_xy):
    # error_xy is [error_x, error_y]
    # Apply PID to each component
    new_error_sum_x = np.clip(total_sum_xy[0] + error_xy[0] * sim_dt, -IMax, IMax)
    new_error_sum_y = np.clip(total_sum_xy[1] + error_xy[1] * sim_dt, -IMax, IMax)
    new_error_sum = np.array([new_error_sum_x, new_error_sum_y])
    
    delta_error = error_xy - prev_error_xy
    control_x = Kp * error_xy[0] + Kd * delta_error[0]/sim_dt + KI * new_error_sum[0]
    control_y = Kp * error_xy[1] + Kd * delta_error[1]/sim_dt + KI * new_error_sum[1]
    control = np.array([control_x, control_y])
    
    return control, new_error_sum

time_sum = 0
# Initialize PID state variables (outside the loop so they persist)
prev_error_xy = np.array([0.0, 0.0])  # Previous horizontal error [x, y]
sum_error_xy = np.array([0.0, 0.0])   # Integral term [x, y]

# Open window
while not glfw.window_should_close(window):
    glfw.set_key_callback(window, keyboard_callback)

    # get current time
    c_time = time.perf_counter()
    frame_time = c_time - time_prev
    time_prev = c_time
    elapsed_time += frame_time
    time_sum += frame_time
    # simulation loop
    while elapsed_time >= sim_dt:
        
        # Initialize force vector
        force = np.zeros(6)

        # Body state: position of the center of the body 
        ball_pos = data.xpos[body_id].copy()

        # Head state: position of the head
        head_pos = data.xpos[head_id].copy()

        # Compute horizontal error (head displacement from ball center in x-y plane)
        # This is the key: we want to move the ball to reduce this horizontal displacement
        error_xy = head_pos[:2] - ball_pos[:2]  # [error_x, error_y]
        error_xy[0] = round(error_xy[0], 3)
        error_xy[1] = round(error_xy[1], 3)
        print(error_xy)
        # Get head velocity to predict future position (optional, helps with stability)
        #head_vel = data.xvelp[head_id][:2] if hasattr(data, 'xvelp') else np.array([0.0, 0.0])
        
        # Calculate desired velocities from PID control
        # The PID controller outputs a desired velocity to reduce the horizontal error
        # For balance ball: when head is ahead, move ball forward to get under it
        control_xy, sum_error_xy = PID_2D(error_xy, prev_error_xy, sum_error_xy)
        
        # Update previous error for next iteration
        prev_error_xy = error_xy.copy()
        
        # The control output is the desired velocity
        # For balance ball: when head tilts forward (error_x > 0), we need to move ball backward
        # to counteract the tilt and restore balance. The ball should move opposite to the error.
        desired_vx = -control_xy[0]  # Negative: move opposite to error direction
        desired_vy = -control_xy[1]
        
        # Limit desired velocity to prevent instability
        desired_vx = np.clip(desired_vx, -max_velocity, max_velocity)
        desired_vy = np.clip(desired_vy, -max_velocity, max_velocity)
        
        # Apply direct velocity control
        # Start control immediately to maintain balance
        if time_sum >= 0.0:  # Start control immediately
            # Directly set the ball's velocity to the desired velocity
            # For a free joint, qvel has [vx, vy, vz, wx, wy, wz] starting at qvel_addr
            data.qvel[qvel_addr + 0] = desired_vx  # vx
            data.qvel[qvel_addr + 1] = desired_vy  # vy
        
        # Apply perturbation forces (from WASD keys) - magnitude 5 in each direction
        # These are applied as forces to test the balance controller's response
        if key_states['W']:
            force[0] += 10.0  # Forward perturbation (+x direction)
        if key_states['S']:
            force[0] += -10.0  # Backward perturbation (-x direction)
        if key_states['A']:
            force[1] += 10.0  # Left perturbation (+y direction)
        if key_states['D']:
            force[1] += -10.0  # Right perturbation (-y direction)
        

        
        # Limit perturbation forces
        force[0] = np.clip(force[0], -max_force, max_force)
        force[1] = np.clip(force[1], -max_force, max_force)

        # print("Error", error, "Control_x", control_vx, "Control_vy", control_vy, "Error_vector", vec)
        # print("stopwatch ", time_sum)

        data.xfrc_applied[body_id] = force
        mj.mj_step(model, data)
        elapsed_time -= sim_dt


    # Rendering Steps
    if c_time - prev_render_t >= frame_dt:
        prev_render_t = c_time

        cam.lookat[0:3] = data.qpos[qpos_addr : qpos_addr + 3]  # [x, y, z]
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)
        glfw.swap_buffers(window)
        glfw.poll_events()


glfw.terminate()


