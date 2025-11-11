from mujoco.glfw import glfw
import numpy as np
import mujoco as mj
import time
import keyboard_control
# import sim.enums as states
from control_state import state

# load model & set up data and camera
modelPath = "balanceball.xml"
model = mj.MjModel.from_xml_path(modelPath)
body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "bb8")
head_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "head")
data = mj.MjData(model) 

# visualization settings
cam = mj.MjvCamera()
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1

opt = mj.MjvOption()

data = mj.MjData(model)

# define bb8 joint
joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "bb8_free")
qpos_addr = model.jnt_qposadr[joint_id]

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
    'W': False,  # Forward
    'A': False,  # Left
    'S': False,  # Back
    'D': False,   # Right
    '1': False, # Forward velocity
    '2': False, # Backward velocity
    '3': False, # Leftward velocity
    '4': False # Rightward velocity
}

def keyboard_callback(window, key, scancode, action, mods):
    global keyBoardControl, cameraControl, key_states
    
    # Handle key press and release for WASD keys
    if action == glfw.PRESS:
        if key == glfw.KEY_W:
            key_states['W'] = True
        elif key == glfw.KEY_A:
            key_states['A'] = True
        elif key == glfw.KEY_S:
            key_states['S'] = True
        elif key == glfw.KEY_D:
            key_states['D'] = True
        elif key == glfw.KEY_1:
            key_states['1'] = True
        elif key == glfw.KEY_2:
            key_states['2'] = True
        elif key == glfw.KEY_3:
            key_states['3'] = True
        elif key == glfw.KEY_4:
            key_states['4'] = True
    elif action == glfw.RELEASE:
        if key == glfw.KEY_W:
            key_states['W'] = False
        elif key == glfw.KEY_A:
            key_states['A'] = False
        elif key == glfw.KEY_S:
            key_states['S'] = False
        elif key == glfw.KEY_D:
            key_states['D'] = False
        elif key == glfw.KEY_1:
            key_states['1'] = False
        elif key == glfw.KEY_2:
            key_states['2'] = False
        elif key == glfw.KEY_3:
            key_states['3'] = False
        elif key == glfw.KEY_4:
            key_states['4'] = False


Kp = 10
Kd = 0.6
KI = 10

# PID controller 
def PID(error, prev_error, total_sum):
    new_error_sum = total_sum + error * sim_dt
    delta_error = error - prev_error
    control = Kp * error + Kd * delta_error/sim_dt + KI * new_error_sum
    return control, new_error_sum

time_sum = 0
# Open window
while not glfw.window_should_close(window):
    glfw.set_key_callback(window, keyboard_callback)

    # get current time
    c_time = time.perf_counter()
    frame_time = c_time - time_prev
    time_prev = c_time
    elapsed_time += frame_time
    perror = 0
    sum_error = 0
    time_sum = 0
    # simulation loop
    while elapsed_time >= sim_dt:
        
        # default force vector (nothing happens)
        force = np.zeros(6)
        

        # perturbed force vector (based on key strokes)
        if key_states['W']:
            force[0] = 5
        if key_states['S']:
            force[0] = -5
        if key_states['A']:
            force[1] = 5
        if key_states['D']:
            force[1] = -5
        # control velocity vector (based on key strokes)
        if key_states['1']:
            data.qvel[0] = control_vx
        if key_states['2']:
            data.qvel[0] = -control_vx
        if key_states['3']:
            data.qvel[1] = control_vy
        if key_states['4']:
            data.qvel[1] = -control_vy

        # applies force vector 
        data.xpos[body_id]

        # Body state: position of the center of the body 
        state_ball = np.array([round(data.xpos[body_id][0], 2), round(data.xpos[body_id][1], 2), round(data.xpos[body_id][2], 2)])

        # Head state: height above ground
        state_head = np.array([round(data.xpos[head_id][0], 2), round(data.xpos[head_id][1], 2), round(data.xpos[head_id][2], 2)])

        # radius vector between center of the ball and head
        r = round(np.linalg.norm(state_head - state_ball), 2)

        # At neutral the head is 0.16 above the body in xpos 

        # vector of deviations
        vec = state_head - state_ball

        # specific error in the theta direction 
        error = round(np.acos((state_head[2] - state_ball[2])/r), 3)

        # phi error 
        phi = np.atan2(vec[1], vec[0])
        
        # calculates control and updates the integral term 
        control, sum_error = PID(error, perror, sum_error)

        # reset the error
        perror = error

        # gives control inputs  
        control_vx = round(-control * sim_dt * np.sin(phi), 4)
        control_vy = round(-control * sim_dt * np.cos(phi), 4)
        

        print("Error", error, "Control_x", control_vx, "Control_vy", control_vy, "Error_vector", vec)

        # update physics sim

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


