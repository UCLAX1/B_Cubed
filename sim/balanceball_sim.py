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
    'KEY_UP': False, # Forward velocity
    'KEY_DOWN': False, # Backward velocity
    'KEY_LEFT': False, # Leftward velocity
    'KEY_RIGHT': False # Rightward velocity
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
        elif key == glfw.KEY_W:
            key_states['KEY_UP'] = True
        elif key == glfw.KEY_A:
            key_states['KEY_DOWN'] = True
        elif key == glfw.KEY_S:
            key_states['KEY_LEFT'] = True
        elif key == glfw.KEY_D:
            key_states['KEY_RIGHT'] = True
    elif action == glfw.RELEASE:
        if key == glfw.KEY_W:
            key_states['W'] = False
        elif key == glfw.KEY_A:
            key_states['A'] = False
        elif key == glfw.KEY_S:
            key_states['S'] = False
        elif key == glfw.KEY_D:
            key_states['D'] = False
        elif key == glfw.KEY_W:
            key_states['KEY_UP'] = False
        elif key == glfw.KEY_A:
            key_states['KEY_DOWN'] = False
        elif key == glfw.KEY_S:
            key_states['KEY_LEFT'] = False
        elif key == glfw.KEY_D:
            key_states['KEY_RIGHT'] = False

#def LQR


# Open window
while not glfw.window_should_close(window):
    glfw.set_key_callback(window, keyboard_callback)

    # get current time
    c_time = time.perf_counter()
    frame_time = c_time - time_prev
    time_prev = c_time
    elapsed_time += frame_time


    # simulation loop
    while elapsed_time >= sim_dt:
        
        # default force vector (nothing happens)
        force = np.zeros(6)
        velocity = np.zeros(6)

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
        if key_states['KEY_UP']:
            velocity[0] = 5
        if key_states['KEY_DOWN']:
            velocity[0] = -5
        if key_states['KEY_LEFT']:
            velocity[1] = 5
        if key_states['KEY_RIGHT']:
            velocity[1] = -5

        # applies force vector 
        data.xfrc_applied[body_id] = force
        data.xpos[body_id]

        state_ball = np.array
        ([
            data.xpos[body_id][0],  
            data.xpos[body_id][1],   
            data.cvel[body_id][0], 
            data.cvel[body_id][1]   
        ])

        # Head state: [x, y, vx, vy]
        state_head = np.array([
            data.xpos[head_id][0],
            data.xpos[head_id][1],
            data.cvel[head_id][0],
            data.cvel[head_id][1]
        ])

        print("Force:", force, "State_Head:", state_head, "State_Body", state_ball)

        # update physics sim
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


