from mujoco.glfw import glfw
import mujoco as mj
import time
import keyboard_control
# import sim.enums as states
from control_state import state

# load model & set up data and camera
modelPath = "b_cubed.xml"
model = mj.MjModel.from_xml_path(modelPath)
data = mj.MjData(model)

# visualization settings
cam = mj.MjvCamera()
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1

opt = mj.MjvOption()

# define bb8 joint
joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "bb8_free")
qpos_addr = model.jnt_qposadr[joint_id]

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

        # do stuff here



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