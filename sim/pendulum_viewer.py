import mujoco as mj
from mujoco.glfw import glfw
import numpy as np

# This viewer demonstrates:
# - Loading a MuJoCo model from XML (pendulum.xml)
# - Using keyboard to apply torque to the pendulum
# - Accessing joint/actuator IDs and simulation data (qpos, qvel)
# - Printing and visualizing state for workshop learning

modelPath = 'pendulum.xml'
displayRefreshRate = 60
torque = 0.0

def keyboard_callback(window, key, scancode, action, mods):
    global torque
    if action == glfw.PRESS or action == glfw.REPEAT:
        if key == glfw.KEY_LEFT:
            torque = -1.0  # Apply negative torque
        elif key == glfw.KEY_RIGHT:
            torque = 1.0   # Apply positive torque
        elif key == glfw.KEY_SPACE:
            torque = 0.0   # Stop torque
    elif action == glfw.RELEASE:
        if key in [glfw.KEY_LEFT, glfw.KEY_RIGHT]:
            torque = 0.0

# Load model and data
model = mj.MjModel.from_xml_path(modelPath)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()

# Get joint and actuator IDs by name (from XML)
joint_name = model.joint(0).name if model.njnt > 0 else None
joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name) if joint_name else -1
actuator_name = model.actuator(0).name if model.nu > 0 else None
actuator_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, actuator_name) if actuator_name else -1

print(f"Loaded pendulum.xml. Joint name: {joint_name}, id: {joint_id}")
print(f"Actuator name: {actuator_name}, id: {actuator_id}")

# Initialize GLFW and window
if not glfw.init():
    raise RuntimeError('Could not initialize GLFW')
window = glfw.create_window(600, 400, "Pendulum Viewer", None, None)
glfw.make_context_current(window)
glfw.set_key_callback(window, keyboard_callback)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=1000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

cam.distance = 2
cam.elevation = -20
cam.azimuth = 90

while not glfw.window_should_close(window):
    # Apply torque to actuator (if present)
    if actuator_id != -1:
        data.ctrl[actuator_id] = torque
    
    # Step simulation
    mj.mj_step(model, data)
    
    # Print qpos and qvel for the pendulum joint
    if joint_id != -1:
        qpos = data.qpos[model.jnt_qposadr[joint_id]]
        qvel = data.qvel[model.jnt_dofadr[joint_id]]
        print(f"qpos (angle): {qpos:.3f}, qvel (angular vel): {qvel:.3f}", end='\r')
    
    # Render
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    cam.lookat[:] = 0  # Center camera
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
