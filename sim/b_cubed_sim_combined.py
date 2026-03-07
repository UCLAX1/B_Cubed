"""b_cubed_sim_combined.py

Combined simulation: wheel/body movement from `b_cubed_moving.py` plus
head/arm preset action animation logic from `b_cubed_sim_fixed.py`.

Features:
 - Uses moving model (bb8_hl.xml) with free joint for full body translation/rotation
 - Supports wheel/body actuators (w1..w4, bd/bf/br if present) via incremental key inputs (WASD + Q/E)
 - Supports manual joint position control (I/J for a1, O/K for a2, P/L for head)
 - Head always points in direction of movement (unless manually controlled within last 1 second)
 - Supports animated head action presets (number keys 1-8) via `preset_actions.update_preset_actions`
 - Camera controls retained (arrow keys, PageUp/Down, Home)

Run:
  python sim/b_cubed_sim_combined.py

Key Reference (from existing keyboard_control mapping):
  Movement: W/S forward/back, A/D left/right strafe, Q/E rotate
  Joints: I/J a1 +/- ; O/K a2 +/- ; P/L head +/-
  Presets: 1 idle, 2 fast, 3 fast turn, 4 inquisitive, 5 head shake,
           6 head spin, 7 hurt, 8 360° spin
  Camera: Arrow keys, PageUp/PageDown (zoom), Home (reset)

Notes:
 - Preset actions stop all movement when started and override joint targets while active; manual adjustments resume after completion.
 - Body actuator values (body_x/y/r_speed) are large forces; adjust keyboard_control.py for different scaling.
"""

from mujoco.glfw import glfw
import mujoco as mj
import time
import os
import sys
import numpy as np

# Ensure root directory in path for relative imports
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from preset_actions import update_preset_actions
from input_management import update_input_matrix

# Load moving model (has freejoint + wheels + head joints)
model_path = os.path.join(ROOT_DIR, "urdf", "bb8_hl.xml")
model = mj.MjModel.from_xml_path(model_path)
data = mj.MjData(model)

# Camera setup
cam = mj.MjvCamera()
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1
opt = mj.MjvOption()

# Identify free joint qpos region for camera tracking
base_link_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base_link")
base_link_joint_id = model.body_jntadr[base_link_body_id]
qpos_addr = model.jnt_qposadr[base_link_joint_id]

# Actuator IDs (some may be -1 if missing; guard assignments)
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

print(
    f"[INIT] Wheel actuators: w1={w1_motor_id}, w2={w2_motor_id}, w3={w3_motor_id}, w4={w4_motor_id}"
)
print(
    f"[INIT] Body actuators: x={body_x_id}, y={body_y_id}, r={body_r_id}; Joint motors: a1={a1_motor_id}, a2={a2_motor_id}, h={h_motor_id}"
)

# Create window & scene
glfw.init()
window = glfw.create_window(720, 540, "B_Cubed Combined", None, None)
glfw.make_context_current(window)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)


# Timing
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


        # Shared input matrix update (non-critical if missing)
        try:
            update_input_matrix()
        except Exception:
            pass

        mj.mj_step(model, data)
        elapsed_time -= sim_dt

    # Rendering
    if c_time - prev_render_t >= frame_dt:
        prev_render_t = c_time
        # Follow free joint position
        cam.lookat[0:3] = data.qpos[qpos_addr : qpos_addr + 3]
        viewport_w, viewport_h = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_w, viewport_h)
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)
        glfw.swap_buffers(window)
        glfw.poll_events()

glfw.terminate()
