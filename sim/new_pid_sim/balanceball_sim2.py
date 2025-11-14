from mujoco.glfw import glfw
import numpy as np
import mujoco as mj
import time

import random



# IMPORTANT DISCLAIMER: THIS WAS ALL WRITTEN BY CHATGPT I HAVE NO IDEA WHAT ITS DOING


# ---------------------------
#  Model + Visualization setup
# ---------------------------
modelPath = "balanceball2.xml"
model = mj.MjModel.from_xml_path(modelPath)
body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "bb8")
head_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "head")
data = mj.MjData(model)

cam = mj.MjvCamera()
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1
opt = mj.MjvOption()

# define bb8 joint
joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "bb8_free")
qpos_addr = model.jnt_qposadr[joint_id]

# ---------------------------
#  Window setup
# ---------------------------
glfw.init()
window = glfw.create_window(720, 540, "Balanceball", None, None)
glfw.make_context_current(window)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# ---------------------------
#  Timing
# ---------------------------
sim_dt = 0.001
elapsed_time = 0.0
time_prev = time.perf_counter()
target_fps = 60
frame_dt = 1.0 / target_fps
prev_render_t = time_prev

# ---------------------------
#  Keyboard state
# ---------------------------
key_states = {k: False for k in ['W','A','S','D','1','2','3','4']}

def keyboard_callback(window, key, scancode, action, mods):
    if key in (glfw.KEY_W, glfw.KEY_A, glfw.KEY_S, glfw.KEY_D,
               glfw.KEY_1, glfw.KEY_2, glfw.KEY_3, glfw.KEY_4):
        pressed = (action == glfw.PRESS or action == glfw.REPEAT)
        if   key == glfw.KEY_W: key_states['W'] = pressed
        elif key == glfw.KEY_A: key_states['A'] = pressed
        elif key == glfw.KEY_S: key_states['S'] = pressed
        elif key == glfw.KEY_D: key_states['D'] = pressed
        elif key == glfw.KEY_1: key_states['1'] = pressed
        elif key == glfw.KEY_2: key_states['2'] = pressed
        elif key == glfw.KEY_3: key_states['3'] = pressed
        elif key == glfw.KEY_4: key_states['4'] = pressed

glfw.set_key_callback(window, keyboard_callback)

# ---------------------------
#  PID parameters
# ---------------------------
Kp = 5
Ki = 0.3
Kd = 0.2
I_MAX = 0.3

# desired roll-rate cap (interpreted as angular speed command proxy)
OMEGA_MAX = 4

# ---------------------------
#  PID controller
# ---------------------------
def PID(error, prev_error, int_error, Kp, Ki, Kd, dt, I_MAX):
    d_error = (error - prev_error) / dt
    new_int = np.clip(int_error + error * dt, -I_MAX, I_MAX)
    control = -(Kp * error + Ki * new_int + Kd * d_error)
    return control, new_int

# ---------------------------
#  Actuator mixing (4 motors @ 0°, 90°, 180°, 270° around +Z)
#  Each motor command = [cosθ, sinθ] · [cmd_x, cmd_y]
#  If a motor spins the opposite way physically, flip its sign in MOTOR_SIGN.
# ---------------------------
angles = np.array([0.0, np.pi/2, np.pi, 3*np.pi/2])   # front, right, back, left
MIX = np.vstack((np.cos(angles), np.sin(angles))).T    # shape (4,2)
MOTOR_SIGN = np.array([1.0, 1.0, 1.0, 1.0])           # tweak signs if any motor is reversed
# Per-actuator ctrlrange from XML for safe clamping:
CTRL_MIN = model.actuator_ctrlrange[:4, 0]
CTRL_MAX = model.actuator_ctrlrange[:4, 1]

# ---------------------------
#  Main loop
# ---------------------------
prev_error = np.zeros(2)
int_error = np.zeros(2)
time_sum = 0.0

# Random perturbation scheduling
next_perturb_time = 0.5
perturb_duration = 0.4
active_perturb_end = 0.0
force_mag = 50.0

while not glfw.window_should_close(window):

    c_time = time.perf_counter()
    frame_time = c_time - time_prev
    time_prev = c_time
    elapsed_time += frame_time
    time_sum += frame_time

    while elapsed_time >= sim_dt:

        # --- Get positions ---
        ball_pos = data.xpos[body_id].copy()
        head_pos = data.xpos[head_id].copy()

        # --- Compute horizontal error (head above ball center) ---
        error_xy = head_pos[:2] - ball_pos[:2]

        # --- PID → desired roll vector (cmd_x, cmd_y) ---

        c_time = time.perf_counter()
        frame_time = c_time - time_prev
        time_prev = c_time
        elapsed_time += frame_time
        time_sum += frame_time
        cmd_xy = np.zeros(2)
        if time_sum >= 1.0:
            cmd_xy, int_error = PID(error_xy, prev_error, int_error, Kp, Ki, Kd, sim_dt, I_MAX)
            prev_error = error_xy

            # cap desired roll rate amplitude
            norm = np.linalg.norm(cmd_xy)
            if norm > OMEGA_MAX:
                cmd_xy *= (OMEGA_MAX / (norm + 1e-9))

            # keyboard nudges (adds to desired roll vector)
            if key_states['W']: cmd_xy[0] += 0.5
            if key_states['S']: cmd_xy[0] -= 0.5
            if key_states['A']: cmd_xy[1] += 0.5
            if key_states['D']: cmd_xy[1] -= 0.5

            # --- Mix to 4 motors ---
            raw_ctrl = (MIX @ cmd_xy) * MOTOR_SIGN  # shape (4,)

            # clamp per actuator using XML ctrlrange
            data.ctrl[0:4] = np.minimum(np.maximum(raw_ctrl, CTRL_MIN), CTRL_MAX)

        # --- Perturbation test forces ---
        force = np.zeros(6)
        if 0.5 <= time_sum <= 1.0:
            force[0] = 5.0  # initial shove

        if time_sum >= next_perturb_time:
            direction = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
            force[0] = direction[0] * force_mag
            force[1] = direction[1] * force_mag
            active_perturb_end = time_sum + perturb_duration
            next_perturb_time = time_sum + random.uniform(1.0, 5.0)
            force_mag = random.randint(25,100)
            print(f"Random push {force_mag} at {time_sum:.2f}s: dir={direction}")

        if time_sum < active_perturb_end:
            data.xfrc_applied[body_id] = force
        else:
            data.xfrc_applied[body_id, :] = 0.0

        mj.mj_step(model, data)
        elapsed_time -= sim_dt

    # --- Rendering ---
    if c_time - prev_render_t >= frame_dt:
        prev_render_t = c_time
        cam.lookat[0:3] = data.qpos[qpos_addr : qpos_addr + 3]
        w, h = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, w, h)
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)
        glfw.swap_buffers(window)
        glfw.poll_events()

glfw.terminate()
