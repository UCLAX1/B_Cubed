import mujoco
import mujoco.viewer
import time
import glfw
import numpy as np

model = mujoco.MjModel.from_xml_path("hello.xml")
data = mujoco.MjData(model)

RADIUS_DIFFERENCE = 0.25

# CAR_ID = model.geom('car').id
# car_mass = model.geom('car').mass[0]
# shell_mass = model.geom('shell').mass[0]
# total_mass = car_mass + shell_mass

STEP = 0.001

theta = 0.0
phi = -np.pi

def key_callback(keycode):
    global theta, phi
    if keycode == 262: # right arrow
        theta -= STEP
    if keycode == 263: # left arrow
        theta += STEP
    if keycode == 265: # up arrow
        phi -= STEP
    if keycode == 264: # down arrow
        phi += STEP
    # if chr(keycode) == 'D':
    #     input_force[0] = INPUT_FORCE_VAL
    # if chr(keycode) == 'A':
    #     input_force[0] = -INPUT_FORCE_VAL
    # if chr(keycode) == 'W':
    #     input_force[1] = -INPUT_FORCE_VAL
    # if chr(keycode) == 'S':
    #     input_force[1] = INPUT_FORCE_VAL

# Launch interactive viewer
# mujoco.viewer.launch(model, data)

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    start = time.time()
    while viewer.is_running():
        step_start = time.time()

        new_pos = RADIUS_DIFFERENCE * np.array([np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)])
        # model.geom_pos[CAR_ID] = new_pos

        # data.ctrl[0] = input_force[0]
        # data.ctrl[1] = input_force[1]

        mujoco.mj_step(model, data)

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)