import os
import time
import numpy as np
from mujoco.glfw import glfw
import mujoco as mj
from head_balance_math import find_motor_angles

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODEL_PATH = os.path.join(ROOT_DIR, "urdf", "imu_head_balance.xml")

def main():
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data = mj.MjData(model)

    a1_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a1_motor")
    a2_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "a2_motor")

    glfw.init()
    window = glfw.create_window(720, 540, "Head Movement Test", None, None)
    glfw.make_context_current(window)
    cam = mj.MjvCamera()
    cam.distance = 2
    cam.azimuth = 45
    cam.elevation = -35
    cam.orthographic = 1
    opt = mj.MjvOption()
    mj.mjv_defaultOption(opt)
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    sim_dt = 0.001
    elapsed_time = 0.0
    time_prev = time.perf_counter()
    target_fps = 60
    frame_dt = 1.0 / target_fps
    prev_render_t = time_prev
    t = 0.0

    while not glfw.window_should_close(window):
        c_time = time.perf_counter()
        frame_time = c_time - time_prev
        time_prev = c_time
        elapsed_time += frame_time
        
        while elapsed_time >= sim_dt:
            t += sim_dt
            # Simulate pitch and roll as sinusoids
            pitch = 20 * np.sin(t)   # degrees
            roll = 20 * np.cos(t)    # degrees
            arm, lazy_susan = find_motor_angles(pitch, roll)
            if a1_id != -1:
                data.ctrl[a1_id] = lazy_susan * np.pi/180
            if a2_id != -1:
                data.ctrl[a2_id] = arm * np.pi/180
            mj.mj_step(model, data)
            elapsed_time -= sim_dt

        if c_time - prev_render_t >= frame_dt:
            prev_render_t = c_time
            cam.lookat[0:3] = data.qpos[0:3]
            viewport_w, viewport_h = glfw.get_framebuffer_size(window)
            viewport = mj.MjrRect(0, 0, viewport_w, viewport_h)
            mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
            mj.mjr_render(viewport, scene, context)
            glfw.swap_buffers(window)
            glfw.poll_events()
    glfw.terminate()

if __name__ == "__main__":
    main()
