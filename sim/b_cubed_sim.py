from mujoco.glfw import glfw
import mujoco as mj
import time
import keyboard_control
from control_state import state
from sim.enums import BotControl
import numpy as np

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

# Han's motor magic
motor_axis_1 = np.array([1.0, 0.0, 1.0]) / np.sqrt(2.0)  # +x direction, 45° from z
motor_axis_2 = np.array([-1.0, 0.0, 1.0]) / np.sqrt(2.0)  # -x direction, 45° from z
# Two axes in y-z plane, 45° from z-axis
motor_axis_3 = np.array([0.0, 1.0, 1.0]) / np.sqrt(2.0)  # +y direction, 45° from z
motor_axis_4 = np.array([0.0, -1.0, 1.0]) / np.sqrt(2.0)  # -y direction, 45° from z


def set_motor_speed(motor_id, speed):
    """
    Set the speed of a specific motor (1-4) that controls rotation about one of the tilted axes.

    Args:
        motor_id (int): Motor number (1, 2, 3, or 4)
        speed (float): Rotation speed about the motor's axis (rad/s)
    """
    global data, qpos_addr

    # Clear existing angular velocities
    data.qvel[qpos_addr + 3:qpos_addr + 6] = 0

    if motor_id == 1:
        world_axis = motor_axis_1
    elif motor_id == 2:
        world_axis = motor_axis_2
    elif motor_id == 3:
        world_axis = motor_axis_3
    elif motor_id == 4:
        world_axis = motor_axis_4
    else:
        print(f"Invalid motor_id: {motor_id}. Must be 1, 2, 3, or 4.")
        return

    # Get the current orientation of the sphere (quaternion)
    sphere_quat = data.qpos[qpos_addr + 3:qpos_addr + 7]  # [w, x, y, z]

    # Convert quaternion to rotation matrix
    w, x, y, z = sphere_quat
    rotation_matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
    ])

    # Transform world axis to body frame: body_axis = R^T * world_axis
    body_axis = rotation_matrix.T @ world_axis

    # Set angular velocity in body frame
    data.qvel[qpos_addr + 3:qpos_addr + 6] = speed * body_axis


def set_motor_speeds(speeds):
    """
    Set speeds for multiple motors simultaneously by combining their effects.

    Args:
        speeds (dict or list): Motor speeds. If dict, keys are motor_ids (1-4), values are speeds.
                              If list, should have 4 elements corresponding to motors 1-4.
    """
    global data, qpos_addr

    # Convert list to dict if needed
    if isinstance(speeds, list):
        if len(speeds) != 4:
            print("Speed list must have exactly 4 elements for motors 1-4")
            return
        speeds = {i + 1: speeds[i] for i in range(4)}

    # Get the current orientation of the sphere (quaternion)
    sphere_quat = data.qpos[qpos_addr + 3:qpos_addr + 7]  # [w, x, y, z]

    # Convert quaternion to rotation matrix
    w, x, y, z = sphere_quat
    rotation_matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
    ])

    # Combine effects of all motors in world frame, then transform to body frame
    combined_world_angular_vel = np.zeros(3)
    for motor_id, speed in speeds.items():
        if motor_id == 1:
            combined_world_angular_vel += speed * motor_axis_1
        elif motor_id == 2:
            combined_world_angular_vel += speed * motor_axis_2
        elif motor_id == 3:
            combined_world_angular_vel += speed * motor_axis_3
        elif motor_id == 4:
            combined_world_angular_vel += speed * motor_axis_4
        else:
            print(f"Invalid motor_id: {motor_id}. Must be 1, 2, 3, or 4.")

    # Transform combined world angular velocity to body frame
    body_angular_vel = rotation_matrix.T @ combined_world_angular_vel

    # Apply the combined angular velocity to the body frame
    data.qvel[qpos_addr + 3:qpos_addr + 6] = body_angular_vel


def motor_1_speed(speed):
    """Wrapper for motor 1 (x-z plane, +x direction, 45° from z)"""
    set_motor_speed(1, speed)


def motor_2_speed(speed):
    """Wrapper for motor 2 (x-z plane, -x direction, 45° from z)"""
    set_motor_speed(2, speed)


def motor_3_speed(speed):
    """Wrapper for motor 3 (y-z plane, +y direction, 45° from z)"""
    set_motor_speed(3, speed)


def motor_4_speed(speed):
    """Wrapper for motor 4 (y-z plane, -y direction, 45° from z)"""
    set_motor_speed(4, speed)


def calculate_and_print_rotation_axis():
    """
    Calculate and print the current rotation axis based on angular velocity.
    Returns the normalized rotation axis vector and angular speed.
    """
    global data, qpos_addr

    # Get current angular velocity in body frame
    body_angular_vel = data.qvel[qpos_addr + 3:qpos_addr + 6]
    body_angular_speed = np.linalg.norm(body_angular_vel)

    if body_angular_speed < 0.001:  # Very small rotation
        print("No significant rotation (angular speed < 0.001 rad/s)")
        return np.array([0, 0, 0]), 0.0

    # Get the current orientation of the sphere (quaternion)
    sphere_quat = data.qpos[qpos_addr + 3:qpos_addr + 7]  # [w, x, y, z]

    # Convert quaternion to rotation matrix
    w, x, y, z = sphere_quat
    rotation_matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
    ])

    # Transform body angular velocity to world frame
    world_angular_vel = rotation_matrix @ body_angular_vel
    world_angular_speed = np.linalg.norm(world_angular_vel)

    # Calculate rotation axis in world frame (normalized angular velocity vector)
    world_rotation_axis = world_angular_vel / world_angular_speed

    # Print detailed information
    print(f"\n=== Rotation Axis Analysis ===")
    print(
        f"Body angular velocity: [{body_angular_vel[0]:.4f}, {body_angular_vel[1]:.4f}, {body_angular_vel[2]:.4f}] rad/s")
    print(
        f"World angular velocity: [{world_angular_vel[0]:.4f}, {world_angular_vel[1]:.4f}, {world_angular_vel[2]:.4f}] rad/s")
    print(f"Angular speed: {world_angular_speed:.4f} rad/s")
    print(
        f"World rotation axis: [{world_rotation_axis[0]:.4f}, {world_rotation_axis[1]:.4f}, {world_rotation_axis[2]:.4f}]")

    # Analyze which motor axis this most closely matches
    motor_axes = [motor_axis_1, motor_axis_2, motor_axis_3, motor_axis_4]
    motor_names = ["Motor 1 (x-z, +x)", "Motor 2 (x-z, -x)", "Motor 3 (y-z, +y)", "Motor 4 (y-z, -y)"]

    best_match_idx = -1
    best_dot_product = -1

    for i, motor_axis in enumerate(motor_axes):
        dot_product = abs(np.dot(world_rotation_axis, motor_axis))
        print(f"Similarity to {motor_names[i]}: {dot_product:.4f}")

        if dot_product > best_dot_product:
            best_dot_product = dot_product
            best_match_idx = i

    if best_match_idx != -1 and best_dot_product > 0.9:
        print(f"Closest match: {motor_names[best_match_idx]} (similarity: {best_dot_product:.4f})")
    else:
        print("Rotation axis is a combination of multiple motor axes")

    print("==============================\n")

    return world_rotation_axis, world_angular_speed


# keyBoardControl = keyboard_callback();


def update_movement_from_keys():
    """Calculate and apply motor speeds based on currently pressed keys."""
    # Calculate movement components based on pressed keys
    # print('this happendd')
    forward_back = 0.0
    left_right = 0.0

    if state.fb_control == BotControl.FWD:  # Forward
        # print('FWD wow')
        forward_back += 1.0
    if state.fb_control == BotControl.BACK: # Back
        forward_back -= 1.0
    if state.rl_control == BotControl.RIGHT:  # Right
        left_right += 1.0
    if state.rl_control == BotControl.LEFT:  # Left
        left_right -= 1.0

    # Apply combined movement to motors
    motor_speeds = {
        1: left_right,  # Motor 1: positive for right
        2: -left_right,  # Motor 2: negative for right
        3: forward_back,  # Motor 3: positive for forward
        4: -forward_back  # Motor 4: negative for forward
    }

    set_motor_speeds(motor_speeds)

# Open window
while not glfw.window_should_close(window):
    # get current time
    c_time = time.perf_counter()
    frame_time = c_time - time_prev
    time_prev = c_time
    elapsed_time += frame_time

    # simulation loop
    while elapsed_time >= sim_dt:
        # print(state.rl_control)

        # do stuff here
        update_movement_from_keys()




        # Updates head positions
        bb8_pos = data.qpos[qpos_addr : qpos_addr + 3]  # ball's world position
        new_head_pos = bb8_pos + np.array([0.0, 0.0, 0.01])  # 0.01m above ball center

        head_bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "head")
        data.mocap_pos[head_bid] = new_head_pos

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