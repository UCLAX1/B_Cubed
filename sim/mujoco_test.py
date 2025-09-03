import mujoco as mj
from mujoco.glfw import glfw
import numpy as np

"""
Motor Control System for BB8 Robot

This system controls the robot through 4 tilted axes instead of direct x,y,z rotation.
The four motor axes are fixed relative to the world frame:

Motor 1: x-z plane, +x direction, 45° from z-axis
Motor 2: x-z plane, -x direction, 45° from z-axis  
Motor 3: y-z plane, +y direction, 45° from z-axis
Motor 4: y-z plane, -y direction, 45° from z-axis

Keyboard Controls:
Multi-Key Movement (Superposition):
- W: Forward
- A: Left  
- S: Back
- D: Right
- Combinations: W+D (forward-right), W+A (forward-left), S+D (back-right), S+A (back-left), etc.

Individual Motor Controls:
- 1: Motor 1 only (x-z plane, +x direction)
- 2: Motor 2 only (x-z plane, -x direction)  
- 3: Motor 3 only (y-z plane, +y direction)
- 4: Motor 4 only (y-z plane, -y direction)
- R: Reset - stop all motors

Camera Controls:
- Arrow Keys: Move camera (Up/Down/Left/Right)
- Page Up/Page Down: Zoom In/Zoom Out
- Home: Reset camera to default position

Usage examples:
- set_motor_speed(1, 2.0)  # Spin motor 1 at 2 rad/s
- motor_1_speed(1.5)       # Convenience wrapper for motor 1
- set_motor_speeds({1: 1.0, 3: -0.5})  # Multiple motors simultaneously
- set_motor_speeds([1.0, 0.0, -0.5, 0.2])  # List format for all 4 motors
"""


print_camera_config = 1 #set to 1 to print camera config
                        #this is useful for initializing view of the model

modelPath = "sim/test_world.xml"
displayRefreshRate = 60


keyBoardControl = 'none'
cameraControl = 'none'

# Multi-key control system for diagonal movement
key_states = {
    'W': False,  # Forward
    'A': False,  # Left
    'S': False,  # Back
    'D': False   # Right
}

def update_movement_from_keys():
    """Calculate and apply motor speeds based on currently pressed keys."""
    # Calculate movement components based on pressed keys
    forward_back = 0.0
    left_right = 0.0
    
    if key_states['W']:  # Forward
        forward_back += 1.0
    if key_states['S']:  # Back
        forward_back -= 1.0
    if key_states['D']:  # Right
        left_right += 1.0
    if key_states['A']:  # Left
        left_right -= 1.0
    
    # Apply combined movement to motors
    motor_speeds = {
        1: left_right,   # Motor 1: positive for right
        2: -left_right,  # Motor 2: negative for right
        3: forward_back, # Motor 3: positive for forward
        4: -forward_back # Motor 4: negative for forward
    }
    
    set_motor_speeds(motor_speeds)
    
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
    elif action == glfw.RELEASE:
        if key == glfw.KEY_W:
            key_states['W'] = False
        elif key == glfw.KEY_A:
            key_states['A'] = False
        elif key == glfw.KEY_S:
            key_states['S'] = False
        elif key == glfw.KEY_D:
            key_states['D'] = False
    
    # Handle other keys (only on press/repeat)
    if action == glfw.PRESS or action == glfw.REPEAT:
        if key == glfw.KEY_X:  # Neutral position or stop
            keyBoardControl = 'nut'
        elif key == glfw.KEY_R:  # Reset - stop all motors
            keyBoardControl = 'reset'
        elif key == glfw.KEY_1:  # Motor 1 only
            keyBoardControl = 'motor1'
        elif key == glfw.KEY_2:  # Motor 2 only
            keyBoardControl = 'motor2'
        elif key == glfw.KEY_3:  # Motor 3 only
            keyBoardControl = 'motor3'
        elif key == glfw.KEY_4:  # Motor 4 only
            keyBoardControl = 'motor4'
        elif key == glfw.KEY_UP:
            cameraControl = 'up'
            print("Key pressed: UP (Camera Up)")
        elif key == glfw.KEY_DOWN:
            cameraControl = 'down'
            print("Key pressed: DOWN (Camera Down)")
        elif key == glfw.KEY_RIGHT:
            cameraControl = 'right'
            print("Key pressed: RIGHT (Camera Right)")
        elif key == glfw.KEY_LEFT:
            cameraControl = 'left'
            print("Key pressed: LEFT (Camera Left)")
        elif key == glfw.KEY_PAGE_UP:
            cameraControl = 'zoom_in'
            print("Key pressed: PAGE_UP (Zoom In)")
        elif key == glfw.KEY_PAGE_DOWN:
            cameraControl = 'zoom_out'
            print("Key pressed: PAGE_DOWN (Zoom Out)")
        elif key == glfw.KEY_HOME:
            cameraControl = 'reset_camera'
            print("Key pressed: HOME (Reset Camera)")
        elif key == glfw.KEY_DELETE:
            cameraControl = 'home'


# MuJoCo data structures
model = mj.MjModel.from_xml_path(modelPath)  # MuJoCo model
data = mj.MjData(model)                     # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options



sphere = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "bb8")
debug = False


# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(int(1200*0.6), int(900*0.6), "Quadruped", None, None)
glfw.make_context_current(window)
# glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard_callback)

# Default camera settings
cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1

# sphere_address = model.jnt_qposadr[sphere]  # Get qpos index for the cube joint

joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "bb8_free")
qpos_addr = model.jnt_qposadr[joint_id]

head_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "head")
# head_site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "head_site")

# Define the four tilted motor axes (fixed relative to world frame)
# Two axes in x-z plane, 45° from z-axis
motor_axis_1 = np.array([1.0, 0.0, 1.0]) / np.sqrt(2.0)    # +x direction, 45° from z
motor_axis_2 = np.array([-1.0, 0.0, 1.0]) / np.sqrt(2.0)   # -x direction, 45° from z
# Two axes in y-z plane, 45° from z-axis  
motor_axis_3 = np.array([0.0, 1.0, 1.0]) / np.sqrt(2.0)    # +y direction, 45° from z
motor_axis_4 = np.array([0.0, -1.0, 1.0]) / np.sqrt(2.0)   # -y direction, 45° from z

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
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
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
        speeds = {i+1: speeds[i] for i in range(4)}
    
    # Get the current orientation of the sphere (quaternion)
    sphere_quat = data.qpos[qpos_addr + 3:qpos_addr + 7]  # [w, x, y, z]
    
    # Convert quaternion to rotation matrix
    w, x, y, z = sphere_quat
    rotation_matrix = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
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
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    
    # Transform body angular velocity to world frame
    world_angular_vel = rotation_matrix @ body_angular_vel
    world_angular_speed = np.linalg.norm(world_angular_vel)
    
    # Calculate rotation axis in world frame (normalized angular velocity vector)
    world_rotation_axis = world_angular_vel / world_angular_speed
    
    # Print detailed information
    print(f"\n=== Rotation Axis Analysis ===")
    print(f"Body angular velocity: [{body_angular_vel[0]:.4f}, {body_angular_vel[1]:.4f}, {body_angular_vel[2]:.4f}] rad/s")
    print(f"World angular velocity: [{world_angular_vel[0]:.4f}, {world_angular_vel[1]:.4f}, {world_angular_vel[2]:.4f}] rad/s")
    print(f"Angular speed: {world_angular_speed:.4f} rad/s")
    print(f"World rotation axis: [{world_rotation_axis[0]:.4f}, {world_rotation_axis[1]:.4f}, {world_rotation_axis[2]:.4f}]")
    
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

while not glfw.window_should_close(window):
    time_prev = data.time
    while data.time - time_prev < 1.0 / displayRefreshRate:

        # Handle multi-key movement (WASD combinations)
        if any(key_states.values()):
            update_movement_from_keys()
        # Handle individual motor controls and other commands
        elif keyBoardControl == "motor1":
            # Key '1' - Motor 1 only (x-z plane, +x direction, 45° from z)
            motor_1_speed(0.5)
        elif keyBoardControl == "motor2":
            # Key '2' - Motor 2 only (x-z plane, -x direction, 45° from z)
            motor_2_speed(0.5)
        elif keyBoardControl == "motor3":
            # Key '3' - Motor 3 only (y-z plane, +y direction, 45° from z)
            motor_3_speed(0.5)
        elif keyBoardControl == "motor4":
            # Key '4' - Motor 4 only (y-z plane, -y direction, 45° from z)
            motor_4_speed(0.5)
        elif keyBoardControl == "reset":
            # Key 'R' - Reset: stop all motors
            set_motor_speeds({1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
        else:
            # No input - stop all motors
            set_motor_speeds({1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})

        # Analyze and print rotation axis after motor commands
        if (any(key_states.values()) or keyBoardControl in ['motor1', 'motor2', 'motor3', 'motor4']) and debug:
            calculate_and_print_rotation_axis()

        # Handle camera controls
        if cameraControl == 'up':
            cam.elevation += 2  # Move camera up
            cameraControl = 'none'  # Reset after single adjustment
        elif cameraControl == 'down':
            cam.elevation -= 2  # Move camera down
            cameraControl = 'none'
        elif cameraControl == 'left':
            cam.azimuth -= 2  # Rotate camera left
            cameraControl = 'none'
        elif cameraControl == 'right':
            cam.azimuth += 2  # Rotate camera right
            cameraControl = 'none'
        elif cameraControl == 'zoom_in':
            cam.distance = max(0.5, cam.distance - 0.1)  # Zoom in (min distance 0.5)
            cameraControl = 'none'
        elif cameraControl == 'zoom_out':
            cam.distance = min(10.0, cam.distance + 0.1)  # Zoom out (max distance 10)
            cameraControl = 'none'
        elif cameraControl == 'reset_camera':
            # Reset camera to default position
            cam.distance = 2
            cam.azimuth = 45
            cam.elevation = -35
            cameraControl = 'none'
            print("Camera reset to default position")

     # Get the site ID once (do this outside the loop)

        # # Inside your render loop:
        bb8_pos = data.qpos[qpos_addr : qpos_addr + 3]  # ball's world position
        new_head_pos = bb8_pos + np.array([0.0, 0.0, 0.01])  # 0.01m above ball center
        # # data.qpos[head_id] = new_head_pos[0]

        #         # Use MuJoCo API to set body position
        # mj.mj_set_body_xpos(model, data, head_id, new_head_pos)

        head_bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "head")
        data.mocap_pos[head_bid] = new_head_pos
        
        # MuJoCo doesn't let you directly set site_xpos, so you need to:
        # (1) define a mocap body, or
        # (2) draw a marker at the updated position each frame
        # (3) or manually move the site body using a 'head' body instead

        # Step the MuJoCo simulation
        mj.mj_step(model, data)
        glfw.poll_events()


    # Handle rendering
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Camera updates (tracking robot position)
    cam.lookat[0:3] = data.qpos[qpos_addr : qpos_addr + 3]  # [x, y, z]

        # Move first light to 2m above the sphere
    # model.light_pos[0][:3] = data.qpos[qpos_addr : qpos_addr + 3] + np.array([0.0, 0.0, 2.0])



    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # Swap buffers
    glfw.swap_buffers(window)

glfw.terminate()