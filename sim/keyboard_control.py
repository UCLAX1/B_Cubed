import glfw
from control_state import state
from enums import BotControl
from enums import CameraControl
from enums import AngularVelocityControl
from enums import JointControl



def keyboard_callback(window, key, scancode, action, mods):
    if action == glfw.PRESS or action == glfw.REPEAT:  # Handle key press or hold

        print("Keyboard pressed")

        # Robot Control

        # Forwards and Backwards Control - increment wheel speeds by 0.1
        # Forward: w1 positive, w3 negative
        if key == glfw.KEY_W: 
            state.w1_speed += 0.1
            state.w3_speed -= 0.1
            # Clamp to [-1, 1] range
            state.w1_speed = max(-1.0, min(1.0, state.w1_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            print(f"Forward: w1={state.w1_speed:.2f}, w3={state.w3_speed:.2f}")
        # Backward: w1 negative, w3 positive
        elif key == glfw.KEY_S: 
            state.w1_speed -= 0.1
            state.w3_speed += 0.1
            # Clamp to [-1, 1] range
            state.w1_speed = max(-1.0, min(1.0, state.w1_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            print(f"Backward: w1={state.w1_speed:.2f}, w3={state.w3_speed:.2f}")

        #Left and Right Control - increment wheel speeds by 0.1
        # Left: w2 positive, w3 negative
        if key == glfw.KEY_A: 
            state.w2_speed += 0.1
            state.w3_speed -= 0.1
            # Clamp to [-1, 1] range
            state.w2_speed = max(-1.0, min(1.0, state.w2_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            print(f"Left: w2={state.w2_speed:.2f}, w3={state.w3_speed:.2f}")
        # Right: w2 negative, w3 positive
        elif key == glfw.KEY_D: 
            state.w2_speed -= 0.1
            state.w3_speed += 0.1
            # Clamp to [-1, 1] range
            state.w2_speed = max(-1.0, min(1.0, state.w2_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            print(f"Right: w2={state.w2_speed:.2f}, w3={state.w3_speed:.2f}")

        # Neutral Key (Note: X is now used for a2 joint decrease, consider using a different key for neutral)
        # TODO: Reassign neutral key if needed, or remove this if X should only control a2
        # if key == glfw.KEY_X:
        #     state.fb_control = BotControl.NEUTRAL
        #     state.rl_control = BotControl.NEUTRAL

        # Angular Velocity Control (Left/Right arrows for base rotation)
        if key == glfw.KEY_LEFT: 
            state.angular_vel_control = AngularVelocityControl.LEFT
        elif key == glfw.KEY_RIGHT:
            state.angular_vel_control = AngularVelocityControl.RIGHT

        # Joint Position Control (one-time increment per key press)
        # Z, X keys: a1 joint increase and decrease (0.2 increment)
        if key == glfw.KEY_Z:
            state.target_a1_pos += 0.2
            state.target_a1_pos = max(-3.14, min(3.14, state.target_a1_pos))
            print(f"a1 position: {state.target_a1_pos:.2f}")
        elif key == glfw.KEY_X:
            state.target_a1_pos -= 0.2
            state.target_a1_pos = max(-3.14, min(3.14, state.target_a1_pos))
            print(f"a1 position: {state.target_a1_pos:.2f}")
        # C, V keys: a2 joint increase and decrease (0.2 increment)
        elif key == glfw.KEY_C:
            state.target_a2_pos += 0.2
            state.target_a2_pos = max(0.7, min(1.5, state.target_a2_pos))
            print(f"a2 position: {state.target_a2_pos:.2f}")
        elif key == glfw.KEY_V:
            state.target_a2_pos -= 0.2
            state.target_a2_pos = max(0.7, min(1.5, state.target_a2_pos))
            print(f"a2 position: {state.target_a2_pos:.2f}")
        # B, N keys: h (head) joint increase and decrease (0.2 increment)
        elif key == glfw.KEY_B:
            state.target_h_pos += 0.2
            state.target_h_pos = max(-3.14, min(3.14, state.target_h_pos))
            print(f"h position: {state.target_h_pos:.2f}")
        elif key == glfw.KEY_N:
            state.target_h_pos -= 0.2
            state.target_h_pos = max(-3.14, min(3.14, state.target_h_pos))
            print(f"h position: {state.target_h_pos:.2f}")

        # Camera control (Note: Left/Right arrows are now used for angular velocity)
        # Only Up/Down arrows and other keys control camera now
        if key == glfw.KEY_UP: state.cam_control = CameraControl.UP
        elif key == glfw.KEY_DOWN: state.cam_control = CameraControl.DOWN
        elif key == glfw.KEY_PAGE_UP: state.cam_control = CameraControl.ZOOM_IN
        elif key == glfw.KEY_PAGE_DOWN: state.cam_control = CameraControl.ZOOM_OUT
        elif key == glfw.KEY_HOME: state.cam_control = CameraControl.RESET_CAMERA
        elif key == glfw.KEY_DELETE: state.cam_control = CameraControl.HOME


    elif action == glfw.RELEASE:

        # Wheel speeds persist - no reset on key release
        # (Wheel speeds are only changed by pressing keys, not by releasing them)

        # Reset angular velocity control
        if key == glfw.KEY_LEFT: state.angular_vel_control = AngularVelocityControl.NONE
        elif key == glfw.KEY_RIGHT: state.angular_vel_control = AngularVelocityControl.NONE

        # Joint controls are one-time increments, no need to reset on release

        # Reset camera back to neutral after letting go of a key
        if key == glfw.KEY_UP: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_DOWN: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_PAGE_UP: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_PAGE_DOWN: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_HOME: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_DELETE: state.cam_control = CameraControl.NONE

    return None
