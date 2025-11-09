import glfw
from control_state import state
from enums import BotControl
from enums import CameraControl
from enums import AngularVelocityControl
from enums import JointControl
from enums import HeadActions



def keyboard_callback(window, key, scancode, action, mods):
    if action == glfw.PRESS or action == glfw.REPEAT:  # Handle key press or hold

        # Robot Control

        # Forwards and Backwards Control - increment wheel speeds by 0.1
        # Forward: w1 positive, w3 negative
        if key == glfw.KEY_W: 
            state.w1_speed += 0.1
            state.w3_speed -= 0.1
            # Clamp to [-1, 1] range
            state.w1_speed = max(-1.0, min(1.0, state.w1_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
        # Backward: w1 negative, w3 positive
        elif key == glfw.KEY_S: 
            state.w1_speed -= 0.1
            state.w3_speed += 0.1
            # Clamp to [-1, 1] range
            state.w1_speed = max(-1.0, min(1.0, state.w1_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))

        #Left and Right Control - increment wheel speeds by 0.1
        # Left: w2 positive, w3 negative
        if key == glfw.KEY_A: 
            state.w2_speed += 0.1
            state.w3_speed -= 0.1
            # Clamp to [-1, 1] range
            state.w2_speed = max(-1.0, min(1.0, state.w2_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
        # Right: w2 negative, w3 positive
        elif key == glfw.KEY_D: 
            state.w2_speed -= 0.1
            state.w3_speed += 0.1
            # Clamp to [-1, 1] range
            state.w2_speed = max(-1.0, min(1.0, state.w2_speed))
            state.w3_speed = max(-1.0, min(1.0, state.w3_speed))

        # Neutral Key (Note: X is now used for a2 joint decrease, consider using a different key for neutral)
        # TODO: Reassign neutral key if needed, or remove this if X should only control a2
        # if key == glfw.KEY_X:
        #     state.fb_control = BotControl.NEUTRAL
        #     state.rl_control = BotControl.NEUTRAL

        # Angular Velocity Control (Left/Right arrows for base rotation)
        # if key == glfw.KEY_LEFT: 
        #     state.angular_vel_control = AngularVelocityControl.LEFT
        # elif key == glfw.KEY_RIGHT:
        #     state.angular_vel_control = AngularVelocityControl.RIGHT

        # Joint Position Control (one-time increment per key press)
        # i, j keys: a1 joint increase and decrease (0.2 increment)
        if key == glfw.KEY_I:
            state.target_a1_pos += 0.2
            state.target_a1_pos = max(-3.14, min(3.14, state.target_a1_pos))
        elif key == glfw.KEY_J:
            state.target_a1_pos -= 0.2
            state.target_a1_pos = max(-3.14, min(3.14, state.target_a1_pos))
        # k, l keys: a2 joint increase and decrease (0.2 increment)
        elif key == glfw.KEY_O:
            state.target_a2_pos += 0.2
            state.target_a2_pos = max(0.7, min(1.5, state.target_a2_pos))
        elif key == glfw.KEY_K:
            state.target_a2_pos -= 0.2
            state.target_a2_pos = max(0.7, min(1.5, state.target_a2_pos))
        # o, p keys: h (head) joint increase and decrease (0.2 increment)
        elif key == glfw.KEY_P:
            state.target_h_pos += 0.2
            state.target_h_pos = max(-3.14, min(3.14, state.target_h_pos))
        elif key == glfw.KEY_L:
            state.target_h_pos -= 0.2
            state.target_h_pos = max(-3.14, min(3.14, state.target_h_pos))

        # Camera control (Note: Left/Right arrows are now used for angular velocity)
        # Only Up/Down arrows and other keys control camera now
        if key == glfw.KEY_UP: state.cam_control = CameraControl.UP
        elif key == glfw.KEY_DOWN: state.cam_control = CameraControl.DOWN
        elif key == glfw.KEY_PAGE_UP: state.cam_control = CameraControl.ZOOM_IN
        elif key == glfw.KEY_PAGE_DOWN: state.cam_control = CameraControl.ZOOM_OUT
        elif key == glfw.KEY_LEFT: state.cam_control = CameraControl.LEFT
        elif key == glfw.KEY_RIGHT: state.cam_control = CameraControl.RIGHT
        elif key == glfw.KEY_HOME: state.cam_control = CameraControl.RESET_CAMERA
        elif key == glfw.KEY_DELETE: state.cam_control = CameraControl.HOME

        # Head action controls: map number keys to HeadActions
        # 1: idle, 2: fast, 3: fast turn, 4: inquisitive, 5: head shake, 6: head spin, 7: hurt
        if key == glfw.KEY_1: state.head_actions = HeadActions.EXPRESSION_IDLE
        elif key == glfw.KEY_2: state.head_actions = HeadActions.EXPRESSION_FAST
        elif key == glfw.KEY_3: state.head_actions = HeadActions.EXPRESSION_FAST_TURN
        elif key == glfw.KEY_4: state.head_actions = HeadActions.EXPRESSION_INQUISITIVE
        elif key == glfw.KEY_5: state.head_actions = HeadActions.EXPRESSION_HEAD_SHAKE
        elif key == glfw.KEY_6: state.head_actions = HeadActions.EXPRESSION_HEAD_SPIN
        elif key == glfw.KEY_7: state.head_actions = HeadActions.EXPRESSION_HURT


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
        elif key == glfw.KEY_LEFT: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_RIGHT: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_HOME: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_DELETE: state.cam_control = CameraControl.NONE

        # Reset head action to NONE on release of number keys
        if key == glfw.KEY_0: state.head_actions = HeadActions.NONE
        elif key == glfw.KEY_1: state.head_actions = HeadActions.NONE
        elif key == glfw.KEY_2: state.head_actions = HeadActions.NONE
        elif key == glfw.KEY_3: state.head_actions = HeadActions.NONE
        elif key == glfw.KEY_4: state.head_actions = HeadActions.NONE
        elif key == glfw.KEY_5: state.head_actions = HeadActions.NONE
        elif key == glfw.KEY_6: state.head_actions = HeadActions.NONE

    return None
