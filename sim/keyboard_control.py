from mujoco.glfw import glfw
from control_state import state
from enums import BotControl
from enums import CameraControl
from enums import AngularVelocityControl
from enums import JointControl
from enums import HeadActions
import time


def keyboard_callback(window, key, scancode, action, mods):
    if action == glfw.PRESS or action == glfw.REPEAT:  # Handle key press or hold

        # Robot Control

        # Forwards and Backwards Control - increment wheel speeds by 0.1
        # Forward: w1 positive, w3 negative
        if key == glfw.KEY_W:
            # state.w1_speed += 0.1
            # state.w3_speed -= 0.1
            # Clamp to [-1, 1] range
            # state.w1_speed = max(-1.0, min(1.0, state.w1_speed))
            # state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            # print(f"Forward: w1={state.w1_speed:.2f}, w3={state.w3_speed:.2f}")

            state.body_x_speed += 10000
            state.body_x_speed = max(-150000.00, min(150000, state.body_x_speed))

            print(f"Forward: body_x={state.body_x_speed:.2f}")

        # Backward: w1 negative, w3 positive
        if key == glfw.KEY_S:
            # state.w1_speed -= 0.1
            # state.w3_speed += 0.1
            # Clamp to [-1, 1] range
            # state.w1_speed = max(-1.0, min(1.0, state.w1_speed))
            # state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            # print(f"Backward: w1={state.w1_speed:.2f}, w3={state.w3_speed:.2f}")

            state.body_x_speed -= 10000
            state.body_x_speed = max(-150000.00, min(150000, state.body_x_speed))
            print(f"Backward: body_x={state.body_x_speed:.2f}")

        # Left and Right Control - increment wheel speeds by 0.1
        # Left: w2 positive, w3 negative
        if key == glfw.KEY_A:
            # state.w2_speed += 0.1
            # state.w3_speed -= 0.1
            # Clamp to [-1, 1] range
            # state.w2_speed = max(-1.0, min(1.0, state.w2_speed))
            # state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            # print(f"Left: w2={state.w2_speed:.2f}, w3={state.w3_speed:.2f}")

            state.body_y_speed += 10000
            state.body_y_speed = max(-150000.00, min(150000, state.body_y_speed))
            print(f"Left: body_y={state.body_y_speed:.2f}")
        # Right: w2 negative, w3 positive
        if key == glfw.KEY_D:
            # state.w2_speed -= 0.1
            # state.w3_speed += 0.1
            # # Clamp to [-1, 1] range
            # state.w2_speed = max(-1.0, min(1.0, state.w2_speed))
            # state.w3_speed = max(-1.0, min(1.0, state.w3_speed))
            # print(f"Right: w2={state.w2_speed:.2f}, w3={state.w3_speed:.2f}")

            state.body_y_speed -= 10000
            state.body_y_speed = max(-150000.00, min(150000, state.body_y_speed))
            print(f"Right: body_y={state.body_y_speed:.2f}")

        # Neutral Key (Note: X is now used for a2 joint decrease, consider using a different key for neutral)
        # TODO: Reassign neutral key if needed, or remove this if X should only control a2
        # if key == glfw.KEY_X:
        #     state.fb_control = BotControl.NEUTRAL
        #     state.rl_control = BotControl.NEUTRAL

        # Angular Velocity Control (Left/Right arrows for base rotation)
        if key == glfw.KEY_Q:
            # state.angular_vel_control = AngularVelocityControl.LEFT
            state.body_r_speed += 10000
            state.body_r_speed = max(-150000.00, min(150000, state.body_r_speed))
            print(f"Rotate Left: body_r={state.body_r_speed:.2f}")
        if key == glfw.KEY_E:
            # state.angular_vel_control = AngularVelocityControl.RIGHT
            state.body_r_speed -= 10000
            state.body_r_speed = max(-150000.00, min(150000, state.body_r_speed))
            print(f"Rotate Right: body_r={state.body_r_speed:.2f}")

        # Joint Position Control (one-time increment per key press)
        # i, j keys: a1 joint increase and decrease (0.2 increment)
        if key == glfw.KEY_I:
            state.target_a1_pos += 0.2
            state.target_a1_pos = max(-3.14159, min(3.14159, state.target_a1_pos))
        elif key == glfw.KEY_J:
            state.target_a1_pos -= 0.2
            state.target_a1_pos = max(-3.14159, min(3.14159, state.target_a1_pos))
        # k, l keys: a2 joint increase and decrease (0.2 increment)
        elif key == glfw.KEY_O:
            state.target_a2_pos += 0.2
            state.target_a2_pos = max(-0.5236, min(0.5236, state.target_a2_pos))
        elif key == glfw.KEY_K:
            state.target_a2_pos -= 0.2
            state.target_a2_pos = max(-0.5236, min(0.5236, state.target_a2_pos))
        # o, p keys: h (head) joint increase and decrease (0.2 increment)
        elif key == glfw.KEY_P:
            state.target_h_pos += 0.2
            state.target_h_pos = max(-3.14159, min(3.14159, state.target_h_pos))
            state.last_head_manual_control = time.time()
        elif key == glfw.KEY_L:
            state.target_h_pos -= 0.2
            state.target_h_pos = max(-3.14159, min(3.14159, state.target_h_pos))
            state.last_head_manual_control = time.time()

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
        # Only accept new actions if not currently locked (animation in progress)
        if key == glfw.KEY_1 and not state.head_action_locked:
            state.head_actions = HeadActions.EXPRESSION_IDLE
            state.head_action_locked = True
        elif key == glfw.KEY_2 and not state.head_action_locked:
            state.head_actions = HeadActions.EXPRESSION_FAST
            state.head_action_locked = True
        elif key == glfw.KEY_3 and not state.head_action_locked:
            state.head_actions = HeadActions.EXPRESSION_FAST_TURN
            state.head_action_locked = True
        elif key == glfw.KEY_4 and not state.head_action_locked:
            state.head_actions = HeadActions.EXPRESSION_INQUISITIVE
            state.head_action_locked = True
        elif key == glfw.KEY_5 and not state.head_action_locked:
            state.head_actions = HeadActions.EXPRESSION_HEAD_SHAKE
            state.head_action_locked = True
        elif key == glfw.KEY_6 and not state.head_action_locked:
            state.head_actions = HeadActions.EXPRESSION_HEAD_SPIN
            state.head_action_locked = True
        elif key == glfw.KEY_7 and not state.head_action_locked:
            state.head_actions = HeadActions.EXPRESSION_HURT
            state.head_action_locked = True


    elif action == glfw.RELEASE:

        # Wheel speeds persist - no reset on key release
        # (Wheel speeds are only changed by pressing keys, not by releasing them)

        # Reset angular velocity control
        if key == glfw.KEY_Q:
            state.angular_vel_control = AngularVelocityControl.NONE
        elif key == glfw.KEY_E:
            state.angular_vel_control = AngularVelocityControl.NONE

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

        # NOTE: We do NOT reset head_actions on key release - the preset_actions
        # module manages animation duration and will auto-reset when done

    return None
