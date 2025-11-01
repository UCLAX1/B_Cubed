import glfw
from control_state import state
from sim.enums import BotControl
from sim.enums import CameraControl



def keyboard_callback(window, key, scancode, action, mods):
    pressed_keys = set()
    if action == glfw.PRESS or action == glfw.REPEAT:  # Handle key press or hold
        if action == glfw.PRESS:
            pressed_keys.add(key)
        elif action == glfw.RELEASE:
            pressed_keys.discard(key)



        # Neutral Key
        if key == glfw.KEY_X:
            state.fb_control = BotControl.NEUTRAL
            state.rl_control = BotControl.NEUTRAL

        # Diagonals
        if glfw.KEY_W in pressed_keys:
            state.fb_control = BotControl.FWD
            print("Keyboard pressed")

            if glfw.KEY_A in pressed_keys:
                state.rl_control = BotControl.LEFT
            elif glfw.KEY_D in pressed_keys:
                state.rl_control = BotControl.RIGHT

        if glfw.KEY_S in pressed_keys:
            state.fb_control = BotControl.BACK
            if glfw.KEY_A in pressed_keys:
                state.rl_control = BotControl.LEFT
            elif glfw.KEY_D in pressed_keys:
                state.rl_control = BotControl.RIGHT

        if glfw.KEY_A in pressed_keys:
            state.rl_control = BotControl.LEFT
        if glfw.KEY_D in pressed_keys:
            state.rl_control = BotControl.RIGHT






        # Camera control
        if key == glfw.KEY_UP: state.cam_control = CameraControl.UP
        elif key == glfw.KEY_DOWN: state.cam_control = CameraControl.DOWN
        elif key == glfw.KEY_LEFT: state.cam_control = CameraControl.LEFT
        elif key == glfw.KEY_RIGHT: state.cam_control = CameraControl.RIGHT

        if key == glfw.KEY_PERIOD: state.cam_control = CameraControl.HOME


    elif action == glfw.RELEASE:

        # Reset robot states back to neutral after letting go of a key
        if key == glfw.KEY_W: state.fb_control = BotControl.NEUTRAL
        elif key == glfw.KEY_S: state.fb_control = BotControl.NEUTRAL

        if key == glfw.KEY_A: state.rl_control = BotControl.NEUTRAL
        elif key == glfw.KEY_D: state.rl_control = BotControl.NEUTRAL


        # Reset camera back to neutral after letting go of a key
        if key == glfw.KEY_UP: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_DOWN: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_RIGHT: state.cam_control = CameraControl.NONE
        elif key == glfw.KEY_LEFT: state.cam_control = CameraControl.NONE

        elif key == glfw.KEY_DELETE: state.cam_control = CameraControl.NONE

    return None
