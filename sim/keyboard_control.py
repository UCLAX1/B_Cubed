import glfw
from control_state import state
from enums import BotControl
from enums import CameraControl



def keyboard_callback(window, key, scancode, action, mods):
    if action == glfw.PRESS or action == glfw.REPEAT:  # Handle key press or hold

        print("Keyboard pressed")

        # Robot Control

        # Forwards and Backwards Control
        if key == glfw.KEY_W: state.fb_control = BotControl.FWD
        elif key == glfw.KEY_S: state.fb_control = BotControl.BACK

        #Left and Right Control
        if key == glfw.KEY_A: state.rl_control = BotControl.LEFT
        elif key == glfw.KEY_D: state.rl_control = BotControl.RIGHT

        # Neutral Key
        if key == glfw.KEY_X:
            state.fb_control = BotControl.NEUTRAL
            state.rl_control = BotControl.NEUTRAL


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
