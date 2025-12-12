from enums import BotControl, CameraControl

class ControlState:
    def __init__(self):
        self.fb_control = BotControl.NEUTRAL
        self.rl_control = BotControl.NEUTRAL
        self.cam_control = CameraControl.NONE

state = ControlState()