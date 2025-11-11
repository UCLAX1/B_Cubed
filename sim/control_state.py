from enums import BotControl, CameraControl, AngularVelocityControl, JointControl, HeadActions

class ControlState:
    def __init__(self):
        self.fb_control = BotControl.NEUTRAL
        self.rl_control = BotControl.NEUTRAL
        self.cam_control = CameraControl.NONE
        self.angular_vel_control = AngularVelocityControl.NONE
        self.joint_control = JointControl.NONE
        self.head_actions = HeadActions.NONE  # Start with NONE, not IDLE
        self.head_action_locked = False  # Lock to prevent interrupting animations
        # Joint positions (in radians)
        self.target_a1_pos = 0.0
        self.target_a2_pos = 0.0  # Initial a2 position (0 = vertical)
        self.target_h_pos = 0.0
        # Wheel motor control values (incremented by 0.1 per key press)
        self.w1_speed = 0.0
        self.w2_speed = 0.0
        self.w3_speed = 0.0
        self.w4_speed = 0.0

state = ControlState()