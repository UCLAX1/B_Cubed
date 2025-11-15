from enums import BotControl, CameraControl, AngularVelocityControl, JointControl

class ControlState:
    def __init__(self):
        self.fb_control = BotControl.NEUTRAL
        self.rl_control = BotControl.NEUTRAL
        self.cam_control = CameraControl.NONE
        self.angular_vel_control = AngularVelocityControl.NONE
        self.joint_control = JointControl.NONE
        # Joint positions (in radians)
        self.target_a1_pos = 0.0
        self.target_a2_pos = 1.0  # Initial a2 position (range: 0.7 to 1.5)
        self.target_h_pos = 0.0
        # Wheel motor control values (incremented by 0.1 per key press)
        self.w1_speed = 0.0
        self.w2_speed = 0.0
        self.w3_speed = 0.0
        self.w4_speed = 0.0
        
        self.body_x_speed = 0.0
        self.body_y_speed = 0.0
        self.body_r_speed = 0.0 

state = ControlState()