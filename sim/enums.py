from enum import Enum

class BotControl(Enum):
    FWD = 0
    LEFT = 1
    RIGHT = 2
    BACK = 3
    NEUTRAL = 4

class CameraControl(Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    HOME = 4
    ZOOM_IN = 5
    ZOOM_OUT = 6
    RESET_CAMERA = 7
    NONE = 8

class AngularVelocityControl(Enum):
    LEFT = 0
    RIGHT = 1
    NONE = 2

class JointControl(Enum):
    A1_INCREASE = 0
    A1_DECREASE = 1
    A2_INCREASE = 2
    A2_DECREASE = 3
    H_INCREASE = 4
    H_DECREASE = 5
    NONE = 6

class HeadActions(Enum):
    EXPRESSION_IDLE = 0
    EXPRESSION_FAST = 1
    EXPRESSION_FAST_TURN = 2
    EXPRESSION_INQUISITIVE = 3
    EXPRESSION_HEAD_SHAKE = 4
    EXPRESSION_HEAD_SPIN = 5
    EXPRESSION_HURT = 6
    NONE = 7



