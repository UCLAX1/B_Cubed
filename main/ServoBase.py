from gpiozero import Servo
from gpiozero import RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
import time
import math
import json
import os
import numpy as np

class ServoBase(Servo):
    INIT_POS_FILE: str = "servo_init_pos.json"
    COUNTS_PER_REVOLUTION: int = 2048

    # check the one google sheet for what "servo_pin", "encoder_pin_a", etc. are
    def __init__(self, servo_pin: int):
        try:
            super().__init__(servo_pin, pin_factory=PiGPIOFactory())
        except Exception:
            # print("ERROR: gpiozero servo could not initialize. Make sure the servos are plugged in to the right pins.")
            raise Exception("ERROR: gpiozero servo could not initialize. Make sure the servos are plugged in to the right pins.")

        # position from -1.0 to 1.0
        self.position: float = 0.0
        self.pin = servo_pin


    def set_position(self, position: float):
        self.position = np.clip(position, -1.0, 1.0)
        self.value = self.position

    # returns position
    def get_position(self) -> float:
        return self.position

    def get_position_radians(self) -> float:
        return self.get_position() * 2.0 * math.pi
