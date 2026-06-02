from ServoBase import ServoBase
import numpy as np

class ServoEx(ServoBase):

    # max range: max range of the servo (ex. servo can only be set from -max_range to max_range), should be a float between 0 and 1
    def __init__(self, servo_pin: int, range_degrees: float, max_value):
        super().__init__(servo_pin)
        self.max_value = max_value
        self.range_degrees = range_degrees

    def set_position(self, position: float):
        super().set_position(position * self.max_value)

    def get_max_value(self) -> float:
        return self.max_value

    def get_range_degrees(self) -> float:
        return self.range_degrees

    def get_range_radians(self) -> float:
        return np.deg2rad(self.range_degrees)

    def update(self):
        pass


