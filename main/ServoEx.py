from ServoBase import ServoBase

class ServoEx(ServoBase):

    def __init__(self, servo_pin: int, encoder_pin_a: int, encoder_pin_b: int):
        super().__init__(servo_pin, encoder_pin_a, encoder_pin_b)

    def update(self):
        pass


