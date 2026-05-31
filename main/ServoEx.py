from ServoBase import ServoBase

class ServoEx(ServoBase):

    def __init__(self, servo_pin: int):
        super().__init__(servo_pin)

    def update(self):
        pass


