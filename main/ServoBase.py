from gpiozero import Servo
from gpiozero import RotaryEncoder
import time
import math
import json
import os

class ServoBase(Servo):
    INIT_POS_FILE: str = "servo_init_pos.json"
    COUNTS_PER_REVOLUTION: int = 2048

    # check the one google sheet for what "servo_pin", "encoder_pin_a", etc. are
    def __init__(self, servo_pin: int, encoder_pin_a: int, encoder_pin_b: int):
        try:
            super().__init__(servo_pin)
            self.encoder = RotaryEncoder(a=encoder_pin_a, b=encoder_pin_b, max_steps=10000000000000)
        except Exception:
            print("ERROR: gpiozero servo could not initialize. Make sure the servos are plugged in to the right pins.")
            exit(1)

        self.pin = servo_pin

        self.wait_for_encoders_a_b_active(encoder_pin_a, encoder_pin_b)

        # create file if it doesn't exist
        if not os.path.exists(self.INIT_POS_FILE):
            self.save_encoder_position()

        self.load_encoder_position()

    def wait_for_encoders_a_b_active(self, encoder_pin_a: int, encoder_pin_b: int):
        print(f"waiting for servo {self.pin}")
        # max_wait = 3
        max_wait = 10000
        start = time.time()
        while not self.is_active:
            if time.time() - start > max_wait:
                print(f"WARNING: Timeout waiting for servo {self.pin} to activate.")
                raise Exception("servo not connected")
            time.sleep(0.05)

        print(f"servo connected {self.pin}")

        # print(f"waiting for encoder a: {encoder_pin_a}, b: {encoder_pin_b}")
        # # max_wait = 3
        # max_wait = 10000
        # start = time.time()
        # while not self.is_active:
        #     if time.time() - start > max_wait:
        #         print(f"WARNING: Timeout waiting for encoder to activate.")
        #         raise Exception("encoder not connected")
        #     time.sleep(0.05)
        #
        # print(f"encoder connected")

    def set_position(self, position: float):
        self.value = position

    # returns position
    def get_position(self) -> float:
        return self.encoder.steps / self.COUNTS_PER_REVOLUTION

    def get_position_radians(self) -> float:
        return self.get_position() * 2.0 * math.pi

    def save_encoder_position(self):
        data = {}

        data[str(self.pin)] = self.encoder.steps

        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)


    def load_encoder_position(self):
        # data = self.get_data_from_servo_init_pos_file()
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
                self.encoder.steps = data.get(str(self.pin), 0)

    def reset_encoder_position(self):

        # update json file
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)

        data[str(self.pin)] = 0

        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)


    def get_data_from_servo_init_pos_file(self) -> dict:
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
        return data

    def write_data_to_servo_init_pos_file(self, data: dict):
        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)
