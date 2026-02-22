from gpiozero import Servo
from gpiozero import RotaryEncoder
from gpiozero import GPIODevice
from gpiozero import DigitalInputDevice
import time
import math
import json
import os
import numpy as np
# main resource:
# https://www.digikey.com/en/maker/tutorials/2021/how-to-control-servo-motors-with-a-raspberry-pi

# backup:
# https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

# INFO:
# encoder is 2048 counts per revolution

# step 1: connect servo to pi correctly
# step 2: power the pi somehow
# step 3: use tailscale to ssh to the pi
# step 4: run the code on the pi

class AbsoluteEncoder:

    POSITION_HISTORY_MAX_SIZE: int = 100

    def __init__(self, pin: int):
        self.pin = pin
        # position in ROTATIONS
        self.position: float = 0

        self.__input_device = DigitalInputDevice(pin)
        self.__time_activated: float = 0
        self.__previous_value: int = 0
        self.__current_value: int = 0
        self.__position_history: list[float] = []


    def update(self):
        self.__current_value = self.__input_device.value

        # rising edge
        if self.__current_value == 1 and self.__previous_value == 0:
            self.__time_activated = time.time()

        # falling edge
        if self.__current_value == 0 and self.__previous_value == 1:
            dt = time.time() - self.__time_activated
            new_position = dt * 1000
            # if the position is too close to the zero, it starts going way past 1.0
            # clamp new_position from 0.0 to 1.0
            new_position = np.clip(new_position, 0.0, 1.0)

            # save position history
            if len(self.__position_history) >= self.POSITION_HISTORY_MAX_SIZE:
                self.__position_history = self.__position_history[1:]
                # [1, 2, 3, 4, 5]
                # VVV
                # [2, 3, 4, 5]

            self.__position_history.append(new_position)

            # set position to average of position history
            self.position = sum(self.__position_history) / len(self.__position_history)

        self.__previous_value = self.__current_value

class ServoEx(Servo):
    INIT_POS_FILE: str = "servo_init_pos.json"
    COUNTS_PER_REVOLUTION: int = 2048

    # when the absolute encoder wraps from 0.98->0.01, since it's based on voltage, it has to glide down from 0.98.
    # this is bad because that means there is a specific angle range where the absolute encoder is wrong.
    # this constant represents the error at which the centering with the absolute position will not happen,
    # due to the absolute encoder having erroneous values in this "deadzone".
    # unit: rotations
    POSITION_CENTERING_DEADZONE_ERROR: float = 0.08

    # center position every x seconds
    POSITION_CENTERING_DELAY: float = 0.50

    def __init__(self, servo_pin: int, encoder_pin_a: int, encoder_pin_b: int, absolute_encoder_pin: int):
        super().__init__(servo_pin)
        self.encoder = RotaryEncoder(a=encoder_pin_a, b=encoder_pin_b, max_steps=10000000000000)
        self.absolute_encoder = AbsoluteEncoder(pin=absolute_encoder_pin)
        self.pin = servo_pin
        self.time_position_last_centered: float = 0

        self.__wait_for_active(encoder_pin_a, encoder_pin_b)

        # create file if it doesn't exist
        if not os.path.exists(self.INIT_POS_FILE):
            self.save_encoder_position()

        self.load_encoder_position()

    def __wait_for_active(self, encoder_pin_a: int, encoder_pin_b: int):
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

        print(f"waiting for encoder a: {encoder_pin_a}, b: {encoder_pin_b}")
        # max_wait = 3
        max_wait = 10000
        start = time.time()
        while not self.is_active:
            if time.time() - start > max_wait:
                print(f"WARNING: Timeout waiting for encoder to activate.")
                raise Exception("encoder not connected")
            time.sleep(0.05)

        print(f"encoder connected")

        print(f"waiting for absolute encoder {self.absolute_encoder.pin}")
        # max_wait = 3
        max_wait = 10000
        start = time.time()
        while not self.is_active:
            if time.time() - start > max_wait:
                print(f"WARNING: Timeout waiting for absolute encoder {self.absolute_encoder.pin} to activate.")
                raise Exception("absolute encoder not connected")
            time.sleep(0.05)

        print(f"absolute encoder connected {self.absolute_encoder.pin}")

    # returns position
    def get_position(self) -> float:
        return self.encoder.steps / self.COUNTS_PER_REVOLUTION

    def get_absolute_position(self) -> float:
        return self.absolute_encoder.position

    def update(self):
        self.update_absolute_encoder()

        if time.time() - self.time_position_last_centered > self.POSITION_CENTERING_DELAY:
            self.center_position_with_absolute_encoder()
            self.time_position_last_centered = time.time()

    def update_absolute_encoder(self):
        self.absolute_encoder.update()

    def center_position_with_absolute_encoder(self):

        # this if statement essentially just makes it so it doesn't do any centering
        # when the absolute position isn't found yet
        if self.get_absolute_position() > (1.0 - self.POSITION_CENTERING_DEADZONE_ERROR) or self.get_absolute_position() < self.POSITION_CENTERING_DEADZONE_ERROR:
            return

        mod_position = self.get_position() % 1.0

        if mod_position > (1.0 - self.POSITION_CENTERING_DEADZONE_ERROR) or mod_position < self.POSITION_CENTERING_DEADZONE_ERROR:
            return

        # how much greater absolute position is from encoder position
        position_difference = self.get_absolute_position() - mod_position
        self.encoder.steps += position_difference * self.COUNTS_PER_REVOLUTION

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

    def get_position_radians(self) -> float:
        return (self.encoder.steps / self.COUNTS_PER_REVOLUTION) * 2 * math.pi

    def __get_data_from_servo_init_pos_file(self) -> dict:
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
        return data

    def __write_data_to_servo_init_pos_file(self, data: dict):
        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)









servo = ServoEx(servo_pin=16, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)

# device = AbsoluteEncoder(5)

# servo = Servo(16)
# encoder = RotaryEncoder(a=26, b=6, max_steps=100000000000) # params: "A" output gpio pin, "B" output gpio pin
# white wire goes to gpio 5 (abs encoder position)
# A: 26
# B: 6

# val = -1
val = 0


print("SLEEPING 1 SEC...")
time.sleep(1)

# timer in seconds
timer = 0
start = time.time()
current_time = start
previous_time = current_time
dt = 0

# previous_encoder_steps = 0
# current_encoder_steps = 0

current_position: float = 0
previous_position: float = 0

current_absolute_position: float = 0
previous_absolute_position: float = 0


try:
    while True:
        current_time = time.time()
        dt = previous_time - current_time
        previous_time = current_time
        timer = current_time - start

        servo.update()

        current_position = servo.get_position()

        current_absolute_position = servo.get_absolute_position()

        # half a period every second
        # val = math.sin(timer / (0.5 * (2.0 * math.pi)))

        print(f"normal: {current_position:.2f}, absolute: {current_absolute_position:.2f}")

        # if current_position != previous_position:
        #     print(f"{current_position:.2f}")
        #
        #
        # # if current_absolute_position != previous_absolute_position:
        # #     print(f"{current_absolute_position:.2f}")

        previous_position = current_position
        previous_absolute_position = current_absolute_position


except KeyboardInterrupt:
    servo.save_encoder_position()
    print("Program stopped")
    exit(1)

servo.save_encoder_position()
print("Program stopped")
