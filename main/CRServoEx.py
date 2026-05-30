from gpiozero import DigitalInputDevice
import time
import math
import json
import numpy as np
import os

from ServoBase import ServoBase
from PIDController import PIDController

class AbsoluteEncoder:

    POSITION_HISTORY_MAX_SIZE: int = 100

    def __init__(self, pin: int):
        self.pin = pin
        # position in ROTATIONS
        self.position: float = 0

        self.__input_device = DigitalInputDevice(pin)
        self.__input_device.when_activated = self.on_activated
        self.__input_device.when_deactivated = self.on_deactivated
        self.__time_activated: float = 0.0
        self.__previous_value: int = 0
        self.__current_value: int = 0
        self.__position_history: list[float] = []

    def is_active(self):
        return self.__input_device.is_active

    def on_activated(self):
        self.__time_activated = time.time()

    def on_deactivated(self):
        dt = time.time() - self.__time_activated

        # sanity check: if dt is too unreasonable then don't do anything
        if dt > 0.005:
            return

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

    def update(self):
        pass
        # self.__current_value = self.__input_device.value
        #
        # # rising edge
        # if self.__current_value == 1 and self.__previous_value == 0:
        #     self.__time_activated = time.time()
        #
        # # falling edge
        # if self.__current_value == 0 and self.__previous_value == 1:
        #     dt = time.time() - self.__time_activated
        #     new_position = dt * 1000
        #     # if the position is too close to the zero, it starts going way past 1.0
        #     # clamp new_position from 0.0 to 1.0
        #     new_position = np.clip(new_position, 0.0, 1.0)
        #
        #     # save position history
        #     if len(self.__position_history) >= self.POSITION_HISTORY_MAX_SIZE:
        #         self.__position_history = self.__position_history[1:]
        #         # [1, 2, 3, 4, 5]
        #         # VVV
        #         # [2, 3, 4, 5]
        #
        #     self.__position_history.append(new_position)
        #
        #     # set position to average of position history
        #     self.position = sum(self.__position_history) / len(self.__position_history)
        #
        # self.__previous_value = self.__current_value

class CRServoEx(ServoBase):

    # when the absolute encoder wraps from 0.98->0.01, since it's based on voltage, it has to glide down from 0.98.
    # this is bad because that means there is a specific angle range where the absolute encoder is wrong.
    # this constant represents the error at which the centering with the absolute position will not happen,
    # due to the absolute encoder having erroneous values in this "deadzone".
    # unit: rotations
    POSITION_CENTERING_DEAD_ZONE_ERROR: float = 0.08

    # center position every x seconds
    POSITION_CENTERING_DELAY: float = 0.25

    kP: float = 0.00
    kI: float = 0.00
    kD: float = 0.00


    # check the one google sheet for what "servo_pin", "encoder_pin_a", etc. are
    def __init__(self, servo_pin: int, encoder_pin_a: int, encoder_pin_b: int, absolute_encoder_pin: int):
        super().__init__(servo_pin, encoder_pin_a, encoder_pin_b)

        self.absolute_encoder = AbsoluteEncoder(pin=absolute_encoder_pin)
        self.wait_for_absolute_encoders_active()

        self.pid_controller: PIDController = PIDController(self.kP, self.kI, self.kD)

        self.time_position_last_centered: float = 0

        # offset between actual position and absolute encoder position in steps
        self.encoder_position_offset = 0.0


    def wait_for_absolute_encoders_active(self):

        print(f"waiting for absolute encoder {self.absolute_encoder.pin}")
        # max_wait = 3
        max_wait = 10000
        start = time.time()
        while not self.absolute_encoder.is_active():
            if time.time() - start > max_wait:
                print(f"WARNING: Timeout waiting for absolute encoder {self.absolute_encoder.pin} to activate.")
                raise Exception("absolute encoder not connected")
            time.sleep(0.05)

        print(f"absolute encoder connected {self.absolute_encoder.pin}")

    # returns position including the encoder position offset, USE THIS!
    def get_position(self) -> float:
        return (self.encoder.steps + self.encoder_position_offset) / self.COUNTS_PER_REVOLUTION

    def get_position_radians(self) -> float:
        return self.get_position() * 2.0 * math.pi

    def get_absolute_position(self) -> float:
        return self.absolute_encoder.position

    def get_absolute_position_radians(self) -> float:
        return self.get_absolute_position() * 2.0 * math.pi

    # call in main loop, dt is delta time
    def update(self, dt: float):

        control = self.pid_controller.update(dt, self.get_position())
        self.set_velocity(control)

        self.update_absolute_encoder()

        if time.time() - self.time_position_last_centered > self.POSITION_CENTERING_DELAY:
            self.center_position_with_absolute_encoder()
            self.time_position_last_centered = time.time()

    def update_absolute_encoder(self):
        self.absolute_encoder.update()

    def set_position(self, position: float):
        self.pid_controller.set_setpoint(position)

    def set_velocity(self, velocity: float):
        self.value = velocity

    def stop(self):
        self.set_velocity(0.0)

    def center_position_with_absolute_encoder(self):

        # this if statement essentially just makes it so it doesn't do any centering
        # when the absolute position isn't found yet
        if self.get_absolute_position() > (1.0 - self.POSITION_CENTERING_DEAD_ZONE_ERROR) or self.get_absolute_position() < self.POSITION_CENTERING_DEAD_ZONE_ERROR:
            return

        mod_position = self.get_position() % 1.0

        if mod_position > (1.0 - self.POSITION_CENTERING_DEAD_ZONE_ERROR) or mod_position < self.POSITION_CENTERING_DEAD_ZONE_ERROR:
            return

        # how much greater absolute position is from encoder position
        position_difference = self.get_absolute_position() - mod_position
        self.encoder_position_offset += position_difference * self.COUNTS_PER_REVOLUTION

    def save_encoder_position(self):
        data = self.get_data_from_servo_init_pos_file()

        data[str(self.pin)] = self.encoder.steps + self.encoder_position_offset

        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)


    def load_encoder_position(self):
        # data = self.get_data_from_servo_init_pos_file()
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
                self.encoder_position_offset = data.get(str(self.pin), 0)

    def reset_encoder_position(self):

        # update json file
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)

        data[str(self.pin)] = 0

        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)
