from gpiozero import Servo
from gpiozero import RotaryEncoder
# from gpiozero import GPIODevice
from gpiozero import DigitalInputDevice
import time
import math
import json
import numpy as np
import os

class AbsoluteEncoder:

    POSITION_HISTORY_MAX_SIZE: int = 100

    def __init__(self, pin: int):
        self.pin = pin
        # position in ROTATIONS
        self.position: float = 0

        self.__input_device = DigitalInputDevice(pin, pull_up=False)
        self.__position_history: list[float] = []


    def __read_pulse_width_ms(self, timeout_s: float = 0.1):
        start = time.time()

        while self.__input_device.value == 1:
            if time.time() - start > timeout_s:
                return None

        while self.__input_device.value == 0:
            if time.time() - start > timeout_s:
                return None

        pulse_start = time.time()

        while self.__input_device.value == 1:
            if time.time() - start > timeout_s:
                return None

        return (time.time() - pulse_start) * 1000.0


    def update(self):
        pulse_width_ms = self.__read_pulse_width_ms()
        if pulse_width_ms is None:
            return

        # Keep the existing 0.0..1.0 normalized representation used by the
        # balance scripts, but derive it from the measured pulse width.
        new_position = np.clip(pulse_width_ms, 0.0, 1.0)

        if len(self.__position_history) >= self.POSITION_HISTORY_MAX_SIZE:
            self.__position_history = self.__position_history[1:]

        self.__position_history.append(new_position)

        self.position = sum(self.__position_history) / len(self.__position_history)

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
    POSITION_CENTERING_DELAY: float = 0.25

    def __init__(self, servo_pin: int, encoder_pin_a: int, encoder_pin_b: int, absolute_encoder_pin: int, initial_value=None, pin_factory=None, save_on_deactivate=False):
        try:
            super().__init__(servo_pin, initial_value=initial_value, pin_factory=pin_factory)
            self.encoder = RotaryEncoder(a=encoder_pin_a, b=encoder_pin_b, max_steps=10000000000000)
        except Exception:
            print("ERROR: gpiozero servo could not initialize. Make sure the servos are plugged in to the right pins.")
            exit(1)

        self.absolute_encoder = AbsoluteEncoder(pin=absolute_encoder_pin)
        self.pin = servo_pin
        self.time_position_last_centered: float = 0
        self.save_on_deactivate = save_on_deactivate
        self._last_value = None

        self.__wait_for_active(encoder_pin_a, encoder_pin_b)

        # create file if it doesn't exist
        if not os.path.exists(self.INIT_POS_FILE):
            self.save_encoder_position()

        self.load_encoder_position()

    def __wait_for_active(self, encoder_pin_a: int, encoder_pin_b: int):
        print(f"servo {self.pin} ready, encoder a={encoder_pin_a} b={encoder_pin_b} abs={self.absolute_encoder.pin} ready")

    def deactivate_and_save(self):
        """Stop the servo and save current position as the new forward/center reference."""
        self.value = None
        self.save_encoder_position()

    # returns position
    def get_position(self) -> float:
        return self.encoder.steps / self.COUNTS_PER_REVOLUTION

    def get_position_radians(self) -> float:
        return (self.encoder.steps / self.COUNTS_PER_REVOLUTION) * 2.0 * math.pi

    def get_absolute_position(self) -> float:
        return self.absolute_encoder.position

    def get_absolute_position_radians(self) -> float:
        return self.absolute_encoder.position * 2.0 * math.pi

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
        """Save the current encoder steps as the persisted reference position.

        On the next startup, load_encoder_position() restores this saved encoder
        value so the current physical pose is treated as the remembered zero.
        This only stays correct if the servo has not moved while powered off.
        """
        data = {}
        data[str(self.pin)] = self.encoder.steps
        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)


    def mark_current_position_as_zero(self):
        """Persist the current physical pose as the zero reference."""
        self.save_encoder_position()


    def load_encoder_position(self):
        """Restore the saved encoder offset from the last session."""
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
                self.encoder.steps = data.get(str(self.pin), 0)

    def reset_encoder_position(self):
        self.encoder.steps = 0

        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)

        data[str(self.pin)] = 0

        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)


    def __get_data_from_servo_init_pos_file(self) -> dict:
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
        return data

    def __write_data_to_servo_init_pos_file(self, data: dict):
        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)
