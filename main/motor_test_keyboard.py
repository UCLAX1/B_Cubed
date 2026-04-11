from HardwareInterface import CanBus, Motor
from PIDController import PIDController
import time
import pygame
from pygame.locals import *
# https://stackoverflow.com/questions/12049154/python-numpy-vector-math
import numpy as np

# tasks:
# 1. see which power maps to which motor speed (figure out units)
#   this will likely be done with a velocity PID controller reading from the encoder
#   the main theory is that the encoder units are in rotations of the output shaft
# 2. test with the pi
#    perchance add a keyboard controller
#    need to do the math for the velocity

# https://stackoverflow.com/questions/459083/how-do-you-run-your-own-code-alongside-tkinters-event-loop



def exit_gracefully():
    # top_left_motor.set_power(0)
    # top_right_motor.set_power(0)
    # bottom_motor.set_power(0)
    # bus.close()
    print("exception occurred")
    exit(1)


class InputHandler:

    def __init__(self):
        self.up_pressed: bool = False
        self.down_pressed: bool = False
        self.left_pressed: bool = False
        self.right_pressed: bool = False

        self.cw_pressed: bool = False
        self.ccw_pressed: bool = False

        self.mouse_pressed: bool = False

        self.mouse_pos: np.ndarray = np.array([0.0, 0.0])


    def get_input(self):
        # HANDLE INPUT

        for event in pygame.event.get():
            if event.type == MOUSEBUTTONDOWN:
                self.mouse_pressed = True

            if event.type == MOUSEBUTTONUP:
                self.mouse_pressed = False

            if event.type == KEYDOWN:

                if event.key == K_RIGHT: self.cw_pressed = True
                if event.key == K_LEFT: self.ccw_pressed = True


            if event.type == KEYUP:

                if event.key == K_RIGHT: self.cw_pressed = False
                if event.key == K_LEFT: self.ccw_pressed = False


        self.mouse_pos = np.array(pygame.mouse.get_pos())




class App:
    WINDOW_SIZE: np.ndarray = np.array([640, 480])
    MIDDLE_COORD: np.ndarray = WINDOW_SIZE / 2
    DRAW_SCALE: float = 100.0

    # units per second
    SPEED_CHANGE_RATE: float = 0.5

    TOP_LEFT_WHEEL_COORD: np.ndarray = MIDDLE_COORD + DRAW_SCALE * np.array([-np.sqrt(3) / 2, 0.5])
    TOP_RIGHT_WHEEL_COORD: np.ndarray = MIDDLE_COORD + DRAW_SCALE * np.array([np.sqrt(3) / 2, 0.5])
    BOTTOM_WHEEL_COORD: np.ndarray = MIDDLE_COORD + DRAW_SCALE * np.array([0, -1])

    # vectors perpendicular to the wheels
    # the axes that the wheels are lying on
    TOP_LEFT_VEC: np.ndarray = np.array([-0.5, -np.sqrt(3) / 2])
    TOP_RIGHT_VEC: np.ndarray = np.array([-0.5, np.sqrt(3) / 2])
    BOTTOM_VEC: np.ndarray = np.array([1, 0])

    MAX_DURATION: float = 9999.0

    def __init__(self):
        # timer in seconds
        self.timer: float = 0.0
        self.start: float = time.time()
        self.current_time: float = self.start
        self.previous_time: float = self.current_time
        self.dt: float = 0.0

        self.input_handler = InputHandler()
        self.mouse_vec: np.ndarray = np.array([0.0, 0.0])

        self.velocity: np.ndarray = np.array([0.0, 0.0])
        self.speed: float = 1.0
        # speed: float = 0.01
        self.angular_velocity: float = 0

        self.top_left_speed: float = 0.0
        self.top_right_speed: float = 0.0
        self.bottom_speed: float = 0.0

        self.bus: CanBus = CanBus(channel='COM5', interface='slcan', bitrate=1000000)
        self.bus.start()

        if not self.bus.started_successfully():
            print("CAN bus did not start, only running graphical")

        if self.bus.started_successfully():
            self.top_left_motor: Motor = Motor(self.bus, 3)
            self.top_left_motor.set_power(0)
            self.top_left_motor.reset_encoder()

            self.top_right_motor: Motor = Motor(self.bus, 7)
            self.top_right_motor.set_power(0)
            self.top_right_motor.reset_encoder()

            self.bottom_motor = Motor(self.bus, 9)
            self.bottom_motor.set_power(0)
            self.bottom_motor.reset_encoder()

        # https://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-a-script-from-the-termina
        # https://www.pygame.org/docs/tut/newbieguide.html
        pygame.init()
        self.screen = pygame.display.set_mode(self.WINDOW_SIZE)
        pygame.display.set_caption('Pygame Keyboard Test')
        pygame.mouse.set_visible(1)

        # 0: top left wheel
        # 1: top right wheel
        # 2: bottom wheel
        # TODO: make this work
        # self.wheel_velocities: np.ndarray = np.array([0.0, 0.0, 0.0])

    def flip_y(self, vec: np.ndarray) -> np.ndarray:
        return np.array([vec[0], 2 * self.MIDDLE_COORD[1] - vec[1]])

    def run(self):
        try:
            while self.timer < self.MAX_DURATION:
                self.current_time = time.time()
                self.dt = self.current_time - self.previous_time
                self.previous_time = self.current_time
                self.timer = self.current_time - self.start

                app.update(self.dt)
                app.draw()

                # DRAWING


        except KeyboardInterrupt:
            exit_gracefully()

    def update(self, dt):
        self.input_handler.get_input()
        # CALCULATE MOTOR SPEEDS AND VELOCITIES

        self.mouse_vec = (self.input_handler.mouse_pos - self.MIDDLE_COORD) / self.DRAW_SCALE
        self.mouse_vec = np.array([self.mouse_vec[0], -self.mouse_vec[1]])

        if self.input_handler.mouse_pressed:
            speed = np.clip(0, 1, np.linalg.norm(self.mouse_vec))
            if speed != 0.0:
                self.velocity = (self.mouse_vec / np.linalg.norm(self.mouse_vec)) * speed
            else:
                self.velocity = np.array([0.0, 0.0])


        self.angular_velocity += self.SPEED_CHANGE_RATE * dt * (self.input_handler.ccw_pressed - self.input_handler.cw_pressed)
        # angular_velocity *= RADIUS

        self.top_left_speed: float = np.dot(self.velocity, self.TOP_LEFT_VEC) + self.angular_velocity
        self.top_right_speed: float = np.dot(self.velocity, self.TOP_RIGHT_VEC) + self.angular_velocity
        self.bottom_speed: float = np.dot(self.velocity, self.BOTTOM_VEC) + self.angular_velocity

        if self.bus.started_successfully():
            # MAKE MOTORS NOT BREAK

            self.top_left_motor.send_heartbeat()
            self.top_right_motor.send_heartbeat()
            self.bottom_motor.send_heartbeat()

            # SET MOTOR SPEEDS

            self.top_left_motor.set_power(self.top_left_speed)
            self.top_right_motor.set_power(self.top_right_speed)
            self.bottom_motor.set_power(self.bottom_speed)

    def draw(self):
        self.screen.fill((0, 0, 0))

        pygame.draw.circle(self.screen, (255, 255, 255), self.MIDDLE_COORD, self.DRAW_SCALE - 2, 2)

        pygame.draw.line(self.screen, (0, 0, 255), self.flip_y(self.TOP_LEFT_WHEEL_COORD - self.DRAW_SCALE * self.TOP_LEFT_VEC), self.flip_y(self.TOP_LEFT_WHEEL_COORD + self.DRAW_SCALE * self.TOP_LEFT_VEC), 1)
        pygame.draw.line(self.screen, (0, 0, 255), self.flip_y(self.TOP_RIGHT_WHEEL_COORD - self.DRAW_SCALE * self.TOP_RIGHT_VEC), self.flip_y(self.TOP_RIGHT_WHEEL_COORD + self.DRAW_SCALE * self.TOP_RIGHT_VEC), 1)
        pygame.draw.line(self.screen, (0, 0, 255), self.flip_y(self.BOTTOM_WHEEL_COORD - self.DRAW_SCALE * self.BOTTOM_VEC), self.flip_y(self.BOTTOM_WHEEL_COORD + self.DRAW_SCALE * self.BOTTOM_VEC), 1)


        pygame.draw.line(self.screen, (255, 0, 0), self.flip_y(self.MIDDLE_COORD), self.flip_y(self.MIDDLE_COORD + self.DRAW_SCALE * self.velocity), 5)

        pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.TOP_LEFT_WHEEL_COORD), self.flip_y(self.TOP_LEFT_WHEEL_COORD + self.DRAW_SCALE * self.top_left_speed * self.TOP_LEFT_VEC), 5)
        pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.TOP_RIGHT_WHEEL_COORD), self.flip_y(self.TOP_RIGHT_WHEEL_COORD + self.DRAW_SCALE * self.top_right_speed * self.TOP_RIGHT_VEC), 5)
        pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.BOTTOM_WHEEL_COORD), self.flip_y(self.BOTTOM_WHEEL_COORD + self.DRAW_SCALE * self.bottom_speed * self.BOTTOM_VEC), 5)

        pygame.display.update()


print("SLEEPING 1 SEC...")
time.sleep(1)



app = App()

app.run()

# on testing, the motor position updates about every 0.01-0.03 seconds

# difference in motor get_pos() output after 1 second with x power:
# 0.5: -24.764
# 0.25: -12.260
# 0.125: -5.975
