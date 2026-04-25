from PIDController import PIDController
from ServoEx import ServoEx
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
        # self.up_pressed: bool = False
        # self.down_pressed: bool = False
        # self.left_pressed: bool = False
        # self.right_pressed: bool = False
        #
        # self.cw_pressed: bool = False
        # self.ccw_pressed: bool = False

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
                pass

            if event.type == KEYUP:
                pass


        self.mouse_pos = np.array(pygame.mouse.get_pos())




class App:
    WINDOW_SIZE: np.ndarray = np.array([640, 480])
    MIDDLE_COORD: np.ndarray = WINDOW_SIZE / 2

    CIRCLE_MIDDLE_COORD: np.ndarray = np.array([MIDDLE_COORD[0], 0.60 * WINDOW_SIZE[1]])

    DRAW_SCALE: float = 170.0

    TARGET_UPDATES_PER_SECOND: float = 240.0
    TARGET_SECONDS_PER_UPDATE: float = 1.0 / TARGET_UPDATES_PER_SECOND

    def __init__(self):
        # timer in seconds
        self.timer: float = 0.0
        self.start: float = time.time()
        self.current_time: float = self.start
        self.previous_time: float = self.current_time
        self.dt: float = 0.0
        self.accumulator: float = 0.0

        self.input_handler = InputHandler()
        self.mouse_vec: np.ndarray = np.array([0.0, 0.0])

        # self.servo: ServoEx = ServoEx(servo_pin=16, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)

        # servo_vec for drawing and stuff
        self.servo_vec: np.ndarray = np.array([0.0, 0.0])
        # self.current_position: float = servo.get_position()
        # self.current_absolute_position: float = servo.get_absolute_position()


        # requested servo angle in radians (0 is straight up)
        self.requested_servo_angle: float = 0.0



        # https://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-a-script-from-the-termina
        # https://www.pygame.org/docs/tut/newbieguide.html
        pygame.init()
        pygame.font.init()
        self.FONT = pygame.font.SysFont('Arial', 30)

        self.screen = pygame.display.set_mode(self.WINDOW_SIZE)
        pygame.display.set_caption('Pygame Servo Test')
        pygame.mouse.set_visible(1)

        # 0: top left wheel
        # 1: top right wheel
        # 2: bottom wheel
        # TODO: make this work
        # self.wheel_velocities: np.ndarray = np.array([0.0, 0.0, 0.0])

    def flip_y(self, vec: np.ndarray, axis_y: float = MIDDLE_COORD[1]) -> np.ndarray:
        return np.array([vec[0], 2 * axis_y - vec[1]])

    def run(self):
        try:
            while True:
                self.current_time = time.time()
                self.dt = self.current_time - self.previous_time
                self.previous_time = self.current_time
                self.timer = self.current_time - self.start

                # delta time
                self.accumulator += self.dt
                while self.accumulator > self.TARGET_SECONDS_PER_UPDATE:
                    app.update(self.dt)
                    self.accumulator -= self.TARGET_SECONDS_PER_UPDATE

                app.draw()

                # DRAWING


        except KeyboardInterrupt:
            exit_gracefully()

    def update(self, dt):
        # self.servo.update()
        # self.current_position = self.servo.get_position()
        #
        # self.current_absolute_position = self.servo.get_absolute_position()

        self.input_handler.get_input()

        self.mouse_vec = (self.input_handler.mouse_pos - self.CIRCLE_MIDDLE_COORD) / self.DRAW_SCALE
        self.mouse_vec = np.array([self.mouse_vec[0], -self.mouse_vec[1]])

        # self.servo_angle = np.arctan2(self.mouse_vec)

        if self.input_handler.mouse_pressed:
            self.servo_vec = self.mouse_vec / np.linalg.norm(self.mouse_vec)
            self.requested_servo_angle = np.arctan2(self.servo_vec[1], self.servo_vec[0])
            # change the zero
            self.requested_servo_angle -= np.pi / 2
            # wrap to -pi to pi
            self.requested_servo_angle = (self.requested_servo_angle + np.pi) % (2 * np.pi) - np.pi



    def draw(self):
        self.screen.fill((0, 0, 0))

        # intersection_a_scalar = np.dot(self.TOP_RIGHT_VEC, self.TOP_RIGHT_VEC - self.TOP_LEFT_VEC) / np.cross(self.TOP_RIGHT_VEC, self.TOP_LEFT_VEC)
        # intersection_a = self.TOP_LEFT_VEC + intersection_a_scalar * np.array(-self.TOP_LEFT_VEC[1], self.TOP_LEFT_VEC[0])
        # pygame.draw.circle(self.screen, (0, 255, 255), self.DRAW_SCALE * self.flip_y(intersection_a), 4, 2)

        # vec to use to draw the servo angle
        servo_angle_vec: np.ndarray = np.array([0.0, 0.0])

        pygame.draw.circle(self.screen, (255, 255, 255), self.CIRCLE_MIDDLE_COORD, self.DRAW_SCALE - 2, 2)

        pygame.draw.line(self.screen, (255, 0, 0), self.CIRCLE_MIDDLE_COORD, self.flip_y(self.CIRCLE_MIDDLE_COORD + self.DRAW_SCALE * self.servo_vec, self.CIRCLE_MIDDLE_COORD[1]), 5)
        text_surface = self.FONT.render("requested angle: " + str(self.requested_servo_angle), False, (255, 255, 255))
        self.screen.blit(text_surface, (0,0))
        #
        # pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.TOP_LEFT_WHEEL_COORD), self.flip_y(self.TOP_LEFT_WHEEL_COORD + self.DRAW_SCALE * self.top_left_speed * self.TOP_LEFT_VEC), 5)
        # pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.TOP_RIGHT_WHEEL_COORD), self.flip_y(self.TOP_RIGHT_WHEEL_COORD + self.DRAW_SCALE * self.top_right_speed * self.TOP_RIGHT_VEC), 5)
        # pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.BOTTOM_WHEEL_COORD), self.flip_y(self.BOTTOM_WHEEL_COORD + self.DRAW_SCALE * self.bottom_speed * self.BOTTOM_VEC), 5)

        pygame.display.update()


# print("SLEEPING 0.5 SEC...")
# time.sleep(0.5)

app = App()

app.run()

# on testing, the motor position updates about every 0.01-0.03 seconds

# difference in motor get_pos() output after 1 second with x power:
# 0.5: -24.764
# 0.25: -12.260
# 0.125: -5.975
