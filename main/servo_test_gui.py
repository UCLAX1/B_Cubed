from PIDController import PIDController
from gpiozero import DigitalOutputDevice
from ServoBase import ServoBase
from ServoEx import ServoEx
from CRServoEx import CRServoEx
import time
import pygame
from pygame.locals import *
# https://stackoverflow.com/questions/12049154/python-numpy-vector-math
import numpy as np
# import keyboard

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
    mosfet.off()
    exit(1)

# convert angle to -pi to pi
def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi

def flip_y(vec: np.ndarray, axis_y: float) -> np.ndarray:
    return np.array([vec[0], 2 * axis_y - vec[1]])


class InputHandler:

    def __init__(self):
        # self.up_pressed: bool = False
        # self.down_pressed: bool = False
        # self.left_pressed: bool = False
        # self.right_pressed: bool = False
        #
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
                pass

            if event.type == KEYUP:
                pass

        self.mouse_pos = np.array(pygame.mouse.get_pos())

class ServoControllerWindow:


    CIRCLE_WIDTH = 2

    def __init__(self, screen, servo: ServoBase, input_handler: InputHandler, display_middle_coord: np.ndarray, display_radius: float, servo_range: float):

        self.servo = servo

        self.input_handler = input_handler
        self.display_middle_coord = display_middle_coord
        self.display_radius = display_radius
        self.screen = screen

        self.mouse_vec: np.ndarray = np.array([0.0, 0.0])

        # self.servo: ServoEx = ServoEx(servo_pin=16, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)

        # servo_vec for drawing and stuff
        self.servo_vec: np.ndarray = np.array([0.0, 0.0])
        # self.current_position: float = servo.get_position()
        # self.current_absolute_position: float = servo.get_absolute_position()


        # requested servo angle in radians (0 is straight up)
        self.requested_servo_angle: float = 0.0

        self.DEAD_ZONE_MIDDLE_ANGLE = np.deg2rad(180)

        # range of the servo in radians
        self.SERVO_RANGE = servo_range
        # self.SERVO_RANGE = np.deg2rad(270)
        print("set servo range: ", np.rad2deg(self.SERVO_RANGE))
        self.DEAD_ZONE_RANGE = 2 * np.pi - self.SERVO_RANGE
        print("set dead-zone range: ", np.rad2deg(self.DEAD_ZONE_RANGE))
        self.DEAD_ZONE_CW_ANGLE = normalize_angle(self.DEAD_ZONE_MIDDLE_ANGLE - self.DEAD_ZONE_RANGE / 2)
        # print("set dead-zone cw angle: ", np.rad2deg(DEAD_ZONE_CW_ANGLE))
        self.DEAD_ZONE_CCW_ANGLE = normalize_angle(self.DEAD_ZONE_MIDDLE_ANGLE + self.DEAD_ZONE_RANGE / 2)
        # print("set dead-zone ccw angle: ", np.rad2deg(DEAD_ZONE_CCW_ANGLE))

    def update(self):

        if self.servo is not None:
            self.servo.update()

        self.mouse_vec = (self.input_handler.mouse_pos - self.display_middle_coord)
        self.mouse_vec = np.array([self.mouse_vec[0], -self.mouse_vec[1]])

        # self.servo_angle = np.arctan2(self.mouse_vec)

        if self.input_handler.mouse_pressed and np.linalg.norm(self.mouse_vec) < self.display_radius:

            self.servo_vec = self.mouse_vec / np.linalg.norm(self.mouse_vec)

            self.requested_servo_angle = np.arctan2(self.servo_vec[1], self.servo_vec[0])
            # change the zero
            self.requested_servo_angle -= np.pi / 2
            # wrap to -pi to pi
            self.requested_servo_angle = normalize_angle(self.requested_servo_angle)

            if self.requested_servo_angle > self.DEAD_ZONE_CW_ANGLE:
                self.requested_servo_angle = self.DEAD_ZONE_CW_ANGLE
            elif self.requested_servo_angle < self.DEAD_ZONE_CCW_ANGLE:
                 self.requested_servo_angle = self.DEAD_ZONE_CCW_ANGLE

            # print(2.0 * self.requested_servo_angle / np.pi)
            # print(2.0 * self.requested_servo_angle / (self.SERVO_RANGE))

            if self.servo is not None:
                self.servo.set_position(2.0 * self.requested_servo_angle / (self.SERVO_RANGE))

    def draw(self):
        # draw cyan dead zone lines
        pygame.draw.line(self.screen, (0, 255, 255), self.display_middle_coord, flip_y(self.display_middle_coord + np.array([-np.sin(self.DEAD_ZONE_CW_ANGLE), np.cos(self.DEAD_ZONE_CW_ANGLE)]) * self.display_radius, self.display_middle_coord[1]), 2)
        pygame.draw.line(self.screen, (0, 255, 255), self.display_middle_coord, flip_y(self.display_middle_coord + np.array([-np.sin(self.DEAD_ZONE_CCW_ANGLE), np.cos(self.DEAD_ZONE_CCW_ANGLE)]) * self.display_radius, self.display_middle_coord[1]), 2)

        # draw white circle
        pygame.draw.circle(self.screen, (255, 255, 255), self.display_middle_coord, self.display_radius - self.CIRCLE_WIDTH, self.CIRCLE_WIDTH)

        # pygame.draw.line(self.screen, (255, 0, 0), self.CIRCLE_MIDDLE_COORD, self.flip_y(self.CIRCLE_MIDDLE_COORD + self.DRAW_SCALE * self.servo_vec, self.CIRCLE_MIDDLE_COORD[1]), 5)

        # draw red line
        requested_servo_vec = np.array([-np.sin(self.requested_servo_angle), np.cos(self.requested_servo_angle)])
        pygame.draw.line(self.screen, (255, 0, 0), self.display_middle_coord, flip_y(self.display_middle_coord + requested_servo_vec * self.display_radius, self.display_middle_coord[1]), 5)






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


        # https://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-a-script-from-the-termina
        # https://www.pygame.org/docs/tut/newbieguide.html
        pygame.init()
        pygame.font.init()
        self.FONT = pygame.font.SysFont('Arial', 30)

        self.screen = pygame.display.set_mode(self.WINDOW_SIZE)
        pygame.display.set_caption('Pygame Servo Test')
        pygame.mouse.set_visible(1)
        # servo_15 = ServoEx(servo_pin=15, range_degrees=270, max_value=0.5)
        lazy_susan_servo = CRServoEx(servo_pin=12, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)

        #try:
        #    servo_15 = ServoEx(servo_pin=15, range_degrees=270, max_value=0.5)
        #    # neck_yaw_servo = ServoEx(servo_pin=20)
        #    # lazy_susan_servo = CRServoEx(servo_pin=12, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)
        #except Exception:
        #    servo_15 = None
        #    # neck_yaw_servo = None
        #    # lazy_susan_servo = None
        #    print("Servos not found, running in graphical")

        self.servo_controller_windows: list[ServoControllerWindow] = []
        # neck yaw: 5 kg
        self.servo_controller_windows.append(ServoControllerWindow(
            screen=self.screen,
            servo=lazy_susan_servo,
            input_handler=self.input_handler,
            display_middle_coord=np.array([1.0 * self.WINDOW_SIZE[0] / 3.0, self.WINDOW_SIZE[1] / 2.0]),
            display_radius=self.WINDOW_SIZE[0] / 6.0,
            servo_range=servo_15.get_range_radians() * servo_15.get_max_value()
            )
        )
        # lazy susan 70 kg
        # self.servo_controller_windows.append(ServoControllerWindow(self.screen, lazy_susan_servo, self.input_handler, np.array([2.0 * self.WINDOW_SIZE[0] / 3.0, self.WINDOW_SIZE[1] / 2.0]), self.WINDOW_SIZE[0] / 6.0))

        # self.servo_controller_windows: list[ServoControllerWindow] = []
        # self.servo_controller_windows.append(ServoControllerWindow(self.screen, None, self.input_handler, np.array([1.0 * self.WINDOW_SIZE[0] / 3.0, self.WINDOW_SIZE[1] / 2.0]), self.WINDOW_SIZE[0] / 6.0))
        # self.servo_controller_windows.append(ServoControllerWindow(self.screen, None, self.input_handler, np.array([2.0 * self.WINDOW_SIZE[0] / 3.0, self.WINDOW_SIZE[1] / 2.0]), self.WINDOW_SIZE[0] / 6.0))


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


        for servo_controller_window in self.servo_controller_windows:
            servo_controller_window.update()





    def draw(self):
        self.screen.fill((0, 0, 0))

        for servo_controller_window in self.servo_controller_windows:
            servo_controller_window.draw()

        # intersection_a_scalar = np.dot(self.TOP_RIGHT_VEC, self.TOP_RIGHT_VEC - self.TOP_LEFT_VEC) / np.cross(self.TOP_RIGHT_VEC, self.TOP_LEFT_VEC)
        # intersection_a = self.TOP_LEFT_VEC + intersection_a_scalar * np.array(-self.TOP_LEFT_VEC[1], self.TOP_LEFT_VEC[0])
        # pygame.draw.circle(self.screen, (0, 255, 255), self.DRAW_SCALE * self.flip_y(intersection_a), 4, 2)

        # vec to use to draw the servo angle
        # servo_angle_vec: np.ndarray = np.array([0.0, 0.0])

        # text_surface = self.FONT.render("requested angle: " + str(self.requested_servo_angle), False, (255, 255, 255))
        # self.screen.blit(text_surface, (0,0))
        #
        # pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.TOP_LEFT_WHEEL_COORD), self.flip_y(self.TOP_LEFT_WHEEL_COORD + self.DRAW_SCALE * self.top_left_speed * self.TOP_LEFT_VEC), 5)
        # pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.TOP_RIGHT_WHEEL_COORD), self.flip_y(self.TOP_RIGHT_WHEEL_COORD + self.DRAW_SCALE * self.top_right_speed * self.TOP_RIGHT_VEC), 5)
        # pygame.draw.line(self.screen, (0, 255, 0), self.flip_y(self.BOTTOM_WHEEL_COORD), self.flip_y(self.BOTTOM_WHEEL_COORD + self.DRAW_SCALE * self.bottom_speed * self.BOTTOM_VEC), 5)

        pygame.display.update()


# print("SLEEPING 0.5 SEC...")
# time.sleep(0.5)

mosfet = DigitalOutputDevice(16)
mosfet.on()

app = App()

app.run()

mosfet.off()

# on testing, the motor position updates about every 0.01-0.03 seconds

# difference in motor get_pos() output after 1 second with x power:
# 0.5: -24.764
# 0.25: -12.260
# 0.125: -5.975
