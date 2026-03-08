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
    # motor.set_power(0.0)
    # bus.close()
    print("exception occurred")
    exit(1)

WINDOW_SIZE = np.array([640, 480])
MIDDLE_COORD = WINDOW_SIZE / 2

def flip_y(vec: np.ndarray) -> np.ndarray:
    return np.array([vec[0], 2 * MIDDLE_COORD[1] - vec[1]])



# https://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-a-script-from-the-termina
# https://www.pygame.org/docs/tut/newbieguide.html
pygame.init()
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('Pygame Keyboard Test')
pygame.mouse.set_visible(1)

bus = CanBus(channel='COM5', interface='slcan', bitrate=1000000)
bus.start()

top_left_motor = Motor(bus, 3)
top_left_motor.set_power(0)
top_left_motor.reset_encoder()

top_right_motor = Motor(bus, 7)
top_right_motor.set_power(0)
top_right_motor.reset_encoder()
#
bottom_motor = Motor(bus, 9)
bottom_motor.set_power(0)
bottom_motor.reset_encoder()


DRAW_SCALE = 100.0

top_left_coord = MIDDLE_COORD + DRAW_SCALE * np.array([-np.sqrt(3) / 2, 0.5])
top_right_coord = MIDDLE_COORD + DRAW_SCALE * np.array([np.sqrt(3) / 2, 0.5])
bottom_coord = MIDDLE_COORD + DRAW_SCALE * np.array([0, -1])

top_left_vec = np.array([-0.5, -np.sqrt(3)/2])
top_right_vec = np.array([-0.5, np.sqrt(3)/2])
bottom_vec = np.array([1, 0])

# MAX_DURATION = 30
MAX_DURATION = 9999


pidController = PIDController(0.04, 0.00, -0.004)
pidController.set_setpoint(10.0)

pid_power : float
previous_pid_power : float = 0.0
current_pos : float = 0.0
previous_pos : float = 0.0

pos_error = 0.01
pid_power_error = 0.01



print("SLEEPING 1 SEC...")
time.sleep(1)

# timer in seconds
timer = 0
start = time.time()
current_time = start
previous_time = current_time
dt = 0

velocity = np.array([0.0, 0.0])
speed: float = 1.0
# speed: float = 0.01
angular_velocity : float = 0

# units per second
SPEED_CHANGE_RATE : float = 0.5

# 0: top left wheel
# 1: top right wheel
# 2: bottom wheel
# TODO: make this work
wheel_velocities = np.array([0.0, 0.0, 0.0])

mouse_pos : np.ndarray = np.array([0.0, 0.0])
mouse_vec : np.ndarray = np.array([0.0, 0.0])

up_pressed = False
down_pressed = False
left_pressed = False
right_pressed = False

cw_pressed = False
ccw_pressed = False

speed_increase_pressed = False
speed_decrease_pressed = False

mouse_pressed = False

try:
    while timer < MAX_DURATION:

        current_time = time.time()
        dt = current_time - previous_time
        previous_time = current_time
        timer = current_time - start

        # motor.send_heartbeat()
        # current_pos = motor.get_pos()
        for event in pygame.event.get():
            if event.type == MOUSEBUTTONDOWN:
                mouse_pressed = True

            if event.type == MOUSEBUTTONUP:
                mouse_pressed = False

            if event.type == KEYDOWN:
                if event.key == K_w: up_pressed = True
                if event.key == K_s: down_pressed = True
                if event.key == K_a: left_pressed = True
                if event.key == K_d: right_pressed = True

                if event.key == K_RIGHT: cw_pressed = True
                if event.key == K_LEFT: ccw_pressed = True

                if event.key == K_UP: speed_increase_pressed = True
                if event.key == K_DOWN: speed_decrease_pressed = True

            if event.type == KEYUP:
                if event.key == K_w: up_pressed = False
                if event.key == K_s: down_pressed = False
                if event.key == K_a: left_pressed = False
                if event.key == K_d: right_pressed = False

                if event.key == K_RIGHT: cw_pressed = False
                if event.key == K_LEFT: ccw_pressed = False

                if event.key == K_UP: speed_increase_pressed = False
                if event.key == K_DOWN: speed_decrease_pressed = False

        mouse_pos = np.array(pygame.mouse.get_pos())
        mouse_vec = (mouse_pos - MIDDLE_COORD) / DRAW_SCALE
        mouse_vec = np.array([mouse_vec[0], -mouse_vec[1]])

        if mouse_pressed:
            speed = np.clip(0, 1, np.linalg.norm(mouse_vec))
            if speed != 0.0:
                velocity = (mouse_vec / np.linalg.norm(mouse_vec)) * speed
            else:
                velocity = np.array([0.0, 0.0])

        angular_velocity += SPEED_CHANGE_RATE * dt * (ccw_pressed - cw_pressed)
        # speed += SPEED_CHANGE_RATE * dt * (speed_increase_pressed - speed_decrease_pressed)
        # speed = np.clip(0, 1, speed)

        # velocity[0] = right_pressed - left_pressed
        # velocity[1] = up_pressed - down_pressed
        # # set magnitude to speed
        # norm = np.linalg.norm(velocity)
        # if norm != 0.0:
        #     velocity *= speed / norm
        # # print("velocity: ", velocity)

        top_left_speed: float = np.dot(velocity, top_left_vec) + angular_velocity
        top_right_speed: float = np.dot(velocity, top_right_vec) + angular_velocity
        bottom_speed: float = np.dot(velocity, bottom_vec) + angular_velocity

        top_left_motor.send_heartbeat()
        top_right_motor.send_heartbeat()
        bottom_motor.send_heartbeat()

        top_left_motor.set_power(top_left_speed)
        top_right_motor.set_power(top_right_speed)
        bottom_motor.set_power(bottom_speed)


        previous_pos = current_pos
        # previous_pid_power = pid_power

        screen.fill((0, 0, 0))

        pygame.draw.circle(screen, (255, 255, 255), MIDDLE_COORD, DRAW_SCALE - 2, 2)

        pygame.draw.line(screen, (0, 0, 255), flip_y(top_left_coord - DRAW_SCALE * top_left_vec), flip_y(top_left_coord + DRAW_SCALE * top_left_vec), 1)
        pygame.draw.line(screen, (0, 0, 255), flip_y(top_right_coord - DRAW_SCALE * top_right_vec), flip_y(top_right_coord + DRAW_SCALE * top_right_vec), 1)
        pygame.draw.line(screen, (0, 0, 255), flip_y(bottom_coord - DRAW_SCALE * bottom_vec), flip_y(bottom_coord + DRAW_SCALE * bottom_vec), 1)


        pygame.draw.line(screen, (255, 0, 0), flip_y(MIDDLE_COORD), flip_y(MIDDLE_COORD + DRAW_SCALE * velocity), 5)

        pygame.draw.line(screen, (0, 255, 0), flip_y(top_left_coord), flip_y(top_left_coord + DRAW_SCALE * top_left_speed * top_left_vec), 5)
        pygame.draw.line(screen, (0, 255, 0), flip_y(top_right_coord), flip_y(top_right_coord + DRAW_SCALE * top_right_speed * top_right_vec), 5)
        pygame.draw.line(screen, (0, 255, 0), flip_y(bottom_coord), flip_y(bottom_coord + DRAW_SCALE * bottom_speed * bottom_vec), 5)

        pygame.display.update()

except KeyboardInterrupt:
    exit_gracefully()



# motor.set_power(0.0)
# motor.reset_encoder()

exit_gracefully()
# bus.close()

# on testing, the motor position updates about every 0.01-0.03 seconds

# difference in motor get_pos() output after 1 second with x power:
# 0.5: -24.764
# 0.25: -12.260
# 0.125: -5.975
