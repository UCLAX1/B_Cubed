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


# https://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-a-script-from-the-termina
# https://www.pygame.org/docs/tut/newbieguide.html
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Pygame Keyboard Test')
pygame.mouse.set_visible(0)


# bus = CanBus(channel='COM5', interface='slcan', bitrate=1000000)
# bus.start()
# motor = Motor(bus, 1)

# motor.set_power(0.0)
# motor.reset_encoder()

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

# 0: top left wheel
# 1: top right wheel
# 2: bottom wheel
# TODO: make this work
wheel_velocities = np.array([0.0, 0.0, 0.0])


up_pressed = False
down_pressed = False
left_pressed = False
right_pressed = False

try:
    while timer < MAX_DURATION:

        current_time = time.time()
        dt = previous_time - current_time
        previous_time = current_time
        timer = current_time - start

        # motor.send_heartbeat()
        # current_pos = motor.get_pos()
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_w: up_pressed = True
                if event.key == K_s: down_pressed = True
                if event.key == K_a: left_pressed = True
                if event.key == K_d: right_pressed = True
            if event.type == KEYUP:
                if event.key == K_w: up_pressed = False
                if event.key == K_s: down_pressed = False
                if event.key == K_a: left_pressed = False
                if event.key == K_d: right_pressed = False

        velocity[0] = right_pressed - left_pressed
        velocity[1] = up_pressed - down_pressed
        # set magnitude to speed
        norm = np.linalg.norm(velocity)
        if norm != 0.0:
            velocity *= speed / norm
        print("velocity: ", velocity)


        previous_pos = current_pos
        # previous_pid_power = pid_power
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
