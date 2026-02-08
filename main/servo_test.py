from gpiozero import Servo
from gpiozero import RotaryEncoder
import time
import math
from gpiozero import RotaryEncoder
import time
import math
# main resource:
# https://www.digikey.com/en/maker/tutorials/2021/how-to-control-servo-motors-with-a-raspberry-pi

# backup:
# https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

# step 1: connect servo to pi correctly
# step 2: power the pi somehow
# step 3: use tailscale to ssh to the pi
# step 4: run the code on the pi

# step 1: connect servo to pi correctly
# step 2: power the pi somehow
# step 3: use tailscale to ssh to the pi
# step 4: run the code on the pi


# servo = Servo(16)
encoder = RotaryEncoder(26, 6, max_steps=100000000000000) # params: "A" output gpio pin, "B" output gpio pin
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

previous_encoder_steps = 0
current_encoder_steps = 0

try:
    while True:
        current_time = time.time()
        dt = previous_time - current_time
        previous_time = current_time
        timer = current_time - start

        current_encoder_steps = encoder.steps

        # val = math.sin(timer)

        # servo.value = val
        # servo.value = 0
        # print(val)
        
        if (current_encoder_steps != previous_encoder_steps):
            print(encoder.steps)
        previous_encoder_steps = current_encoder_steps


except KeyboardInterrupt:
    print("Program stopped")

