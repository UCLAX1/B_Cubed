from gpiozero import Servo
from time import sleep

# main resource:
# https://www.digikey.com/en/maker/tutorials/2021/how-to-control-servo-motors-with-a-raspberry-pi

# backup:
# https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

# step 1: connect servo to pi correctly
# step 2: power the pi somehow
# step 3: use tailscale to ssh to the pi
# step 4: run the code on the pi


servo = Servo(16)
val = -1


try:
    while True:
        servo.value = val
        print(val)
        sleep(0.1)
        val = val + 0.1
        if val > 1:
            val = -1
except KeyboardInterrupt:
    print("Program stopped")
