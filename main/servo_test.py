from gpiozero import Servo
from time import sleep

# main resource:
# https://www.digikey.com/en/maker/tutorials/2021/how-to-control-servo-motors-with-a-raspberry-pi

# backup:
# https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/


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
