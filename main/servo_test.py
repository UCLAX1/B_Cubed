import time
from ServoEx import ServoEx
# main resource:
# https://www.digikey.com/en/maker/tutorials/2021/how-to-control-servo-motors-with-a-raspberry-pi

# backup:
# https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

# INFO:
# encoder is 2048 counts per revolution

# step 1: connect servo to pi correctly
# step 2: power the pi somehow
# step 3: use tailscale to ssh to the pi
# step 4: run the code on the pi



servo = ServoEx(servo_pin=16, encoder_pin_a=26, encoder_pin_b=6, absolute_encoder_pin=5)

# device = AbsoluteEncoder(5)

# servo = Servo(16)
# encoder = RotaryEncoder(a=26, b=6, max_steps=100000000000) # params: "A" output gpio pin, "B" output gpio pin
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

# previous_encoder_steps = 0
# current_encoder_steps = 0

current_position: float = 0
previous_position: float = 0

current_absolute_position: float = 0
previous_absolute_position: float = 0


try:
    while True:
        current_time = time.time()
        dt = previous_time - current_time
        previous_time = current_time
        timer = current_time - start

        servo.update()

        current_position = servo.get_position()

        current_absolute_position = servo.get_absolute_position()

        # half a period every second
        # val = math.sin(timer / (0.5 * (2.0 * math.pi)))

        print(f"normal: {current_position:.2f}, absolute: {current_absolute_position:.2f}")

        # if current_position != previous_position:
        #     print(f"{current_position:.2f}")
        #
        #
        # # if current_absolute_position != previous_absolute_position:
        # #     print(f"{current_absolute_position:.2f}")

        previous_position = current_position
        previous_absolute_position = current_absolute_position


except KeyboardInterrupt:
    servo.save_encoder_position()
    print("Program stopped")
    exit(1)

servo.save_encoder_position()
print("Program stopped")
