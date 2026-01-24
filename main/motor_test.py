# try:
from HardwareInterface import CanBus, Motor

import time

bus = CanBus(channel='COM5', interface='slcan', bitrate=1000000)
bus.start()
motor = Motor(bus, 1)

motor.set_power(-0.125)
# motor.set_power(-0.1)
# motor.set_power(1.0)
# motor.set_power(0.0)

# timer in seconds
timer = 0
MAX_DURATION = 1
start = time.time()

while timer < MAX_DURATION:
    motor.send_heartbeat()
    print(f"Motor position: {motor.get_pos()}")
    timer = time.time() - start

# motor.reset_encoder()

# difference in motor get_pos() output after 1 second with x power:
# 0.5: -24.764
# 0.25: -12.260
# 0.125: -5.975

bus.close()