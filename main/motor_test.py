# try:
from HardwareInterface import CanBus, Motor
# except ImportError:
#     from controls.Joint_Controller.HardwareInterface import CanBus, Motor

import time

bus = CanBus(channel='COM5', interface='slcan', bitrate=1000000)
bus.start()
motor = Motor(bus, 1)

motor.set_power(-0.5)

for i in range(20):
    motor.send_heartbeat()
    print(f"Motor position: {motor.get_pos()}")
    time.sleep(0.1)

bus.close()