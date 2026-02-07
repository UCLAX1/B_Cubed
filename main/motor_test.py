from HardwareInterface import CanBus, Motor
from PIDController import PIDController
import tkinter as tk
import threading
import time

# https://stackoverflow.com/questions/459083/how-do-you-run-your-own-code-alongside-tkinters-event-loop
class App(threading.Thread):

    def __init__(self):

        threading.Thread.__init__(self)
        self.start()


    def callback(self):
        self.root.quit()

    def run(self):
        self.root = tk.Tk()
        self.scale_var = tk.DoubleVar()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)
        self.root.geometry("400x100")

        label = tk.Label(self.root, text="Hello World")
        label.pack()
        scale = tk.Scale(self.root, from_=-0.5, to=0.5, resolution=0.01, orient="horizontal", length=400, variable=self.scale_var)
        scale.pack()

        self.root.mainloop()

def exit_gracefully():
    motor.set_power(0.0)
    bus.close()
    print("exception occurred")
    exit(1)


app = App()



bus = CanBus(channel='COM5', interface='slcan', bitrate=1000000)
bus.start()
motor = Motor(bus, 1)

# motor.set_power(-0.125)
# motor.set_power(1.0)
# motor.set_power(-0.1)
motor.set_power(0.0)
motor.reset_encoder()

# MAX_DURATION = 30
MAX_DURATION = 9999


pidController = PIDController(0.04, 0.00, -0.004)
pidController.set_setpoint(10.0)

pid_power : float
previous_pid_power : float = 0.0
current_pos : float
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

try:
    while timer < MAX_DURATION:

        current_time = time.time()
        dt = previous_time - current_time
        previous_time = current_time
        timer = current_time - start

        motor.send_heartbeat()
        current_pos = motor.get_pos()
        print(app.scale_var.get())
        # pid_power = pidController.update(current_pos, dt)
        # if abs(current_pos - previous_pos) > pos_error:
        #     print(f"Motor position: {current_pos:.2f}")
        #     print()
        # if abs(pid_power - previous_pid_power) > pid_power_error:
        #     print(f"pid power: {pid_power:.2f}")
        #     print()
        # motor.set_power(pid_power)
        motor.set_power(app.scale_var.get())

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
