from gpiozero import Servo, DigitalInputDevice
from time import sleep
import signal

# --- CONFIGURATION ---
SERVO_PIN = 18       
ENCODER_PIN_A = 23   
ENCODER_PIN_B = 24   

# --- SETUP ---
# Calibration for RDS51150 (500us - 2500us)
servo = Servo(SERVO_PIN, min_pulse_width=0.0005, max_pulse_width=0.0025)

encoder_counter = 0
def on_encoder_change():
    global encoder_counter
    p_a = line_a.value
    p_b = line_b.value
    if p_a == p_b:
        encoder_counter += 1
    else:
        encoder_counter -= 1

line_a = DigitalInputDevice(ENCODER_PIN_A)
line_b = DigitalInputDevice(ENCODER_PIN_B)
line_a.when_activated = on_encoder_change
line_a.when_deactivated = on_encoder_change

# --- TEST ---
def test_cycle():
    print("--- Starting Servo & Encoder Test ---")
    sleep(1)
    try:
        for pos in [-1, -0.5, 0, 0.5, 1]:
            print(f"Moving Servo to position: {pos}")
            servo.value = pos
            sleep(1)
            print(f"Current Encoder Count: {encoder_counter}")
        servo.value = 0
    except KeyboardInterrupt:
        print("\nTest stopped.")

if __name__ == "__main__":
    test_cycle()
    signal.pause()
