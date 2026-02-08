from gpiozero import Servo, DigitalInputDevice
from time import sleep
import signal

# configuration
SERVO_PIN = 18       # PWM capable pin
ENCODER_PIN_A = 23   # REV Encoder Channel A
ENCODER_PIN_B = 24   # REV Encoder Channel B

# servo setup
# min_pulse_width=0.0005 (500us), max_pulse_width=0.0025 (2500us)
servo = Servo(
    SERVO_PIN, 
    min_pulse_width=0.0005, 
    max_pulse_width=0.0025
)

# encoder setup
encoder_counter = 0

def on_encoder_change():
    global encoder_counter
    p_a = line_a.value
    p_b = line_b.value
    
    # Determine direction based on state
    if p_a == p_b:
        encoder_counter += 1
    else:
        encoder_counter -= 1

line_a = DigitalInputDevice(ENCODER_PIN_A)
line_b = DigitalInputDevice(ENCODER_PIN_B)

# interrupt like events to the A channel edges
line_a.when_activated = on_encoder_change
line_a.when_deactivated = on_encoder_change

# test routine
def test_cycle():
    print("--- Starting Servo & Encoder Test ---")
    print("12V power is ON for servo.")
    sleep(1)

    try:
        positions = [-1, -0.5, 0, 0.5, 1] # -1 is min angle, 1 is max angle
        
        for pos in positions:
            print(f"Moving Servo to position: {pos}")
            servo.value = pos
            sleep(1) # Wait for servo to reach position
            print(f"Current Encoder Count: {encoder_counter}")
            
        print("--- Return to Center ---")
        servo.value = 0
        sleep(1)
        print(f"Final Encoder Count: {encoder_counter}")

    except KeyboardInterrupt:
        print("\nTest stopped by user.")

if __name__ == "__main__":
    test_cycle()
    # Keep script running to monitor encoder if needed manually
    signal.pause()
