"""
Low-level GPIO input test - monitor raw pin state while servo spins.
"""

import time
from gpiozero import DigitalInputDevice, DigitalOutputDevice, Servo

SERVO_PIN = 18
TEST_PINS = [27, 22]  # A and B encoder pins
MOSFET_PIN = 16

def main():
    print("Creating digital input devices for encoder pins...")
    inputs = {pin: DigitalInputDevice(pin, pull_up=True) for pin in TEST_PINS}

    print("Turning MOSFET on...")
    mosfet = DigitalOutputDevice(MOSFET_PIN)
    mosfet.on()
    time.sleep(0.5)

    print("Initializing servo...")
    servo = Servo(SERVO_PIN)
    
    print("\nSpinning servo and monitoring GPIO states...\n")
    servo.value = 0.4
    
    start = time.time()
    pin_transitions = {pin: [] for pin in TEST_PINS}
    last_state = {pin: None for pin in TEST_PINS}
    
    while time.time() - start < 5.0:
        current_time = time.time() - start
        
        # Check for state changes
        for pin in TEST_PINS:
            current_state = inputs[pin].value
            if last_state[pin] is not None and current_state != last_state[pin]:
                transition = "0→1" if current_state else "1→0"
                pin_transitions[pin].append((current_time, transition))
                print(f"  GPIO {pin}: {transition} at t={current_time:.3f}s")
            last_state[pin] = current_state
        
        # Print current state every 0.5 seconds
        if int(current_time * 2) != int((current_time - 0.05) * 2):
            state_str = " | ".join(f"GPIO{pin}={inputs[pin].value}" for pin in TEST_PINS)
            print(f"t={current_time:.1f}s | {state_str}")
        
        time.sleep(0.01)
    
    servo.value = 0.0
    servo.close()
    mosfet.off()
    
    print("\n" + "="*60)
    print("RESULTS:")
    for pin in TEST_PINS:
        count = len(pin_transitions[pin])
        print(f"GPIO {pin}: {count} transitions detected")
        if pin_transitions[pin]:
            for t, trans in pin_transitions[pin][:5]:  # Show first 5
                print(f"    t={t:.3f}s {trans}")
            if count > 5:
                print(f"    ... and {count-5} more")

if __name__ == "__main__":
    main()
