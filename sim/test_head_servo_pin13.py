"""
Temporary test: head servo (BCM 18) with encoder Channel A on physical pin 13 (BCM 27).
Tests if physical pin 13 is dead or now working.
"""

import time
from gpiozero import DigitalInputDevice, DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

SERVO_PIN  = 18
MOSFET_PIN = 16
ENC_A_PIN  = 27  # Physical pin 13 - testing if dead
SPIN_SPEED = 0.3
DURATION   = 2.0

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

try:
    factory = PiGPIOFactory()
    servo = Servo(SERVO_PIN, initial_value=None, pin_factory=factory)
    print(f"Head servo (BCM {SERVO_PIN}) initialized.")

    # Try to read encoder pin 13 (BCM 27)
    try:
        enc_a = DigitalInputDevice(ENC_A_PIN, pull_up=True, pin_factory=factory)
        print(f"Encoder Channel A on physical pin 13 (BCM {ENC_A_PIN}) initialized.")
        pin13_ok = True
    except Exception as e:
        print(f"Failed to initialize pin 13 (BCM {ENC_A_PIN}): {e}")
        pin13_ok = False

    if pin13_ok:
        print("\nSpinning right, monitoring pin 13 transitions...")
        enc_transitions = 0
        servo.value = SPIN_SPEED
        last_state = enc_a.value
        start = time.time()

        while time.time() - start < DURATION:
            if enc_a.value != last_state:
                enc_transitions += 1
                last_state = enc_a.value
                print(f"  Transition #{enc_transitions}")
            time.sleep(0.01)

        print(f"Right spin: {enc_transitions} transitions detected on pin 13")

        print(f"\nSpinning left, monitoring pin 13 transitions...")
        enc_transitions = 0
        servo.value = -SPIN_SPEED
        last_state = enc_a.value
        start = time.time()

        while time.time() - start < DURATION:
            if enc_a.value != last_state:
                enc_transitions += 1
                last_state = enc_a.value
                print(f"  Transition #{enc_transitions}")
            time.sleep(0.01)

        print(f"Left spin: {enc_transitions} transitions detected on pin 13")

        if enc_transitions > 5:
            print("\n✓ Physical pin 13 is WORKING - encoder transitions detected!")
        else:
            print("\n✗ Physical pin 13 is DEAD or not responding - no/few encoder transitions")

        enc_a.close()

    print("\nStopping...")
    servo.value = 0.0
    time.sleep(0.5)
    print("Test complete.")

finally:
    servo.value = None
    time.sleep(0.2)
    mosfet.off()
