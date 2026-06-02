"""Move the arm servo through a short diagnostic sequence.

Uses the same hardware assumptions as the existing 150kg servo test:
- Servo signal on BCM 15
- MOSFET power enable on BCM 16
- gpiozero + PiGPIOFactory
"""

import time

from gpiozero import DigitalOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory


SERVO_PIN = 15
MOSFET_PIN = 16
POSITIONS = [-0.5, 0.5, 0.0]
HOLD_SECONDS = 1.0


def main() -> None:
    factory = PiGPIOFactory()
    mosfet = DigitalOutputDevice(MOSFET_PIN)
    servo = Servo(SERVO_PIN, initial_value=0.0, pin_factory=factory)

    mosfet.on()
    time.sleep(0.5)

    try:
        for position in POSITIONS:
            servo.value = position
            print(f"servo -> {position:+.1f}")
            time.sleep(HOLD_SECONDS)
    finally:
        servo.value = 0.0
        time.sleep(0.5)
        mosfet.off()
        print("servo -> +0.0 (safe shutdown)")


if __name__ == "__main__":
    main()