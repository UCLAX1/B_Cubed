"""
Minimal ServoEx-only spin test for the 5.5kg servo.

This script intentionally uses only ServoEx for motion + encoder readout.
Adjust pin constants below to match your wiring.
"""

import time
from ServoEx import ServoEx


# 5.5kg head servo defaults (from existing project scripts)
# physical pins: A=13, B=15, ABS=11 -> BCM: A=27, B=22, ABS=17
SERVO_PIN = 18
ENCODER_PIN_A = 4   # physical pin 7 (pin 13 is dead)
ENCODER_PIN_B = 22  # physical pin 15
ABS_ENCODER_PIN = 17  # physical pin 11

SPIN_COMMAND = 0.4   # continuous-servo command in range [-1, 1]
SAMPLE_HZ = 10.0


def main() -> None:
    print("Initializing ServoEx...")
    servo = ServoEx(
        servo_pin=SERVO_PIN,
        encoder_pin_a=ENCODER_PIN_A,
        encoder_pin_b=ENCODER_PIN_B,
        absolute_encoder_pin=ABS_ENCODER_PIN,
    )

    sample_period = 1.0 / SAMPLE_HZ
    start_time = time.time()

    try:
        print(f"Spinning at command {SPIN_COMMAND:+.2f}. Press Ctrl+C to stop.")
        servo.value = SPIN_COMMAND

        while True:
            servo.update()

            t = time.time() - start_time
            rel_rot = servo.get_position()
            abs_rot = servo.get_absolute_position()
            
            # DEBUG: Raw encoder step count
            raw_steps = servo.encoder.steps

            print(
                f"t={t:7.2f}s | cmd={SPIN_COMMAND:+.2f} | "
                f"steps={raw_steps:8.0f} | "
                f"rel={rel_rot:8.4f} rot ({rel_rot * 360.0:8.2f} deg) | "
                f"abs={abs_rot:8.4f} rot ({abs_rot * 360.0:8.2f} deg)",
                flush=True,
            )

            time.sleep(sample_period)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        servo.value = 0.0
        servo.save_encoder_position()
        print("Stopped and saved encoder position.")


if __name__ == "__main__":
    main()
