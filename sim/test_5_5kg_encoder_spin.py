"""
Spin-test script for the 5.5kg continuous servo with encoder feedback.

Default target is the garosa 5.5kg servo on GPIO 18.
The script alternates forward/reverse spin and prints both encoder values:
- Relative encoder position (quadrature)
- Absolute encoder position

Example:
    python3 test_5_5kg_encoder_spin.py --enc-a 26 --enc-b 6 --enc-abs 5
"""

import argparse
import time
from gpiozero import DigitalOutputDevice

from ServoEx import ServoEx


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read encoder values while spinning the 5.5kg servo."
    )
    parser.add_argument("--servo-pin", type=int, default=18, help="Servo GPIO pin (default: 18)")
    parser.add_argument("--enc-a", type=int, default=26, help="Quadrature encoder A GPIO pin")
    parser.add_argument("--enc-b", type=int, default=6, help="Quadrature encoder B GPIO pin")
    parser.add_argument("--enc-abs", type=int, default=5, help="Absolute encoder GPIO pin")
    parser.add_argument("--mosfet-pin", type=int, default=16, help="MOSFET control GPIO pin")
    parser.add_argument(
        "--speed",
        type=float,
        default=0.25,
        help="Continuous-servo speed command in range [0.0, 1.0]",
    )
    parser.add_argument(
        "--segment-seconds",
        type=float,
        default=3.0,
        help="Seconds to run each direction before switching",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=10.0,
        help="Encoder print rate in Hz",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="Forward+reverse cycles to run. 0 means run forever.",
    )
    return parser.parse_args()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def print_reading(elapsed_s: float, command: float, rel_rot: float, abs_rot: float) -> None:
    rel_deg = rel_rot * 360.0
    abs_deg = abs_rot * 360.0
    print(
        f"t={elapsed_s:7.2f}s | cmd={command:+.2f} | "
        f"rel={rel_rot:8.4f} rot ({rel_deg:8.2f} deg) | "
        f"abs={abs_rot:8.4f} rot ({abs_deg:8.2f} deg)",
        flush=True,
    )


def run_test(args: argparse.Namespace) -> None:
    speed = clamp(abs(args.speed), 0.0, 1.0)
    if args.sample_hz <= 0:
        raise ValueError("--sample-hz must be > 0")
    if args.segment_seconds <= 0:
        raise ValueError("--segment-seconds must be > 0")

    print("Initializing 5.5kg servo encoder spin test...")
    print(
        "Config: "
        f"servo={args.servo_pin}, encA={args.enc_a}, encB={args.enc_b}, "
        f"encAbs={args.enc_abs}, mosfet={args.mosfet_pin}, speed={speed}"
    )

    servo = ServoEx(
        servo_pin=args.servo_pin,
        encoder_pin_a=args.enc_a,
        encoder_pin_b=args.enc_b,
        absolute_encoder_pin=args.enc_abs,
    )
    mosfet = DigitalOutputDevice(args.mosfet_pin)

    sample_period = 1.0 / args.sample_hz
    start = time.time()

    # Alternate forward and reverse to make encoder movement obvious.
    phase_commands = [
        ("forward", +speed),
        ("reverse", -speed),
    ]

    try:
        print("Turning MOSFET ON...")
        mosfet.on()
        time.sleep(0.5)

        cycle = 0
        while True:
            if args.cycles > 0 and cycle >= args.cycles:
                break

            cycle += 1
            print(f"\nCycle {cycle}{' (infinite mode)' if args.cycles == 0 else ''}")

            for phase_name, command in phase_commands:
                print(f"  Phase: {phase_name} (cmd={command:+.2f})")
                servo.value = command

                phase_end = time.time() + args.segment_seconds
                while time.time() < phase_end:
                    servo.update()
                    rel_rot = servo.get_position()
                    abs_rot = servo.get_absolute_position()
                    elapsed = time.time() - start
                    print_reading(elapsed, command, rel_rot, abs_rot)
                    time.sleep(sample_period)

            print("  Phase: stop (cmd=0.00)")
            servo.value = 0.0
            time.sleep(0.25)

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C).")
    finally:
        print("Stopping servo and saving encoder position...")
        servo.value = 0.0
        servo.save_encoder_position()
        time.sleep(0.2)

        print("Turning MOSFET OFF...")
        mosfet.off()
        print("Done.")


if __name__ == "__main__":
    run_test(parse_args())
