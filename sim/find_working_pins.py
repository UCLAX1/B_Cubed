"""
Scan Raspberry Pi BCM GPIO pins and report which ones are readable, stable,
or changing while hardware moves.

This script is intentionally conservative:
- It treats pins as inputs by default.
- It uses pull-up and pull-down reads to spot stuck or floating lines.
- It can optionally watch pins for transitions while you move encoders.
- It can optionally toggle output pins if you wire them to an input loopback.

Typical uses:

    python3 find_working_pins.py
    python3 find_working_pins.py --pins 4,5,6,12,15,16,17,20,22,26
    python3 find_working_pins.py --duration 20
    python3 find_working_pins.py --output 12 --loopback 26

The script cannot prove a pin is truly "working" without external hardware.
What it can do is identify pins that are:
- accessible to gpiozero/pigpio,
- reading clean high/low values,
- changing when you move the attached hardware,
- or responding to a loopback toggle.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable

from gpiozero import DigitalInputDevice, DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory


DEFAULT_PINS = list(range(2, 28))
KNOWN_REPO_PINS = [4, 5, 6, 12, 15, 16, 17, 20, 22, 26]


@dataclass
class PinSnapshot:
    pin: int
    pull_up_value: int
    pull_down_value: int
    transition_count: int
    samples: int

    @property
    def is_stuck(self) -> bool:
        return self.transition_count == 0 and self.samples > 0


def parse_pin_list(raw: str | None) -> list[int]:
    if not raw:
        return []
    pins: list[int] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        pins.append(int(item))
    return pins


def make_factory() -> PiGPIOFactory:
    try:
        return PiGPIOFactory()
    except Exception as exc:  # pragma: no cover - runtime hardware fallback
        print(f"ERROR: Could not create PiGPIOFactory: {exc}", file=sys.stderr)
        raise


def read_input_level(pin: int, pull_up: bool, factory: PiGPIOFactory) -> int:
    device = DigitalInputDevice(pin, pull_up=pull_up, pin_factory=factory)
    try:
        return int(device.value)
    finally:
        device.close()


def sample_pin(pin: int, factory: PiGPIOFactory, duration_s: float, interval_s: float) -> PinSnapshot:
    pull_up_value = read_input_level(pin, pull_up=True, factory=factory)
    pull_down_value = read_input_level(pin, pull_up=False, factory=factory)

    device = DigitalInputDevice(pin, pull_up=False, pin_factory=factory)
    transition_count = 0
    samples = 0
    last_value = int(device.value)
    start = time.time()

    try:
        while time.time() - start < duration_s:
            current_value = int(device.value)
            samples += 1
            if current_value != last_value:
                transition_count += 1
                last_value = current_value
            time.sleep(interval_s)
    finally:
        device.close()

    return PinSnapshot(
        pin=pin,
        pull_up_value=pull_up_value,
        pull_down_value=pull_down_value,
        transition_count=transition_count,
        samples=samples,
    )


def format_status(snapshot: PinSnapshot) -> str:
    if snapshot.transition_count > 0:
        return "ACTIVE"
    if snapshot.pull_up_value != snapshot.pull_down_value:
        return "READABLE"
    return "STUCK"


def print_snapshot(snapshot: PinSnapshot) -> None:
    status = format_status(snapshot)
    print(
        f"BCM {snapshot.pin:2d}: "
        f"pull_up={snapshot.pull_up_value} "
        f"pull_down={snapshot.pull_down_value} "
        f"transitions={snapshot.transition_count:3d} "
        f"samples={snapshot.samples:3d} "
        f"status={status}"
    )


def run_scan(pins: Iterable[int], duration_s: float, interval_s: float) -> None:
    factory = make_factory()
    print("Scanning GPIO pins as inputs...\n")
    for pin in pins:
        try:
            snapshot = sample_pin(pin, factory, duration_s=duration_s, interval_s=interval_s)
            print_snapshot(snapshot)
        except Exception as exc:
            print(f"BCM {pin:2d}: ERROR {exc}")


def run_loopback(output_pin: int, input_pin: int, pulses: int, pulse_s: float) -> None:
    factory = make_factory()
    output = DigitalOutputDevice(output_pin, pin_factory=factory)
    input_device = DigitalInputDevice(input_pin, pull_up=False, pin_factory=factory)

    try:
        print(f"Loopback test: output BCM {output_pin} -> input BCM {input_pin}")
        print(f"Initial input value: {int(input_device.value)}")
        for index in range(pulses):
            output.on()
            time.sleep(pulse_s)
            high_value = int(input_device.value)
            output.off()
            time.sleep(pulse_s)
            low_value = int(input_device.value)
            print(f"pulse {index + 1:02d}: input_high={high_value} input_low={low_value}")
    finally:
        input_device.close()
        output.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Scan GPIO pins and report which ones respond.")
    parser.add_argument(
        "--pins",
        default=",".join(str(pin) for pin in KNOWN_REPO_PINS),
        help="Comma-separated BCM pins to scan (default: repo pins used by encoders/servos)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Scan all BCM pins from 2 through 27 instead of the repo pin list.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Seconds to watch each pin for transitions during the scan.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="Seconds between samples while watching each pin.",
    )
    parser.add_argument(
        "--output",
        type=int,
        help="BCM pin to toggle as an output for a loopback test.",
    )
    parser.add_argument(
        "--loopback",
        type=int,
        help="BCM pin to read during an output loopback test.",
    )
    parser.add_argument(
        "--pulses",
        type=int,
        default=5,
        help="Number of pulses to send in loopback mode.",
    )
    parser.add_argument(
        "--pulse-time",
        type=float,
        default=0.2,
        help="Seconds to hold each loopback pulse high or low.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.output is not None or args.loopback is not None:
        if args.output is None or args.loopback is None:
            parser.error("--output and --loopback must be provided together")
        run_loopback(
            output_pin=args.output,
            input_pin=args.loopback,
            pulses=args.pulses,
            pulse_s=args.pulse_time,
        )
        return 0

    pins = DEFAULT_PINS if args.all else parse_pin_list(args.pins)
    if not pins:
        print("No pins selected.", file=sys.stderr)
        return 1

    run_scan(pins, duration_s=args.duration, interval_s=args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())