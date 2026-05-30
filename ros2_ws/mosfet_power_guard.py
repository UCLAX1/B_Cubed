#!/usr/bin/env python3
"""Hold a GPIO-controlled MOSFET on until this process is stopped."""

from __future__ import annotations

import argparse
import signal
import sys
import time

from gpiozero import DigitalOutputDevice


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pin", type=int, required=True)
    parser.add_argument("--active-low", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mosfet = DigitalOutputDevice(
        args.pin,
        active_high=not args.active_low,
        initial_value=False,
    )
    shutting_down = False

    def shutdown(signum, _frame) -> None:
        nonlocal shutting_down
        if shutting_down:
            return
        shutting_down = True
        print(f"Turning MOSFET OFF on GPIO {args.pin}", flush=True)
        mosfet.off()
        mosfet.close()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"Turning MOSFET ON on GPIO {args.pin}", flush=True)
    mosfet.on()

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        shutdown(signal.SIGINT, None)

    return 0


if __name__ == "__main__":
    sys.exit(main())
