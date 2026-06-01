"""
Test Raspberry Pi GPIO pins used by the REV-11-1271 encoders.

Run the bias test with encoder signal wires unplugged from the Pi:
    python3 test_encoder_gpio_pins.py bias

Run the loopback test with one jumper wire from physical pin 8 (BCM 14) to
each requested input pin when prompted:
    python3 test_encoder_gpio_pins.py loopback
"""

import sys
import time

import pigpio


TEST_OUTPUT = 14  # physical pin 8; temporarily borrowed for the pin self-test

ENCODER_INPUTS = {
    26: "lazy A/blue (physical 37)",
    6: "lazy B/yellow (physical 31)",
    5: "lazy ABS/white (physical 29)",
    4: "head A/blue (physical 7)",
    22: "head B/yellow (physical 15)",
    17: "head ABS/white (physical 11)",
}

REPLACEMENT_CANDIDATES = {
    19: "replacement candidate (physical 35)",
    21: "replacement candidate (physical 40)",
}


def read_with_pull(pi, pin, pull):
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pull)
    time.sleep(0.05)
    return pi.read(pin)


def run_bias_test(pi):
    print("Disconnect encoder signal wires from the Pi before interpreting results.")
    print("A functional disconnected GPIO input should read up=1 and down=0.\n")
    passed = True
    for pin, label in {**ENCODER_INPUTS, **REPLACEMENT_CANDIDATES}.items():
        up = read_with_pull(pi, pin, pigpio.PUD_UP)
        down = read_with_pull(pi, pin, pigpio.PUD_DOWN)
        ok = up == 1 and down == 0
        passed = passed and ok
        print(f"GPIO {pin:2d} {label:32s}: up={up} down={down} {'PASS' if ok else 'FAIL'}")
    return passed


def run_loopback_test(pi):
    print(f"Use a jumper wire from physical pin 40 (BCM {TEST_OUTPUT}).")
    print("Disconnect encoder signal wires before testing.\n")
    passed = True
    pi.set_mode(TEST_OUTPUT, pigpio.OUTPUT)
    try:
        for pin, label in {**ENCODER_INPUTS, **REPLACEMENT_CANDIDATES}.items():
            input(f"Connect physical pin 40 to GPIO {pin:2d} ({label}), then press Enter: ")
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
            edges = 0

            def count_edge(_pin, level, _tick):
                nonlocal edges
                if level in (0, 1):
                    edges += 1

            callback = pi.callback(pin, pigpio.EITHER_EDGE, count_edge)
            try:
                for _ in range(10):
                    pi.write(TEST_OUTPUT, 1)
                    time.sleep(0.02)
                    pi.write(TEST_OUTPUT, 0)
                    time.sleep(0.02)
            finally:
                callback.cancel()

            ok = edges >= 18
            passed = passed and ok
            print(f"  observed {edges} edges: {'PASS' if ok else 'FAIL'}\n")
    finally:
        pi.write(TEST_OUTPUT, 0)
    return passed


if len(sys.argv) != 2 or sys.argv[1] not in ("bias", "loopback"):
    raise SystemExit("usage: python3 test_encoder_gpio_pins.py {bias|loopback}")

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpiod is not available")

try:
    passed = run_bias_test(pi) if sys.argv[1] == "bias" else run_loopback_test(pi)
finally:
    pi.set_mode(TEST_OUTPUT, pigpio.INPUT)
    pi.stop()

raise SystemExit(0 if passed else 1)
