"""
Read REV-11-1271 encoder samples from the ESP32-S3 USB serial bridge.

Install the only dependency once:
    python3 -m pip install pyserial

Run:
    python3 read_esp32_encoders.py
    python3 read_esp32_encoders.py /dev/ttyACM1
"""

import glob
import json
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("missing dependency: run `python3 -m pip install pyserial`") from exc


BAUD_RATE = 115200
SERIAL_PATTERNS = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")
RESERVED_SERIAL_DEVICES = {"/dev/ttyACM0"}


def find_serial_device():
    if len(sys.argv) > 1:
        return sys.argv[1]

    candidates = []
    for pattern in SERIAL_PATTERNS:
        candidates.extend(glob.glob(pattern))
    candidates = sorted(
        {
            candidate
            for candidate in candidates
            if candidate not in RESERVED_SERIAL_DEVICES
        }
    )

    if not candidates:
        raise SystemExit(
            "ESP32 serial device not found; connect USB or pass /dev/ttyACM1 explicitly"
        )
    if len(candidates) > 1:
        raise SystemExit(
            f"multiple serial devices found: {candidates}; pass the ESP32 device explicitly"
        )
    return candidates[0]


def validate_sample(sample):
    required_fields = {
        "lazy_steps",
        "lazy_abs",
        "lazy_abs_valid",
        "head_steps",
        "head_abs",
        "head_abs_valid",
    }
    missing_fields = required_fields - sample.keys()
    if missing_fields:
        raise ValueError(f"sample missing fields: {sorted(missing_fields)}")


device = find_serial_device()
print(f"Reading ESP32 encoder bridge on {device} at {BAUD_RATE} baud")

with serial.Serial(device, BAUD_RATE, timeout=1) as bridge:
    time.sleep(1)
    while True:
        try:
            line = bridge.readline().decode("utf-8", errors="replace").strip()
            if not line:
                print("timeout waiting for ESP32 sample", flush=True)
                continue

            sample = json.loads(line)
            validate_sample(sample)
            print(
                f"lazy: steps={sample['lazy_steps']:8d} "
                f"abs={sample['lazy_abs']:8.4f} "
                f"{'OK' if sample['lazy_abs_valid'] else 'BAD'} | "
                f"head: steps={sample['head_steps']:8d} "
                f"abs={sample['head_abs']:8.4f} "
                f"{'OK' if sample['head_abs_valid'] else 'BAD'}",
                flush=True,
            )
        except KeyboardInterrupt:
            break
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            print(f"ignored malformed ESP32 sample: {exc}", flush=True)
