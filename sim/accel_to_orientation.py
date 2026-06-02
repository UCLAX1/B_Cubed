#!/usr/bin/env python3
"""Convert accelerometer readings to pitch/roll, and yaw when magnetometer data is available.

Yaw cannot be computed from accelerometer values alone. If you provide magnetometer
values too, this script computes a tilt-compensated yaw estimate.
"""

import argparse
import math
import sys


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def normalize_vector(x, y, z):
    magnitude = math.sqrt(x * x + y * y + z * z)
    if magnitude == 0.0:
        raise ValueError("Vector magnitude is zero")
    return x / magnitude, y / magnitude, z / magnitude


def accel_to_pitch_roll(ax, ay, az):
    """Return pitch and roll in degrees from accelerometer data.

    Assumes the accelerometer is measuring mostly gravity.
    """
    ax, ay, az = normalize_vector(ax, ay, az)

    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    roll = math.degrees(math.atan2(ay, az))
    return pitch, roll


def mag_to_yaw(mx, my, mz, pitch_deg, roll_deg):
    """Compute tilt-compensated yaw in degrees from magnetometer data."""
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
    my2 = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )

    yaw = math.degrees(math.atan2(-my2, mx2))
    return (yaw + 360.0) % 360.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert accelerometer readings to pitch/roll, with optional yaw from magnetometer data."
    )
    parser.add_argument("ax", type=float, help="Accelerometer X")
    parser.add_argument("ay", type=float, help="Accelerometer Y")
    parser.add_argument("az", type=float, help="Accelerometer Z")
    parser.add_argument("mx", type=float, nargs="?", help="Magnetometer X")
    parser.add_argument("my", type=float, nargs="?", help="Magnetometer Y")
    parser.add_argument("mz", type=float, nargs="?", help="Magnetometer Z")
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        pitch_deg, roll_deg = accel_to_pitch_roll(args.ax, args.ay, args.az)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Accel: ax={args.ax:+.4f} ay={args.ay:+.4f} az={args.az:+.4f}")
    print(f"Pitch: {pitch_deg:+.2f} deg")
    print(f"Roll:  {roll_deg:+.2f} deg")

    if args.mx is None or args.my is None or args.mz is None:
        print("Yaw:   unavailable from accelerometer alone")
        return 0

    yaw_deg = mag_to_yaw(args.mx, args.my, args.mz, pitch_deg, roll_deg)
    print(f"Mag:   mx={args.mx:+.4f} my={args.my:+.4f} mz={args.mz:+.4f}")
    print(f"Yaw:   {yaw_deg:+.2f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())