"""Kinematics helpers for the three-wheel Kiwi drive platform."""

from __future__ import annotations

from dataclasses import dataclass
import math


SQRT_3 = math.sqrt(3.0)

TOP_LEFT_DIRECTION = (-0.5, -SQRT_3 / 2.0)
TOP_RIGHT_DIRECTION = (-0.5, SQRT_3 / 2.0)
BOTTOM_DIRECTION = (1.0, 0.0)


@dataclass(frozen=True)
class WheelPowers:
    """Normalized power commands for the three Kiwi drive wheels."""

    top_left: float
    top_right: float
    bottom: float

    def as_list(self) -> list[float]:
        """Return powers in the CAN motor order used by this package."""
        return [self.top_left, self.top_right, self.bottom]


def dot2(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Return the dot product of two planar vectors."""
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1])


def clip(value: float, limit: float) -> float:
    """Clamp a value to +/- limit."""
    limit = abs(float(limit))
    return max(-limit, min(float(value), limit))


def twist_to_wheel_powers(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    *,
    linear_scale: float = 1.0,
    angular_scale: float = 1.0,
    max_power: float = 1.0,
    normalize_over_limit: bool = False,
) -> WheelPowers:
    """Convert planar robot velocity into normalized wheel power commands."""
    velocity = (
        float(linear_x) * float(linear_scale),
        float(linear_y) * float(linear_scale),
    )
    angular_power = float(angular_z) * float(angular_scale)
    powers = [
        dot2(velocity, TOP_LEFT_DIRECTION) + angular_power,
        dot2(velocity, TOP_RIGHT_DIRECTION) + angular_power,
        dot2(velocity, BOTTOM_DIRECTION) + angular_power,
    ]

    max_power = abs(float(max_power))
    if max_power <= 0.0:
        return WheelPowers(0.0, 0.0, 0.0)

    if normalize_over_limit:
        largest = max(abs(power) for power in powers)
        if largest > max_power:
            scale = max_power / largest
            powers = [power * scale for power in powers]
    else:
        powers = [clip(power, max_power) for power in powers]

    return WheelPowers(
        top_left=powers[0],
        top_right=powers[1],
        bottom=powers[2],
    )
