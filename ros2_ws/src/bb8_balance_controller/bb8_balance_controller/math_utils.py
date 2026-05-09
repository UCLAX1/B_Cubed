"""Math helpers for the ROS balance controller."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def clip_norm(vector: Iterable[float], max_norm: float) -> np.ndarray:
    arr = np.asarray(vector, dtype=np.float32)
    norm = float(np.linalg.norm(arr))
    if norm <= max_norm or norm == 0.0:
        return arr
    return arr * (max_norm / norm)


def quat_to_roll_pitch(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch

