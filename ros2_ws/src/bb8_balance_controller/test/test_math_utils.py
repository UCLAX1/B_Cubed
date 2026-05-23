import math

import numpy as np

from bb8_balance_controller.math_utils import (
    clip_norm,
    quat_to_roll_pitch,
    rpy_to_quat,
    wrap_pi,
)


def test_clip_norm_limits_vector():
    result = clip_norm([3.0, 4.0], 2.5)
    assert np.isclose(np.linalg.norm(result), 2.5)


def test_quat_to_roll_pitch_identity():
    roll, pitch = quat_to_roll_pitch(0.0, 0.0, 0.0, 1.0)
    assert math.isclose(roll, 0.0)
    assert math.isclose(pitch, 0.0)


def test_rpy_to_quat_round_trips_roll_pitch():
    quat = rpy_to_quat(0.2, -0.15, 0.4)

    roll, pitch = quat_to_roll_pitch(*quat)

    assert math.isclose(roll, 0.2)
    assert math.isclose(pitch, -0.15)


def test_wrap_pi_keeps_angle_in_principal_range():
    assert math.isclose(wrap_pi(3.0 * math.pi), -math.pi)
