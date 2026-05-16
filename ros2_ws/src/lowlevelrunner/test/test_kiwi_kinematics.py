from importlib import import_module
from pathlib import Path
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

twist_to_wheel_powers = import_module(
    "lowlevelrunner.kiwi_kinematics"
).twist_to_wheel_powers


def test_forward_x_maps_to_expected_wheels():
    powers = twist_to_wheel_powers(0.25, 0.0, 0.0)

    assert math.isclose(powers.top_left, -0.125)
    assert math.isclose(powers.top_right, -0.125)
    assert math.isclose(powers.bottom, 0.25)


def test_left_y_maps_to_opposed_top_wheels():
    powers = twist_to_wheel_powers(0.0, 0.25, 0.0)

    assert math.isclose(powers.top_left, -math.sqrt(3.0) / 8.0)
    assert math.isclose(powers.top_right, math.sqrt(3.0) / 8.0)
    assert math.isclose(powers.bottom, 0.0)


def test_rotation_adds_same_power_to_all_wheels():
    powers = twist_to_wheel_powers(0.0, 0.0, 0.2)

    assert powers.as_list() == [0.2, 0.2, 0.2]


def test_power_clipping_matches_motor_power_range():
    powers = twist_to_wheel_powers(2.0, 0.0, 0.0, max_power=0.4)

    assert powers.as_list() == [-0.4, -0.4, 0.4]
