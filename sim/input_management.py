"""input_management.py

Simple input processing utility for the simulator.

This module exposes a small matrix-like structure that stores three
values: [body_x_speed, body_y_speed, head_action_code]. The values are
sampled from `sim.control_state.state` via `update_input_matrix()` and
can be retrieved with `get_input_matrix()`.

The module is intentionally lightweight (no numpy) so it can be imported
from the simulation loop without extra dependencies.
"""

from typing import List
from control_state import state

# Internal storage for the latest input vector: [x_speed, y_speed, head_state]
_input_vec: List[float] = [0.0, 0.0, 0.0]


def update_input_matrix() -> None:
    """Read the current `state` and update the internal input vector.

    - x velocity:  state.body_x_speed
    - y velocity:  state.body_y_speed
    - state:       numeric code for head action (HeadActions enum value)
    """
    try:
        x = float(state.body_x_speed)
    except Exception:
        x = 0.0
    try:
        y = float(state.body_y_speed)
    except Exception:
        y = 0.0
    # head action: use the enum value if available, else map to 0
    try:
        h = float(state.head_actions.value)
    except Exception:
        # If head_actions isn't an enum, try casting directly
        try:
            h = float(state.head_actions)
        except Exception:
            h = 0.0

    _input_vec[0] = x
    _input_vec[1] = y
    _input_vec[2] = h


def get_input_matrix() -> List[float]:
    """Return a copy of the current input vector."""
    return [_input_vec[0], _input_vec[1], _input_vec[2]]


def set_body_speeds(x: float, y: float, r: float = 0.0) -> None:
    """Convenience to set body speeds on the shared `state` object.

    This updates the central `control_state.state` so other parts of the
    code (like `b_cubed_moving.py`) will pick up the values.
    """
    try:
        state.body_x_speed = float(x)
        state.body_y_speed = float(y)
        state.body_r_speed = float(r)
    except Exception:
        pass
