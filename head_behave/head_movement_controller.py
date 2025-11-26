"""head_movement_controller.py

High-level behavior controller utilities.

This module implements a small behavior evaluation step, `behaveControl`,
which maps a single input sample [x_vel, y_vel, state] -> updates a
3-element output array [head_rotate_deg, head_bend_deg, base_rotate_deg].

Key utilities:
- behaveControl(input_sample, on_off, output_arr, state_mem, dt)
- stream_behaviour(inputs, movement_ctrl=None, dt=0.02, on_off=True)

Notes:
- This code is a high-level behavior prototype, not a motor controller.
- Primitives in `predefined_motions.py` will either mutate the provided
    `output_arr` in-place (live mode) or schedule moves on a provided
    MovementController instance (controller-backed mode).
"""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, Iterable, Optional, Sequence

import numpy as np

from head_behave.head_predefined_motions import (
    idle_sway,
    set_base_to_velocity_and_bend,
    snappy_shake_with_bend,
    simple_face_direction,
    inquisitive_pose,
    head_shake,
    head_spin,
    hurt_sequence,
    stop_all_motion,
)

# Default persistent state memory
_default_state_mem: dict = {}


def _compute_direction_and_speed(x_vel: float, y_vel: float) -> tuple[float, float]:
    """Return (direction_deg, speed) from components.

    direction_deg uses standard atan2(y, x) with 0 degrees = +x axis.
    """
    direction_rad = math.atan2(y_vel, x_vel)
    direction_deg = math.degrees(direction_rad)
    speed = math.hypot(x_vel, y_vel)
    return direction_deg, speed


def compute_behavior_flags(
    x_vel: float,
    y_vel: float,
    state: int,
    prev_dir: float,
    observing: bool
) -> dict:
    """Compute boolean action flags from current motion sample.

    This separates decision logic (which actions should run) from the
    implementation of those actions. It returns a dict of booleans that
    the main `behaveControl` uses to dispatch to primitives.
    """
    direction_deg, movement_speed = _compute_direction_and_speed(x_vel, y_vel)

    # Thresholds
    
    _SPEED = 5.0
    RUN_SPEED = 15.0
    TURN_THRESHOLD = 0.5

    # Compute direction change magnitude
    dir_change = abs((direction_deg - prev_dir + 180.0) % 360.0 - 180.0)

    flags = {
        "idle_sway": False,
        "set_base_and_bend": False,
        "snappy_shake": False,
        "face_direction": False,
        "inquisitive": False,
        "head_shake": False,
        "head_spin": False,
        "hurt": False,
        "stop_all": False,
        # convenience values for callers
        "direction_deg": direction_deg,
        "movement_speed": movement_speed,
        "dir_change": dir_change,
    }

    if state == 0:
        if not observing:
            if movement_speed < _SPEED:
                flags["idle_sway"] = True
            elif movement_speed > RUN_SPEED:
                if dir_change > TURN_THRESHOLD:
                    flags["set_base_and_bend"] = True
                else:
                    flags["snappy_shake"] = True
                    flags["face_direction"] = True
            else:
                flags["face_direction"] = True
    else:
        # special states
        if state == 1:
            pass
        elif state == 2:
            flags["stop_all"] = True
        elif state == 3:
            flags["inquisitive"] = True
        elif state == 4:
            flags["head_shake"] = True
        elif state == 5:
            flags["head_spin"] = True
        elif state == 6:
            flags["hurt"] = True

    return flags


def behaveControl(
    input_sample: Sequence[float],
    on_off: bool,
    output_arr: np.ndarray,
    state_mem: Optional[Dict] = None,
    dt: float = 0.02,
    movement_ctrl: Optional[object] = None,
) -> None:
    """Evaluate one control step of the high-level behavior controller.

    Args:
        input_sample: sequence-like [x_vel, y_vel, state]
        on_off: boolean enabling the behavior loop
        output_arr: numpy array shape (3,) mutated in-place -> [head_rotate, head_bend, base_rotate]
        state_mem: optional dict persisted between calls (used for previous direction, timers, etc.)
        dt: timestep (seconds) for time-based effects
        movement_ctrl: optional MovementController for scheduling moves
        
    Notes:
        - The function is non-blocking: call it repeatedly (e.g., in your main loop) with new inputs.
        - Most specialized actions are delegated to functions in `predefined_motions.py`.
    """
    if state_mem is None:
        state_mem = _default_state_mem

    x_vel, y_vel, state = float(input_sample[0]), float(input_sample[1]), int(input_sample[2])

    # Initialize state memory defaults
    prev_dir = state_mem.get("prev_dir", 0.0)
    observing = state_mem.get("observing", False)
    state_mem.setdefault("spin_angle", 0.0)

    # Ensure output array shape
    if output_arr.shape != (3,):
        raise ValueError("output_arr must be a numpy array with shape (3,)")

    if not on_off:
        state_mem["prev_dir"] = prev_dir
        return

    # Compute flags
    flags = compute_behavior_flags(x_vel, y_vel, state, prev_dir, observing)

    # Ensure per-primitive timers dict exists
    timers = state_mem.setdefault("timers", {})

    # Priority: emergency stop
    if flags.get("stop_all"):
        stop_all_motion(output_arr, controller=movement_ctrl)
        state_mem["prev_dir"] = flags.get("direction_deg", prev_dir)
        return

    # Special states/actions
    if flags.get("inquisitive"):
        inquisitive_pose(output_arr, controller=movement_ctrl)

    if flags.get("head_shake"):
        # per-primitive timer
        t_local = timers.get("head_shake", 0.0)
        next_t = head_shake(output_arr, dt=dt, controller=movement_ctrl, start_time=None, t=t_local)
        timers["head_shake"] = next_t

    if flags.get("head_spin"):
        t_local = timers.get("head_spin", 0.0)
        val, next_t = head_spin(
            output_arr, dt=dt, controller=movement_ctrl,
            start_time=None, t=t_local
        )
        timers["head_spin"] = next_t
        output_arr[0] = val

    if flags.get("hurt"):
        hurt_sequence(output_arr, dt=dt, state_mem=state_mem, controller=movement_ctrl)

    # Motion-driven behaviors (state 0): base orientation, bend, and head rotate
    RUN_SPEED = 15.0  # Constant for calculations
    
    if flags.get("set_base_and_bend"):
        t_local = timers.get("set_base_and_bend", 0.0)
        base_val, next_t = set_base_to_velocity_and_bend(
            flags["direction_deg"], flags["movement_speed"], flags["dir_change"],
            controller=movement_ctrl, start_time=None, duration=0.2, t=t_local
        )
        timers["set_base_and_bend"] = next_t
        output_arr[2] = base_val
        output_arr[1] = float(np.clip(
            10.0 + (flags["movement_speed"] / (RUN_SPEED * 2.0)) * 20.0 +
            (flags["dir_change"] / 180.0) * 10.0, 10.0, 30.0
        ))

    if flags.get("snappy_shake"):
        t_local = timers.get("snappy_shake", 0.0)
        val, next_t = snappy_shake_with_bend(
            flags["direction_deg"], output_arr[0], flags["movement_speed"],
            t=t_local, dt=dt, controller=movement_ctrl
        )
        timers["snappy_shake"] = next_t
        output_arr[0] = val
        output_arr[1] = float(np.clip(
            10.0 + (flags["movement_speed"] / RUN_SPEED) * 20.0, 10.0, 30.0
        ))

    if flags.get("face_direction") and not flags.get("snappy_shake"):
        t_local = timers.get("face_direction", 0.0)
        val, next_t = simple_face_direction(
            flags["direction_deg"], controller=movement_ctrl,
            start_time=None, duration=0.2, t=t_local
        )
        timers["face_direction"] = next_t
        output_arr[0] = val

    if flags.get("idle_sway"):
        t_local = timers.get("idle_sway", 0.0)
        val, next_t = idle_sway(output_arr[0], t=t_local, dt=dt, controller=movement_ctrl)
        timers["idle_sway"] = next_t
        output_arr[0] = val

    # Update state
    state_mem["prev_dir"] = flags.get("direction_deg", prev_dir)
    state_mem["t"] = state_mem.get("t", 0.0) + dt


def stream_behaviour(
    inputs: Iterable[Sequence[float]],
    movement_ctrl: Optional[object] = None,
    dt: float = 0.02,
    on_off: bool = True,
    initial_output: Optional[np.ndarray] = None,
    periodic_generate: bool = False,
) -> tuple[np.ndarray, np.ndarray | None]:
    """Run behaveControl over an iterable of input samples.

    This adapter is useful for feeding live streams of [x_vel,y_vel,state]
    into the behavior layer. Two modes are supported:

    - Live-only: no `movement_ctrl` provided. `behaveControl` mutates a small
      output array each timestep and the final output is returned.
    - Controller-backed: pass a `MovementController` instance. Primitives will
      schedule moves on the controller. At the end, if `periodic_generate`
      is True the controller's `generate()` will be invoked and the produced
      trajectory returned.

    Args:
        inputs: iterable of [x_vel, y_vel, state] samples.
        movement_ctrl: optional MovementController to schedule moves on.
        dt: timestep used for behaveControl's timekeeping.
        on_off: whether behavior is enabled.
        initial_output: optional starting output array (shape (3,)).
        periodic_generate: if True and movement_ctrl provided, call
            `movement_ctrl.generate()` at the end and return the trajectory.

    Returns:
        (final_output_arr, trajectory_or_None)
    """
    out = np.zeros(3) if initial_output is None else initial_output.copy()
    mem: Dict = {}
    for sample in inputs:
        behaveControl(sample, on_off, out, state_mem=mem, dt=dt, movement_ctrl=movement_ctrl)

    traj = None
    if movement_ctrl is not None and periodic_generate:
        try:
            traj = movement_ctrl.generate(output_degrees=True)
        except Exception:
            traj = None
    return out, traj


def behaveControl_from_iterable(
    inputs: Iterable[Sequence[float]] | np.ndarray,
    on_off: bool,
    initial_output: Optional[np.ndarray] = None,
    dt: float = 0.02,
    iterations: Optional[int] = None,
    stop_predicate: Optional[Callable[[], bool]] = None,
) -> np.ndarray:
    """Run behaveControl over either an iterable of inputs or a shared input array.

    Behavior:
    - If `inputs` is a numpy array of shape (3,) (a shared, constantly-updating
      input buffer), this function delegates to `behaveControl_live`, which polls
      the shared buffer each dt. Pass `iterations` or `stop_predicate` to control
      termination for the live case.
    - Otherwise `inputs` is treated as a finite iterable of samples and this
      function iterates over it, calling `behaveControl` for each sample
      (backwards-compatible behavior).

    Args:
        inputs: iterable of [x_vel,y_vel,state] samples or a shared numpy array shape (3,)
        on_off: whether behaviour is enabled
        initial_output: optional starting output array (shape (3,))
        dt: timestep
        iterations: for live mode, number of iterations to run (None -> run until stop_predicate/interrupt)
        stop_predicate: callable used in live mode to decide when to stop

    Returns:
        Final output array (shape (3,)).
    """
    # Detect shared numpy array live-mode: prefer the live behavior by default
    if isinstance(inputs, np.ndarray) and inputs.shape == (3,):
        return behaveControl_live(inputs, on_off=on_off, output=initial_output, state_mem=None, dt=dt, movement_ctrl=None, iterations=iterations, stop_predicate=stop_predicate)

    # Fallback: inputs is a finite iterable -> preserve existing behavior
    out = np.zeros(3) if initial_output is None else initial_output.copy()
    mem: Dict = {}
    it = 0
    for sample in inputs:
        if iterations is not None and it >= iterations:
            break
        behaveControl(sample, on_off, out, state_mem=mem, dt=dt)
        it += 1
    return out


def behaveControl_live(
    shared_input: np.ndarray,
    on_off: bool = True,
    output: Optional[np.ndarray] = None,
    state_mem: Optional[Dict] = None,
    dt: float = 0.02,
    movement_ctrl: Optional[object] = None,
    iterations: Optional[int] = None,
    stop_predicate: Optional[Callable[[], bool]] = None,
) -> np.ndarray:
    """Run behaveControl in a live loop polling a shared input array.

    This function expects `shared_input` to be a mutable sequence-like object
    (typically a numpy array) of length >= 3 whose contents are updated
    externally by another thread/process or the main loop. Each dt seconds
    this function will read the current sample (first three elements) and
    call `behaveControl(sample, ...)`, mutating/returning an output array.

    Args:
        shared_input: mutable sequence (e.g. numpy array) containing [x_vel, y_vel, state]
        on_off: initial on/off flag (loop continues while True and stop_predicate allows)
        output: optional output array (shape (3,)) to mutate; created if None
        state_mem: optional persistent state dict; if None, module default is used
        dt: timestep in seconds between calls
        movement_ctrl: optional MovementController for scheduling (kept optional)
        iterations: if provided, run exactly this many iterations and return
        stop_predicate: optional callable returning True to continue, False to stop

    Returns:
        The final output array (numpy array shape (3,)).

    Note: If iterations is None and stop_predicate is not provided, this will run
    indefinitely until `on_off` is set False or interrupted. Use iterations or
    provide a stop_predicate in typical tests.
    """
    if output is None:
        out = np.zeros(3)
    else:
        out = output

    # Use the same state_mem semantics as behaveControl (module default when None)
    if state_mem is None:
        state_mem = _default_state_mem

    it = 0
    try:
        while True:
            # stop conditions
            if not on_off:
                break
            if stop_predicate is not None and not stop_predicate():
                break
            if iterations is not None and it >= iterations:
                break

            # sample the shared input (assume at least 3 elements)
            sample = (float(shared_input[0]), float(shared_input[1]), int(shared_input[2]))
            behaveControl(sample, True, out, state_mem=state_mem, dt=dt, movement_ctrl=movement_ctrl)

            it += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        # allow graceful interruption
        pass

    return out


__all__ = [
    "behaveControl",
    "behaveControl_from_iterable",
    "behaveControl_live",
]
