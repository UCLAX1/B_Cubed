"""predefined_motions.py

list of pre-defined motions
"""

from __future__ import annotations

import math
from typing import Optional, Dict

import numpy as np


def idle_sway(current_head_rotate: float, t: float = 0.0, dt: float = 0.02, controller: Optional[object] = None, start_time: Optional[float] = None, cycles: int = 1) -> tuple[float, float]:
    """when no action head will move slightly to not be a statue"""

    # If a controller is provided, schedule a short sequence of moves approximating sway
    period = 4.0
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # simple three-step sway: +amp, -amp, 0 per cycle
        # Schedule one cycle only when t==0. Caller (behaveControl) will reset
        # t to 0 when the cycle completes; this avoids scheduling duplicates.
        if abs(t) < 1e-9:
            for c in range(max(1, cycles)):
                controller.move([9.0, 0.0, 0.0], durations=[period / 3.0] * 3, start_time=st + c * period)
                controller.move([-9.0, 0.0, 0.0], durations=[period / 3.0] * 3, start_time=st + c * period + period / 3.0)
                controller.move([0.0, 0.0, 0.0], durations=[period / 3.0] * 3, start_time=st + c * period + 2.0 * period / 3.0)
        # return unchanged current and advance timer
        next_t = (t + dt)
        if next_t >= period:
            next_t = 0.0
        return float(current_head_rotate), next_t

    # No controller provided: return continuous sinusoidal value
    angle = 9.0 * math.sin(2.0 * math.pi * (t / period))
    next_t = (t + dt)
    if next_t >= period:
        next_t = 0.0
    return angle, next_t


def set_base_to_velocity_and_bend(direction_deg: float, speed: float, dir_change: float, controller: Optional[object] = None, start_time: Optional[float] = None, duration: float = 0.2, t: float = 0.0, dt: float = 0.02) -> tuple[float, float]:
    """Set base rotate to face velocity direction; return base_rotate.

    Also meant to be used together with a separate head_bend calculation.
    """
    # head bend: base value influenced by speed and directional change
    head_bend = float(min(30.0, 10.0 + (speed / 15.0) * 20.0 + (dir_change / 180.0) * 10.0))
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # schedule a move that sets head_rotate toward direction, head_bend, and base_rotate
        # only schedule when the primitive timer is at zero to avoid duplicate scheduling
        if abs(t) < 1e-9:
            controller.move([direction_deg, head_bend, direction_deg], durations=[duration] * 3, start_time=st)
        # advance timer and loop once duration passes
        next_t = (t + dt)
        if next_t >= duration:
            next_t = 0.0
        return direction_deg, next_t
    next_t = (t + dt)
    if next_t >= duration:
        next_t = 0.0
    return float(direction_deg), next_t


def snappy_shake_with_bend(direction_deg: float, prev_head_rotate: float, speed: float, t: float = 0.0, dt: float = 0.02, controller: Optional[object] = None, start_time: Optional[float] = None, duration: float = 0.15) -> tuple[float, float]:
    """Have head rotate roughly toward direction but with a small snappy shake.

    This returns a new head_rotate value (deg).
    """
    # Small-amplitude fast oscillation superimposed on direction
    wobble_amp = 5.0  # degrees max
    wobble_freq = 4.0  # Hz (fast)
    wobble = wobble_amp * math.sin(2.0 * math.pi * wobble_freq * t) * 0.05
    # Interpolate a fraction toward the direction each step for snappy effect
    alpha = min(1.0, 0.3 + 0.7 * min(1.0, speed / 15.0))
    target = direction_deg
    value = float(prev_head_rotate + (target + wobble - prev_head_rotate) * alpha)
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # schedule a short snappy move toward 'value' only when timer==0
        if abs(t) < 1e-9:
            controller.move([value, 0.0, 0.0], durations=[duration] * 3, start_time=st)
        next_t = (t + dt)
        if next_t >= duration:
            next_t = 0.0
        return value, next_t
    next_t = (t + dt)
    if next_t >= duration:
        next_t = 0.0
    return value, next_t


def simple_face_direction(direction_deg: float, controller: Optional[object] = None, start_time: Optional[float] = None, duration: float = 0.2, t: float = 0.0, dt: float = 0.02) -> tuple[float, float]:
    """Return a simple head_rotate value that faces the direction (deg).

    If a controller is provided, schedule an absolute move to face the direction.
    """
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # schedule only when primitive timer is at zero
        if abs(t) < 1e-9:
            controller.move([direction_deg, 0.0, direction_deg], durations=[duration] * 3, start_time=st)
        next_t = (t + dt)
        if next_t >= duration:
            next_t = 0.0
        return direction_deg, next_t
    next_t = (t + dt)
    if next_t >= duration:
        next_t = 0.0
    return float(direction_deg), next_t


def inquisitive_pose(output_arr: np.ndarray, controller: Optional[object] = None, start_time: Optional[float] = None) -> None:
    """Turn into an inquisitive pose (mutates output_arr in-place).

    The pseudo-code requested:
    [Turn base rotate 100 degrees
     Turn head bend 15 degrees
     Turn head rotate -100]
    """
    # If a MovementController is provided, schedule this as a move sequence
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # Schedule a single simultaneous pose (absolute angles)
        controller.move([-100.0, 15.0, 100.0], durations=[0.6, 0.6, 0.6], start_time=st)
        return

    output_arr[2] = 100.0  # base_rotate
    output_arr[1] = 15.0   # head_bend
    output_arr[0] = -100.0 # head_rotate


def head_shake(output_arr: np.ndarray, dt: float = 0.02, controller: Optional[object] = None, start_time: Optional[float] = None, t: float = 0.0) -> float:
    """Simple back-and-forth head rotate mutation.

    This toggles head_rotate left/right based on a short period.
    """
    # new signature: accept t and return next_t when used via behaveControl
    # To keep backward compatibility, we accept optional t kwarg.
    def _impl(t_val: float):
        amp = 20.0
        period = 0.6
        output_arr[0] = amp * math.sin(2.0 * math.pi * (t_val / period))
        next_t_local = (t_val + dt)
        if next_t_local >= period:
            next_t_local = 0.0
        return next_t_local

    if controller is not None:
        # Schedule a quick left-right shake over ~0.6s, but only when timer==0
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        if abs(t) < 1e-9:
            controller.move([20.0, 0.0, 0.0], durations=[0.15, 0.15, 0.15], start_time=st)
            controller.move([-20.0, 0.0, 0.0], durations=[0.15, 0.15, 0.15], start_time=st + 0.15)
            controller.move([0.0, 0.0, 0.0], durations=[0.15, 0.15, 0.15], start_time=st + 0.30)
        # return next timer value so behaveControl will advance and avoid re-scheduling
        next_t = (t + dt)
        period = 0.6
        if next_t >= period:
            next_t = 0.0
        return next_t

    # live-mode: we expect caller to pass t as kwarg; but maintain previous
    # behavior if not provided by using an internal attribute
    # (behaveControl will pass a timer so this branch is primarily for safety)
    t_val = t if t is not None else getattr(head_shake, "t", 0.0)
    next_t = _impl(t_val)
    head_shake.t = t_val + dt
    return next_t


def head_spin(output_arr: np.ndarray, dt: float = 0.02, controller: Optional[object] = None, start_time: Optional[float] = None, t: float = 0.0) -> tuple[float, float]:
    """Rotate head continuously (used by state 5)."""
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # Schedule a 360-degree rotation in 2 seconds only when timer==0
        if abs(t) < 1e-9:
            controller.move([360.0, 0.0, 0.0], durations=[2.0, 2.0, 2.0], start_time=st)
        next_t = (t + dt)
        if next_t >= 2.0:
            next_t = 0.0
        return 0.0, next_t

    # live mode: compute value and advance timer
    period = 2.0
    val = (t / period) * 360.0 % 360.0
    output_arr[0] = val
    next_t = (t + dt)
    if next_t >= period:
        next_t = 0.0
    return val, next_t


def hurt_sequence(output_arr: np.ndarray, dt: float = 0.02, state_mem: Optional[Dict] = None, controller: Optional[object] = None, start_time: Optional[float] = None) -> None:
    """Multi-step hurt behavior.

    Sequence (per pseudo-code):
    - head bend -10 quickly
    - slowly go to 0 (linear easing)
    - readjust head bend 15 while head rotate shakes fast then returns
    - return to base 0
    """
    if state_mem is None:
        state_mem = {}
    phase = state_mem.get("hurt_phase", 0)
    t = state_mem.get("hurt_t", 0.0)

    if controller is not None:
        # Schedule a sequence once; use state_mem to avoid duplicates
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        if state_mem is None:
            state_mem = {}
        if not state_mem.get("hurt_scheduled", False):
            controller.move([output_arr[0], -10.0, output_arr[2]], durations=[0.05, 0.05, 0.05], start_time=st)
            controller.move([output_arr[0], 0.0, output_arr[2]], durations=[0.6, 0.6, 0.6], start_time=st + 0.05)
            controller.move([output_arr[0], 15.0, output_arr[2]], durations=[0.5, 0.5, 0.5], start_time=st + 0.65)
            # optional shake scheduled by head_shake call
            controller.move([20.0, 15.0, output_arr[2]], durations=[0.3, 0.3, 0.3], start_time=st + 0.65)
            state_mem["hurt_scheduled"] = True
        return

    if phase == 0:
        # quick bend to -10
        output_arr[1] = -10.0
        state_mem["hurt_phase"] = 1
        state_mem["hurt_t"] = 0.0
        return

    if phase == 1:
        # slowly go to 0 using a simple linear easing step
        prev = output_arr[1]
        target = 0.0
        # step a fraction toward the target each call
        output_arr[1] = prev + (target - prev) * 0.2
        state_mem["hurt_t"] = t + dt
        if abs(output_arr[1] - 0.0) < 0.5:
            state_mem["hurt_phase"] = 2
            state_mem["hurt_t"] = 0.0
        return

    if phase == 2:
        # readjust head bend 15 and shake head rotate quickly
        output_arr[1] = 15.0
        tt = state_mem.get("hurt_t", 0.0)
        amp = 25.0 * max(0.2, 1.0 - tt)
        freq = 6.0
        output_arr[0] = amp * math.sin(2.0 * math.pi * freq * tt)
        state_mem["hurt_t"] = tt + dt
        if tt > 1.0:
            state_mem["hurt_phase"] = 3
            state_mem["hurt_t"] = 0.0
        return

    # phase 3: slow return to zero for all
    output_arr[0] = 0.0
    output_arr[1] = 0.0
    output_arr[2] = 0.0
    # reset phases for next invocation
    state_mem["hurt_phase"] = 0
    state_mem["hurt_t"] = 0.0


def stop_all_motion(output_arr: Optional[np.ndarray] = None, controller: Optional[object] = None) -> None:
    """Stops everything"""
    if controller is not None:
        # Prefer to call controller.stop_all() if provided
        try:
            controller.stop_all()
        except Exception:
            pass
    if output_arr is not None:
        output_arr[0] = 0.0
        output_arr[1] = 0.0
        output_arr[2] = 0.0


__all__ = [
    "idle_sway",
    "set_base_to_velocity_and_bend",
    "snappy_shake_with_bend",
    "simple_face_direction",
    "inquisitive_pose",
    "head_shake",
    "head_spin",
    "hurt_sequence",
    "stop_all_motion",
]
