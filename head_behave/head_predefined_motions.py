"""head_predefined_motions.py

Pre-defined motion primitives for robot head behavior.
"""

from __future__ import annotations

import math
from typing import Optional, Dict

import numpy as np


def idle_sway(
    current_head_rotate: float,
    t: float = 0.0,
    dt: float = 0.02,
    controller: Optional[object] = None,
    start_time: Optional[float] = None,
    cycles: int = 1
) -> tuple[float, float]:
    """Idle sway motion - head moves slightly to not be a statue."""
    period = 4.0
    
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # Schedule one cycle only when t==0 to avoid duplicates
        if abs(t) < 1e-9:
            for c in range(max(1, cycles)):
                base_t = st + c * period
                controller.move([9.0, 0.0, 0.0], durations=[period / 3.0] * 3, start_time=base_t)
                controller.move([-9.0, 0.0, 0.0], durations=[period / 3.0] * 3, start_time=base_t + period / 3.0)
                controller.move([0.0, 0.0, 0.0], durations=[period / 3.0] * 3, start_time=base_t + 2.0 * period / 3.0)
        next_t = (t + dt) % period
        return float(current_head_rotate), next_t

    # No controller: return continuous sinusoidal value
    angle = 9.0 * math.sin(2.0 * math.pi * (t / period))
    next_t = (t + dt) % period
    return angle, next_t


def set_base_to_velocity_and_bend(
    direction_deg: float,
    speed: float,
    dir_change: float,
    controller: Optional[object] = None,
    start_time: Optional[float] = None,
    duration: float = 0.2,
    t: float = 0.0,
    dt: float = 0.02
) -> tuple[float, float]:
    """Set base rotate to face velocity direction; return base_rotate."""
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        # Schedule only when timer is at zero to avoid duplicates
        if abs(t) < 1e-9:
            head_bend = float(min(30.0, 10.0 + (speed / 15.0) * 20.0 + (dir_change / 180.0) * 10.0))
            controller.move([direction_deg, head_bend, direction_deg], durations=[duration] * 3, start_time=st)
        next_t = (t + dt) % duration
        return direction_deg, next_t
    
    next_t = (t + dt) % duration
    return float(direction_deg), next_t


def snappy_shake_with_bend(
    direction_deg: float,
    prev_head_rotate: float,
    speed: float,
    t: float = 0.0,
    dt: float = 0.02,
    controller: Optional[object] = None,
    start_time: Optional[float] = None,
    duration: float = 0.15
) -> tuple[float, float]:
    """Head rotate roughly toward direction with a small snappy shake."""
    # Small-amplitude fast oscillation superimposed on direction
    wobble = 5.0 * math.sin(2.0 * math.pi * 4.0 * t) * 0.05
    # Interpolate toward direction for snappy effect
    alpha = min(1.0, 0.3 + 0.7 * min(1.0, speed / 15.0))
    value = float(prev_head_rotate + (direction_deg + wobble - prev_head_rotate) * alpha)
    
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        if abs(t) < 1e-9:
            controller.move([value, 0.0, 0.0], durations=[duration] * 3, start_time=st)
        next_t = (t + dt) % duration
        return value, next_t
    
    next_t = (t + dt) % duration
    return value, next_t


def simple_face_direction(
    direction_deg: float,
    controller: Optional[object] = None,
    start_time: Optional[float] = None,
    duration: float = 0.2,
    t: float = 0.0,
    dt: float = 0.02
) -> tuple[float, float]:
    """Return head_rotate value that faces the direction (deg)."""
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        if abs(t) < 1e-9:
            controller.move([direction_deg, 0.0, direction_deg], durations=[duration] * 3, start_time=st)
        next_t = (t + dt) % duration
        return direction_deg, next_t
    
    next_t = (t + dt) % duration
    return float(direction_deg), next_t


def inquisitive_pose(
    output_arr: np.ndarray,
    controller: Optional[object] = None,
    start_time: Optional[float] = None
) -> None:
    """Turn into an inquisitive pose (mutates output_arr in-place).
    
    Sets: base_rotate=100°, head_bend=15°, head_rotate=-100°
    """
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        controller.move([-100.0, 15.0, 100.0], durations=[0.6, 0.6, 0.6], start_time=st)
        return

    output_arr[2] = 100.0   # base_rotate
    output_arr[1] = 15.0    # head_bend
    output_arr[0] = -100.0  # head_rotate


def head_shake(
    output_arr: np.ndarray,
    dt: float = 0.02,
    controller: Optional[object] = None,
    start_time: Optional[float] = None,
    t: float = 0.0
) -> float:
    """Simple back-and-forth head rotate mutation."""
    period = 0.6
    
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        if abs(t) < 1e-9:
            controller.move([20.0, 0.0, 0.0], durations=[0.15, 0.15, 0.15], start_time=st)
            controller.move([-20.0, 0.0, 0.0], durations=[0.15, 0.15, 0.15], start_time=st + 0.15)
            controller.move([0.0, 0.0, 0.0], durations=[0.15, 0.15, 0.15], start_time=st + 0.30)
        return (t + dt) % period

    # Live mode
    output_arr[0] = 20.0 * math.sin(2.0 * math.pi * (t / period))
    return (t + dt) % period


def head_spin(
    output_arr: np.ndarray,
    dt: float = 0.02,
    controller: Optional[object] = None,
    start_time: Optional[float] = None,
    t: float = 0.0
) -> tuple[float, float]:
    """Rotate head continuously (360° rotation)."""
    period = 2.0
    
    if controller is not None:
        st = start_time if start_time is not None else getattr(controller, "current_time", 0.0)
        if abs(t) < 1e-9:
            controller.move([360.0, 0.0, 0.0], durations=[period, period, period], start_time=st)
        return 0.0, (t + dt) % period

    # Live mode: compute value and advance timer
    val = (t / period) * 360.0 % 360.0
    output_arr[0] = val
    return val, (t + dt) % period


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


def stop_all_motion(
    output_arr: Optional[np.ndarray] = None,
    controller: Optional[object] = None
) -> None:
    """Stop all motion and return to neutral position."""
    if controller is not None:
        try:
            controller.stop_all()
        except Exception:
            pass
    
    if output_arr is not None:
        output_arr[:] = 0.0


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
