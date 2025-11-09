"""head_behave.py

Generate time-stepped 3-angle trajectories for a robot head and convert them
to a matrix of angles for every timestep

Outputs:
- A numpy array of shape (N, 4) where columns are [t, angle_0_deg, angle_1_deg, angle_2_deg]
- A simple motor command conversion: angles -> radians and constrained by max
  angular velocity derived from motor max RPM.

"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


@dataclass
class MotorModel:
    """ holds a motor variables, change here """

    default_timestep: float = 0.002
    default_min_duration: float = 0.02

def generate_trajectory_per_axis(start: Sequence[float], end: Sequence[float], durations: Sequence[float], timestep: float | None = None, motor: MotorModel | None = None) -> np.ndarray:
    """Generate per-axis trajectories with parabolic (ease-in/ease-out) profiles.

    Each axis moves according to a quadratic ease-in/ease-out profile (smooth
    acceleration then deceleration). After an axis reaches its target it holds
    the final value until the global sequence end. The function returns an
    Nx4 array with columns [t, a0_deg, a1_deg, a2_deg].

    Args:
        start: 3-element start angles in degrees
        end: 3-element end angles in degrees
        durations: 3-element per-axis finish times (seconds)
        timestep: command timestep (seconds); if None and motor provided, uses motor.default_timestep; if both None uses 0.02
        motor: optional MotorModel instance (used for default timestep if timestep is None)

    Returns:
        Nx4 array [t, a0_deg, a1_deg, a2_deg] sampled at common timestep from t=0 to max(durations).
    """
    if timestep is None:
        if motor is not None:
            timestep = motor.default_timestep
        else:
            timestep = 0.002  # fallback default
    
    assert len(start) == 3 and len(end) == 3 and len(durations) == 3, "start/end/durations must be 3-element sequences"
    if any(T <= 0 for T in durations):
        raise ValueError("all durations must be positive")

    T_max = max(durations)
    times = np.arange(0.0, T_max + 1e-9, timestep)
    out = np.zeros((len(times), 4), dtype=float)
    out[:, 0] = times

    # Ease-in-out quadratic function (u in [0,1])
    def ease_in_out_quad(u: np.ndarray) -> np.ndarray:
        # vectorized: u can be scalar or array
        u = np.asarray(u)
        out_u = np.empty_like(u, dtype=float)
        mask = u < 0.5
        out_u[mask] = 2.0 * u[mask] ** 2
        out_u[~mask] = -1.0 + (4.0 - 2.0 * u[~mask]) * u[~mask]
        return out_u

    # For each axis compute eased profile and hold after finish
    for ch in range(3):
        p0 = float(start[ch])
        p1 = float(end[ch])
        Ti = float(durations[ch])
        if Ti == 0.0:
            out[:, ch + 1] = p1
            continue

        t = times
        # normalized time in [0,1] for the axis; values >1 will be clipped
        u = np.minimum(1.0, t / Ti)
        eased = ease_in_out_quad(u)
        vals = p0 + (p1 - p0) * eased
        # for times after Ti, ensure hold at p1
        vals = np.where(t <= Ti, vals, p1)
        out[:, ch + 1] = vals

    return out

class MovementController:
    """Controller that manages movements dynamically - you can add moves while others are running.
    
    This is the main interface for creating robot head movements. Call move() to add
    movements at any time, and they will be overlaid on the current motion.
    """
    
    def __init__(self, motor: MotorModel):
        """Initialize the controller with a motor model.
        
        Args:
            motor: MotorModel instance defining motor specs and defaults
        """
        self.motor = motor
        # Each move: (start_time, target(3), durations(3)|None, speed)
        self.moves: list[tuple[float, Sequence[float], Sequence[float] | None, float]] = []
        self.current_time = 0.0
        self._positions_cache: np.ndarray | None = None
        self._cache_valid = False
        # Live auxiliary array updated during generate_with_aux: [vx, vy, angular velocity, state]
        self.aux_live = np.zeros(4, dtype=float)
    
    def move(self, target: Sequence[float], durations: Sequence[float] | None = None, start_time: float | None = None, speed: float = 1.0) -> 'MovementController':
        """Add a movement starting at the specified time (or now if not specified).
        
        This is the MAIN function you use to create movements. You can call this
        while another movement is running, and the new movement will be added to
        the sequence.
        
        Args:
            target: 3-element target position in degrees [motor0, motor1, motor2]
            durations: optional 3-element durations in seconds (None = auto-plan)
            start_time: when to start this move (None = current_time)
        
        Returns:
            self (for method chaining)
        
        Example:
            controller = MovementController(motor)
            
            # Move motor 0 to 90° starting now
            controller.move([90, 0, 0])
            
            # After 2.5s, also move motor 1 to 60° (while motor 0 is still moving)
            controller.move([90, 60, 0], start_time=2.5)
            
            # Generate the complete command sequence
            commands = controller.generate()
        """
        if start_time is None:
            start_time = self.current_time

        if float(speed) <= 0.0:
            raise ValueError("speed must be positive")

        # Store the move; durations may be None (auto-plan), speed is applied when planning
        self.moves.append((start_time, list(target), None if durations is None else list(durations), float(speed)))
        self._cache_valid = False
        
        # Update current_time to end of this move
        if durations is not None:
            # we don't pre-scale durations here; scaling by speed is applied in _generate_positions
            move_end = start_time + max(durations)
            self.current_time = max(self.current_time, move_end)
        
        return self
    
    def stop_all(self) -> 'MovementController':
        """ stops everything and freezes in place """

        if not self.moves:
            return self
        
        # Generate positions up to current_time
        if not self._cache_valid:
            self._generate_positions()
        
        # Find current position at current_time
        if self._positions_cache is not None and len(self._positions_cache) > 0:
            timestep = self.motor.default_timestep
            current_step = min(int(self.current_time / timestep), len(self._positions_cache) - 1)
            hold_position = self._positions_cache[current_step].copy()
            
            # Clear all moves and replace with a single hold at current position
            self.moves = [(self.current_time, hold_position, [0.01, 0.01, 0.01], 1.0)]
            self._cache_valid = False
        
        return self
    
    def _generate_positions(self) -> np.ndarray:
        """Internal: generate the position matrix from all queued moves."""

        if not self.moves:
            # No moves - return a single timestep at origin
            return np.zeros((1, 3))
        
        # Sort moves by start time
        sorted_moves = sorted(self.moves, key=lambda m: m[0])
        
        # Calculate total duration needed
        max_end_time = 0.0
        for move in sorted_moves:
            if len(move) == 4:
                start_time, end_pos, durations, move_speed = move
            else:
                start_time, end_pos, durations = move
            if durations is None:
                durations = [self.motor.default_min_duration] * 3
            # Note: we don't scale by speed here; scaling is applied when planning trajectories
            move_end = start_time + max(durations)
            max_end_time = max(max_end_time, move_end)
        
        # Create timesteps
        timestep = self.motor.default_timestep
        times = np.arange(0.0, max_end_time + timestep/2, timestep)
        num_steps = len(times)
        
        # Initialize output with starting position (0,0,0)
        positions = np.zeros((num_steps, 3))
        
        # Track which timesteps are controlled by which move for each axis
        # Format: axis_control[axis][timestep] = move_index
        axis_control = [{} for _ in range(3)]
        
        # First pass: determine which move controls each axis at each timestep
        for move_idx, move in enumerate(sorted_moves):
            # move tuple: (start_time, end_pos, durations, move_speed)
            if len(move) == 4:
                start_time, end_pos, durations, move_speed = move
            else:
                # backward compatibility (shouldn't happen) - default speed=1.0
                start_time, end_pos, durations = move
                move_speed = 1.0
            start_step = int(start_time / timestep)
            if start_step >= num_steps:
                continue
            
            # Plan durations if needed
            start_pos = positions[start_step].copy()
            if durations is None:
                # Hardware timing removed; use default min duration for motion-only mode
                durations = [self.motor.default_min_duration] * 3
            # Apply speed: higher speed -> shorter durations
            durations = [float(d) / float(move_speed) for d in durations]
            
            # For each axis that's moving in this move
            for axis in range(3):
                if abs(end_pos[axis] - start_pos[axis]) > 0.01:  # This axis is moving
                    axis_end_time = start_time + durations[axis]
                    axis_end_step = min(int(axis_end_time / timestep), num_steps - 1)
                    
                    # Mark timesteps controlled by this move for this axis
                    for step in range(start_step, axis_end_step + 1):
                        current_time = step * timestep
                        if current_time > axis_end_time:
                            break
                        
                        # Check if there's already a move controlling this axis at this time
                        if step in axis_control[axis]:
                            # There's a conflict - check if they target the same position
                            prev_move_idx = axis_control[axis][step]
                            prev_move = sorted_moves[prev_move_idx]
                            if len(prev_move) == 4:
                                prev_start, prev_end, prev_durs, prev_speed = prev_move
                            else:
                                prev_start, prev_end, prev_durs = prev_move

                            # If targeting same position, keep the earlier move
                            if abs(prev_end[axis] - end_pos[axis]) < 0.01:
                                # Same target - earlier move takes precedence
                                continue
                            else:
                                # Different targets - later move overrides
                                axis_control[axis][step] = move_idx
                        else:
                            # No conflict - this move controls this timestep
                            axis_control[axis][step] = move_idx
        
        # Second pass: generate trajectories based on control assignments
        for move_idx, move in enumerate(sorted_moves):
            # move tuple: (start_time, end_pos, durations, move_speed)
            if len(move) == 4:
                start_time, end_pos, durations, move_speed = move
            else:
                start_time, end_pos, durations = move
                move_speed = 1.0
            start_step = int(start_time / timestep)
            if start_step >= num_steps:
                continue
            
            start_pos = positions[start_step].copy()
            
            # Plan durations if needed
            if durations is None:
                # Hardware timing removed; use default min duration for motion-only mode
                durations = [self.motor.default_min_duration] * 3
            # Apply speed scaling (shorter durations for higher speed)
            durations = [float(d) / float(move_speed) for d in durations]
            
            # Generate trajectory for this move
            move_traj = generate_trajectory_per_axis(start_pos, end_pos, durations, timestep, self.motor)
            
            # Apply trajectory only to timesteps/axes controlled by this move
            for i, t_offset in enumerate(move_traj[:, 0]):
                global_step = start_step + i
                if global_step >= num_steps:
                    break
                
                for axis in range(3):
                    # Only update if this move controls this axis at this timestep
                    if global_step in axis_control[axis] and axis_control[axis][global_step] == move_idx:
                        positions[global_step, axis] = move_traj[i, axis + 1]
        
        self._positions_cache = positions
        self._cache_valid = True
        return positions
    
    def generate(self, output_degrees: bool = True) -> np.ndarray:
        """Generate the final motor command sequence from all queued movements.
        
        This applies velocity limiting and returns the complete command matrix.
        Call this after you've added all your moves with move().
        
        Args:
            output_degrees: if True, return degrees; if False, return radians
        
        Returns:
            Nx3 array of motor command positions (degrees by default)
        
        Example:
            controller = MovementController(motor)
            controller.move([90, 0, 0], [5.0, 5.0, 5.0])
            controller.move([90, 60, 0], start_time=2.5)
            
            commands = controller.generate()  # Nx3 array ready to send to motors
        """
        positions = self._generate_positions()
        # Motion-only mode: no velocity clipping or motor-bound enforcement.
        # Return degrees by default; convert to radians if requested.
        return positions if output_degrees else np.deg2rad(positions)

    # Note: velocity clipping and motor-bound enforcement removed. The
    # controller now focuses purely on generating motion trajectories.
    
    def reset(self) -> 'MovementController':
        """Clear all movements and reset to initial state.
        
        Returns:
            self (for method chaining)
        """
        self.moves = []
        self.current_time = 0.0
        self._positions_cache = None
        self._cache_valid = False
        return self


def _schedule_move(controller: MovementController, *, target: list[float] | None = None,
                   pan: float | None = None, tilt: float | None = None,
                   speed: float = 1.0, base_duration: float | None = None,
                   start_time: float = 0.0) -> None:
    """Helper that schedules a move on a controller.

    You can either pass `target` (3-element list) or `pan`+`tilt`.
    If `base_duration` is not provided it uses a small sensible default.
    """
    if target is None:
        if pan is None or tilt is None:
            raise ValueError("Either target or pan+tilt must be provided")
        target = [float(pan), float(tilt), 0.0]

    dur = (1.5 if base_duration is None else float(base_duration)) / float(max(1e-6, speed))
    durations = [dur, dur, dur]
    controller.move(target, durations=durations, start_time=start_time)
