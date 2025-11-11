"""preset_actions.py

Integration module for head_behave preset motions into MuJoCo simulation.
Maps HeadActions enum to motion primitives and applies them to joint targets.

Motor mapping for head_behave primitives:
- output[0] = head_rotate -> h (head rotation)
- output[1] = head_bend -> a2 (arm segment 2 - vertical tilt)
- output[2] = base_rotate -> a1 (arm segment 1 - base rotation)
"""

import numpy as np
from control_state import state
from enums import HeadActions

# Import motion primitives from head_behave
import sys
import os
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from head_behave.head_predefined_motions import (
    idle_sway,
    inquisitive_pose,
    head_shake,
    head_spin,
    hurt_sequence,
)

# Module-level state for per-primitive timers and animation tracking
_timers = {}
_current_action = HeadActions.NONE
_animation_start_time = 0.0
_total_animation_time = 0.0
_return_to_upright_phase = False
_upright_duration = 0.5  # Time in seconds to return to upright

# Upright position (in degrees)
_upright_position = {
    'h': 0.0,     # Head centered
    'a2': 0.0,    # Arm 2 upright (0 degrees = vertical)
    'a1': 0.0     # Base centered
}

# Animation durations for each action (in seconds)
_animation_durations = {
    HeadActions.EXPRESSION_IDLE: 4.0,
    HeadActions.EXPRESSION_FAST: 1.5,
    HeadActions.EXPRESSION_FAST_TURN: 1.0,
    HeadActions.EXPRESSION_INQUISITIVE: 3.0,
    HeadActions.EXPRESSION_HEAD_SHAKE: 2.0,
    HeadActions.EXPRESSION_HEAD_SPIN: 4.0,
    HeadActions.EXPRESSION_HURT: 3.0,
}


def update_preset_actions(dt: float = 0.001) -> None:
    """Update joint targets based on active preset action.
    
    This function should be called every simulation step. It reads the current
    HeadActions from the global state and applies the corresponding motion
    primitive to update the target joint positions.
    
    Args:
        dt: Simulation timestep in seconds
        
    Motor mapping:
        output[0] -> h (head rotation)
        output[1] -> a1 (arm segment 1) 
        output[2] -> a2 (arm segment 2)
    """
    global _timers, _current_action, _animation_start_time, _total_animation_time, _return_to_upright_phase
    
    # Check if action has changed or needs to start
    if state.head_actions != _current_action:
        if state.head_actions != HeadActions.NONE:
            # New action started - begin with return to upright phase
            print(f"[PRESET] Starting action: {state.head_actions.name} - returning to upright first")
            _current_action = state.head_actions
            _animation_start_time = 0.0
            _total_animation_time = 0.0
            _return_to_upright_phase = True
            # Reset timer for this action
            timer_key = state.head_actions.name
            _timers[timer_key] = 0.0
        else:
            # Action was manually reset to NONE
            _current_action = HeadActions.NONE
            _return_to_upright_phase = False
            return
    
    # If no action is active, do nothing
    if state.head_actions == HeadActions.NONE:
        return
    
    # Track total animation time
    _total_animation_time += dt
    
    # Handle return to upright phase
    if _return_to_upright_phase:
        if _total_animation_time < _upright_duration:
            # Interpolate to upright position
            progress = _total_animation_time / _upright_duration
            # Use smooth interpolation (ease-out)
            smooth_progress = 1 - (1 - progress) ** 2
            
            # Get current positions in degrees
            current_h = np.degrees(state.target_h_pos)
            current_a2 = np.degrees(state.target_a2_pos)
            current_a1 = np.degrees(state.target_a1_pos)
            
            # Interpolate towards upright
            target_h = current_h + ((_upright_position['h'] - current_h) * smooth_progress)
            target_a2 = current_a2 + ((_upright_position['a2'] - current_a2) * smooth_progress)
            target_a1 = current_a1 + ((_upright_position['a1'] - current_a1) * smooth_progress)
            
            # Apply interpolated positions
            state.target_h_pos = np.clip(np.radians(target_h), -3.14, 3.14)
            state.target_a2_pos = np.clip(np.radians(target_a2), -1.57, 1.57)
            state.target_a1_pos = np.clip(np.radians(target_a1), -3.14, 3.14)
            return
        else:
            # Upright phase complete, start actual animation
            print(f"[PRESET] Upright position reached - starting {state.head_actions.name} animation")
            _return_to_upright_phase = False
            _total_animation_time = 0.0  # Reset timer for the actual animation
            timer_key = state.head_actions.name
            _timers[timer_key] = 0.0
            return
    
    # Check if animation duration has elapsed
    action_duration = _animation_durations.get(state.head_actions, 3.0)
    if _total_animation_time >= action_duration:
        # Animation complete - unlock and reset
        print(f"[PRESET] Completed action: {state.head_actions.name} (duration: {_total_animation_time:.2f}s)")
        state.head_actions = HeadActions.NONE
        state.head_action_locked = False
        _current_action = HeadActions.NONE
        return
    
    # Create output array for the primitive functions
    # [head_rotate, head_bend, base_rotate] maps to [h, a2, a1]
    output = np.array([
        np.degrees(state.target_h_pos),   # output[0] -> h
        np.degrees(state.target_a2_pos),  # output[1] -> a2 (head_bend)
        np.degrees(state.target_a1_pos)   # output[2] -> a1 (base_rotate)
    ])
    
    # Get timer for current action
    timer_key = state.head_actions.name
    t = _timers.get(timer_key, 0.0)
    
    # Dispatch to appropriate motion primitive
    if state.head_actions == HeadActions.EXPRESSION_IDLE:
        val, next_t = idle_sway(output[0], t=t, dt=dt, controller=None)
        output[0] = val
        _timers[timer_key] = next_t
        
    elif state.head_actions == HeadActions.EXPRESSION_FAST:
        # Fast motion - quick movements using position control
        # Simple oscillation for demonstration
        freq = 3.0  # Hz
        target_h = 30.0 * np.sin(2.0 * np.pi * freq * t)
        target_a1 = 15.0 * np.sin(2.0 * np.pi * freq * t * 0.5)
        output[0] = target_h
        output[1] = target_a1
        _timers[timer_key] = t + dt
        
    elif state.head_actions == HeadActions.EXPRESSION_FAST_TURN:
        # Fast turn - rapid rotation using position control
        freq = 2.0
        target_h = 45.0 * np.sin(2.0 * np.pi * freq * t)
        target_a2_deg = np.degrees(1.0 + 0.2 * np.sin(2.0 * np.pi * freq * t * 0.3))
        output[0] = target_h
        output[2] = target_a2_deg
        _timers[timer_key] = t + dt
        
    elif state.head_actions == HeadActions.EXPRESSION_INQUISITIVE:
        inquisitive_pose(output, controller=None)
        _timers[timer_key] = t + dt
        
    elif state.head_actions == HeadActions.EXPRESSION_HEAD_SHAKE:
        next_t = head_shake(output, dt=dt, controller=None, t=t)
        _timers[timer_key] = next_t
        
    elif state.head_actions == HeadActions.EXPRESSION_HEAD_SPIN:
        val, next_t = head_spin(output, dt=dt, controller=None, t=t)
        output[0] = val
        _timers[timer_key] = next_t
        
    elif state.head_actions == HeadActions.EXPRESSION_HURT:
        # Create a temporary state_mem for hurt_sequence
        if 'hurt_state' not in _timers:
            _timers['hurt_state'] = {}
        hurt_sequence(output, dt=dt, state_mem=_timers['hurt_state'], controller=None)
        _timers[timer_key] = t + dt
    
    # Convert output from degrees to radians and apply to targets
    # output[0]=h, output[1]=a2(head_bend), output[2]=a1(base_rotate)
    state.target_h_pos = np.clip(np.radians(output[0]), -3.14, 3.14)
    state.target_a2_pos = np.clip(np.radians(output[1]), -1.57, 1.57)
    state.target_a1_pos = np.clip(np.radians(output[2]), -3.14, 3.14)
