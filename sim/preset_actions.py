"""preset_actions.py

Integration module for head_behave preset motions into MuJoCo simulation.
Maps HeadActions enum to motion primitives and applies them to joint targets.

Motor mapping:
- angle0 = h (head rotation)
- angle1 = a1 (arm segment 1)
- angle2 = a2 (arm segment 2)
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
    global _timers, _current_action, _animation_start_time, _total_animation_time
    
    # Check if action has changed or needs to start
    if state.head_actions != _current_action:
        if state.head_actions != HeadActions.NONE:
            # New action started
            print(f"[PRESET] Starting action: {state.head_actions.name}")
            _current_action = state.head_actions
            _animation_start_time = 0.0
            _total_animation_time = 0.0
            # Reset timer for this action
            timer_key = state.head_actions.name
            _timers[timer_key] = 0.0
        else:
            # Action was manually reset to NONE
            _current_action = HeadActions.NONE
            return
    
    # If no action is active, do nothing
    if state.head_actions == HeadActions.NONE:
        return
    
    # Track total animation time
    _total_animation_time += dt
    
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
    # [h, a1, a2] in degrees
    output = np.array([
        np.degrees(state.target_h_pos),
        np.degrees(state.target_a1_pos),
        np.degrees(state.target_a2_pos)
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
        # Fast motion - quick movements
        # Simple oscillation for demonstration
        freq = 3.0  # Hz
        output[0] = 30.0 * np.sin(2.0 * np.pi * freq * t)
        output[1] = 15.0 * np.sin(2.0 * np.pi * freq * t * 0.5)
        _timers[timer_key] = (t + dt) % (1.0 / freq)
        
    elif state.head_actions == HeadActions.EXPRESSION_FAST_TURN:
        # Fast turn - rapid rotation
        freq = 2.0
        output[0] = 45.0 * np.sin(2.0 * np.pi * freq * t)
        output[2] = np.degrees(1.0 + 0.2 * np.sin(2.0 * np.pi * freq * t * 0.3))
        _timers[timer_key] = (t + dt) % (1.0 / freq)
        
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
    # Clamp values to valid ranges
    state.target_h_pos = np.clip(np.radians(output[0]), -3.14, 3.14)
    state.target_a1_pos = np.clip(np.radians(output[1]), -3.14, 3.14)
    state.target_a2_pos = np.clip(np.radians(output[2]), 0.7, 1.5)
