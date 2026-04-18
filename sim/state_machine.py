"""State-machine core with pluggable input and output adapters.

This module separates robot intent/state evolution from transport details:
- Input sources update the shared state (keyboard, vision, etc.).
- Output sinks consume commands (MuJoCo ctrl, real motor commands, etc.).

Swapping interfaces should only require changing adapter construction.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional, Protocol

from control_state import ControlState
from preset_actions import update_preset_actions


@dataclass
class RobotCommand:
    """Command bundle produced by the state machine each step."""

    target_a1_pos: float
    target_a2_pos: float
    target_h_pos: float
    w1_speed: float
    w2_speed: float
    w3_speed: float
    w4_speed: float


class InputSource(Protocol):
    """Input adapter interface.

    Implementations translate external observations into ControlState updates.
    """

    def update(self, state: ControlState, dt: float) -> None:
        ...


class OutputSink(Protocol):
    """Output adapter interface.

    Implementations translate RobotCommand into downstream output targets.
    """

    def send(self, command: RobotCommand, context: Optional[dict] = None) -> None:
        ...


class KeyboardInputSource:
    """No-op input source for keyboard mode.

    Keyboard events already mutate ControlState via GLFW callback.
    """

    def update(self, state: ControlState, dt: float) -> None:
        del state, dt


class VisionInputSource:
    """Vision input adapter.

    The provider should return a partial dict of ControlState field updates,
    e.g. {"body_x_speed": 0.3, "head_actions": HeadActions.EXPRESSION_IDLE}
    """

    def __init__(self, provider: Callable[[], Dict[str, object]]):
        self._provider = provider

    def update(self, state: ControlState, dt: float) -> None:
        del dt
        patch = self._provider() or {}
        for key, value in patch.items():
            if hasattr(state, key):
                setattr(state, key, value)


class MuJoCoOutputSink:
    """Output sink that writes commands to MuJoCo actuator controls."""

    def __init__(self, actuator_ids: Dict[str, int]):
        self._actuator_ids = actuator_ids

    def send(self, command: RobotCommand, context: Optional[dict] = None) -> None:
        if context is None or "data" not in context:
            return
        data = context["data"]

        id_a1 = self._actuator_ids.get("a1_motor", -1)
        id_a2 = self._actuator_ids.get("a2_motor", -1)
        id_h = self._actuator_ids.get("h_motor", -1)
        id_w1 = self._actuator_ids.get("w1_motor", -1)
        id_w2 = self._actuator_ids.get("w2_motor", -1)
        id_w3 = self._actuator_ids.get("w3_motor", -1)
        id_w4 = self._actuator_ids.get("w4_motor", -1)

        if id_a1 != -1:
            data.ctrl[id_a1] = command.target_a1_pos
        if id_a2 != -1:
            data.ctrl[id_a2] = command.target_a2_pos
        if id_h != -1:
            data.ctrl[id_h] = command.target_h_pos
        if id_w1 != -1:
            data.ctrl[id_w1] = command.w1_speed
        if id_w2 != -1:
            data.ctrl[id_w2] = command.w2_speed
        if id_w3 != -1:
            data.ctrl[id_w3] = command.w3_speed
        if id_w4 != -1:
            data.ctrl[id_w4] = command.w4_speed


class RealMotorOutputSink:
    """Output sink that forwards commands to a real hardware transport."""

    def __init__(self, sender: Callable[[RobotCommand], None]):
        self._sender = sender

    def send(self, command: RobotCommand, context: Optional[dict] = None) -> None:
        del context
        self._sender(command)


class RobotStateMachine:
    """Main control loop unit that owns the state transition/output step."""

    def __init__(
        self,
        state: ControlState,
        output_sink: OutputSink,
        input_source: Optional[InputSource] = None,
    ):
        self._state = state
        self._output_sink = output_sink
        self._input_source = input_source

    def step(self, dt: float, output_context: Optional[dict] = None) -> None:
        if self._input_source is not None:
            self._input_source.update(self._state, dt)

        update_preset_actions(dt=dt)
        cmd = self._build_command()
        self._output_sink.send(cmd, output_context)

    def _build_command(self) -> RobotCommand:
        return RobotCommand(
            target_a1_pos=self._state.target_a1_pos,
            target_a2_pos=self._state.target_a2_pos,
            target_h_pos=self._state.target_h_pos,
            w1_speed=self._state.w1_speed,
            w2_speed=self._state.w2_speed,
            w3_speed=self._state.w3_speed,
            w4_speed=self._state.w4_speed,
        )
