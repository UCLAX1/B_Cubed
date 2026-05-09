"""Interactive viewer for the abstract BB-8 balance model."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
from mujoco.glfw import glfw
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from balance_rl.utils.math_utils import clip_norm


KEY_SPACE = glfw.KEY_SPACE


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "assets"
        / "bb8_balance_abstract.xml",
    )
    parser.add_argument(
        "--controller",
        choices=("none", "pd"),
        default="pd",
        help="Use no balance controller or a simple PD controller.",
    )
    parser.add_argument(
        "--command",
        choices=("idle", "sweep"),
        default="sweep",
        help="Drive command shown in the viewer.",
    )
    parser.add_argument("--control-dt", type=float, default=0.02)
    parser.add_argument("--max-accel", type=float, default=16.0)
    parser.add_argument("--max-force", type=float, default=600.0)
    parser.add_argument("--pd-kp", type=float, default=7.5)
    parser.add_argument("--pd-kd", type=float, default=1.3)
    parser.add_argument("--sweep-speed", type=float, default=0.55)
    parser.add_argument("--sweep-period", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    return parser.parse_args()


class ViewerController:
    """Small interactive controller for visualizing the abstract model."""

    def __init__(self, args: argparse.Namespace, model: mujoco.MjModel):
        self.args = args
        self.paused = False
        self.reset_requested = False
        self.held_keys: set[int] = set()
        self.system_mass = float(np.sum(model.body_mass))

    def key_callback(self, key: int) -> None:
        self._handle_toggle_key(key)

    def glfw_key_callback(self, window, key: int, scancode: int, action: int, mods: int) -> None:
        del window, scancode, mods
        if action in (glfw.PRESS, glfw.REPEAT):
            self.held_keys.add(key)
            if action == glfw.PRESS:
                self._handle_toggle_key(key)
        elif action == glfw.RELEASE:
            self.held_keys.discard(key)

    def _handle_toggle_key(self, key: int) -> None:
        if key == KEY_SPACE:
            self.paused = not self.paused
            return
        if key == glfw.KEY_R:
            self.reset_requested = True
        elif key == glfw.KEY_1:
            self.args.command = "idle"
        elif key == glfw.KEY_2:
            self.args.command = "sweep"
        elif key == glfw.KEY_0:
            self.args.controller = "none"
        elif key == glfw.KEY_P:
            self.args.controller = "pd"

    def manual_accel(self) -> np.ndarray:
        accel = np.zeros(2, dtype=np.float32)
        if glfw.KEY_W in self.held_keys:
            accel += np.array([4.0, 0.0], dtype=np.float32)
        if glfw.KEY_S in self.held_keys:
            accel += np.array([-4.0, 0.0], dtype=np.float32)
        if glfw.KEY_A in self.held_keys:
            accel += np.array([0.0, 4.0], dtype=np.float32)
        if glfw.KEY_D in self.held_keys:
            accel += np.array([0.0, -4.0], dtype=np.float32)
        return accel

    def pendulum_torque(self) -> np.ndarray:
        torque = np.zeros(2, dtype=np.float32)
        if glfw.KEY_J in self.held_keys:
            torque += np.array([0.16, 0.0], dtype=np.float32)
        if glfw.KEY_L in self.held_keys:
            torque += np.array([-0.16, 0.0], dtype=np.float32)
        if glfw.KEY_I in self.held_keys:
            torque += np.array([0.0, 0.16], dtype=np.float32)
        if glfw.KEY_K in self.held_keys:
            torque += np.array([0.0, -0.16], dtype=np.float32)
        return torque

    def command_accel(self, sim_time: float, base_velocity: np.ndarray) -> np.ndarray:
        if self.args.command == "idle":
            target_velocity = np.zeros(2, dtype=np.float32)
        else:
            phase = 2.0 * math.pi * sim_time / self.args.sweep_period
            target_velocity = self.args.sweep_speed * np.array(
                [math.sin(phase), 0.55 * math.sin(0.5 * phase + 0.7)],
                dtype=np.float32,
            )
        return 10.0 * (target_velocity - base_velocity)

    def balance_accel(self, roll: float, pitch: float, roll_rate: float, pitch_rate: float) -> np.ndarray:
        if self.args.controller == "none":
            return np.zeros(2, dtype=np.float32)
        # Positive pitch corresponds to a forward lean; positive roll corresponds
        # to a left/right lean. These signs match the ROS PD fallback defaults.
        return np.array(
            [
                self.args.pd_kp * pitch + self.args.pd_kd * pitch_rate,
                -(self.args.pd_kp * roll + self.args.pd_kd * roll_rate),
            ],
            dtype=np.float32,
        )


def main() -> None:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)

    qpos = {
        name: model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        ]
        for name in ("shell_x", "shell_y", "pitch", "roll")
    }
    qvel = {
        name: model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        ]
        for name in ("shell_x", "shell_y", "pitch", "roll")
    }
    actuator = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("x_force", "y_force")
    }

    controller = ViewerController(args, model)
    sim_dt = float(model.opt.timestep)
    control_steps = max(1, round(args.control_dt / sim_dt))
    step_count = 0

    print(
        "Controls: 1 idle, 2 sweep, 0 no controller, P PD controller, "
        "WASD shell kicks, IJKL pendulum torque kicks, R reset, Space pause."
    )
    print(
        "This is an equivalent holonomic-force model, not a physical three-wheel "
        "kiwi-drive contact simulation."
    )

    if not glfw.init():
        raise RuntimeError("Could not initialize GLFW.")

    window = glfw.create_window(args.width, args.height, "B_Cubed Balance RL Viewer", None, None)
    if window is None:
        glfw.terminate()
        raise RuntimeError("Could not create GLFW window.")

    glfw.make_context_current(window)
    glfw.swap_interval(1)
    glfw.set_key_callback(window, controller.glfw_key_callback)

    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultCamera(cam)
    mujoco.mjv_defaultOption(opt)
    cam.distance = 1.7
    cam.azimuth = 45
    cam.elevation = -25
    cam.orthographic = 1

    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

    elapsed_time = 0.0
    previous_time = time.perf_counter()
    target_fps = 60.0
    frame_dt = 1.0 / target_fps
    previous_render_time = previous_time

    try:
        while not glfw.window_should_close(window):
            current_time = time.perf_counter()
            frame_time = min(0.05, current_time - previous_time)
            previous_time = current_time
            elapsed_time += frame_time

            if controller.reset_requested:
                mujoco.mj_resetData(model, data)
                controller.reset_requested = False

            while elapsed_time >= sim_dt:
                if not controller.paused:
                    if step_count % control_steps == 0:
                        base_velocity = np.array(
                            [data.qvel[qvel["shell_x"]], data.qvel[qvel["shell_y"]]],
                            dtype=np.float32,
                        )
                        roll = float(data.qpos[qpos["roll"]])
                        pitch = float(data.qpos[qpos["pitch"]])
                        roll_rate = float(data.qvel[qvel["roll"]])
                        pitch_rate = float(data.qvel[qvel["pitch"]])

                        accel = (
                            controller.command_accel(data.time, base_velocity)
                            + controller.balance_accel(roll, pitch, roll_rate, pitch_rate)
                            + controller.manual_accel()
                        )
                        accel = clip_norm(accel, args.max_accel)
                        force = np.clip(
                            accel * controller.system_mass,
                            -args.max_force,
                            args.max_force,
                        )
                        data.ctrl[actuator["x_force"]] = float(force[0])
                        data.ctrl[actuator["y_force"]] = float(force[1])

                    torque = controller.pendulum_torque()
                    data.qfrc_applied[:] = 0.0
                    data.qfrc_applied[qvel["roll"]] = float(torque[0])
                    data.qfrc_applied[qvel["pitch"]] = float(torque[1])
                    mujoco.mj_step(model, data)
                    step_count += 1

                elapsed_time -= sim_dt

            if current_time - previous_render_time >= frame_dt:
                previous_render_time = current_time
                cam.lookat[0] = float(data.qpos[qpos["shell_x"]])
                cam.lookat[1] = float(data.qpos[qpos["shell_y"]])
                cam.lookat[2] = 0.15
                width, height = glfw.get_framebuffer_size(window)
                viewport = mujoco.MjrRect(0, 0, width, height)
                mujoco.mjv_updateScene(
                    model,
                    data,
                    opt,
                    None,
                    cam,
                    mujoco.mjtCatBit.mjCAT_ALL.value,
                    scene,
                )
                mujoco.mjr_render(viewport, scene, context)
                glfw.swap_buffers(window)

            glfw.poll_events()
    finally:
        glfw.terminate()


if __name__ == "__main__":
    main()
