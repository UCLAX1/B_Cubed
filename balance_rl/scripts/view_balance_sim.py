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
        choices=("idle", "hold", "sweep"),
        default="idle",
        help="Drive command shown in the viewer.",
    )
    parser.add_argument("--control-dt", type=float, default=0.02)
    parser.add_argument("--max-accel", type=float, default=14.0)
    parser.add_argument("--max-force", type=float, default=600.0)
    parser.add_argument(
        "--manual-accel",
        type=float,
        default=12.0,
        help="Extra shell acceleration from WASD keys in m/s^2.",
    )
    parser.add_argument(
        "--manual-torque",
        type=float,
        default=0.8,
        help="Extra roll/pitch generalized torque from IJKL keys.",
    )
    parser.add_argument(
        "--shell-kick-speed",
        type=float,
        default=0.35,
        help="Instant shell velocity kick from each WASD key press in m/s.",
    )
    parser.add_argument(
        "--shell-kick-distance",
        type=float,
        default=0.04,
        help="Instant shell position kick from each WASD key press in meters.",
    )
    parser.add_argument(
        "--tilt-kick-rate",
        type=float,
        default=0.8,
        help="Instant roll/pitch rate kick from each IJKL key press in rad/s.",
    )
    parser.add_argument(
        "--tilt-kick-angle",
        type=float,
        default=0.08,
        help="Instant roll/pitch angle kick from each IJKL key press in radians.",
    )
    parser.add_argument(
        "--follow-camera",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Keep the camera centered on the shell instead of fixed on the world.",
    )
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
        self.shell_position_kick = np.zeros(2, dtype=np.float32)
        self.shell_velocity_kick = np.zeros(2, dtype=np.float32)
        self.tilt_angle_kick = np.zeros(2, dtype=np.float32)
        self.tilt_rate_kick = np.zeros(2, dtype=np.float32)
        self.follow_camera = bool(args.follow_camera)

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
            print(f"paused={self.paused}")
            return
        if key == glfw.KEY_R:
            self.reset_requested = True
            print("reset requested")
        elif key == glfw.KEY_1:
            self.args.command = "idle"
            print("command=idle/no drive force")
        elif key == glfw.KEY_2:
            self.args.command = "sweep"
            print("command=sweep")
        elif key == glfw.KEY_H:
            self.args.command = "hold"
            print("command=hold/zero velocity tracking")
        elif key == glfw.KEY_0:
            self.args.controller = "none"
            print("controller=none")
        elif key == glfw.KEY_P:
            self.args.controller = "pd"
            print("controller=pd")
        elif key == glfw.KEY_F:
            self.follow_camera = not self.follow_camera
            print(f"follow_camera={self.follow_camera}")
        elif key in (glfw.KEY_W, glfw.KEY_A, glfw.KEY_S, glfw.KEY_D):
            key_vector = self._shell_key_vector(key)
            self.shell_position_kick += key_vector * self.args.shell_kick_distance
            self.shell_velocity_kick += key_vector * self.args.shell_kick_speed
            print(
                "shell kick "
                f"pos={np.round(self.shell_position_kick, 3).tolist()} m, "
                f"vel={np.round(self.shell_velocity_kick, 3).tolist()} m/s"
            )
        elif key in (glfw.KEY_I, glfw.KEY_J, glfw.KEY_K, glfw.KEY_L):
            key_vector = self._tilt_key_vector(key)
            self.tilt_angle_kick += key_vector * self.args.tilt_kick_angle
            self.tilt_rate_kick += key_vector * self.args.tilt_kick_rate
            print(
                "tilt kick "
                f"angle={np.round(self.tilt_angle_kick, 3).tolist()} rad, "
                f"rate={np.round(self.tilt_rate_kick, 3).tolist()} rad/s"
            )

    @staticmethod
    def _shell_key_vector(key: int) -> np.ndarray:
        if key == glfw.KEY_W:
            return np.array([1.0, 0.0], dtype=np.float32)
        if key == glfw.KEY_S:
            return np.array([-1.0, 0.0], dtype=np.float32)
        if key == glfw.KEY_A:
            return np.array([0.0, 1.0], dtype=np.float32)
        if key == glfw.KEY_D:
            return np.array([0.0, -1.0], dtype=np.float32)
        return np.zeros(2, dtype=np.float32)

    @staticmethod
    def _tilt_key_vector(key: int) -> np.ndarray:
        if key == glfw.KEY_J:
            return np.array([1.0, 0.0], dtype=np.float32)
        if key == glfw.KEY_L:
            return np.array([-1.0, 0.0], dtype=np.float32)
        if key == glfw.KEY_I:
            return np.array([0.0, 1.0], dtype=np.float32)
        if key == glfw.KEY_K:
            return np.array([0.0, -1.0], dtype=np.float32)
        return np.zeros(2, dtype=np.float32)

    def consume_shell_position_kick(self) -> np.ndarray:
        kick = self.shell_position_kick.copy()
        self.shell_position_kick[:] = 0.0
        return kick

    def consume_shell_velocity_kick(self) -> np.ndarray:
        kick = self.shell_velocity_kick.copy()
        self.shell_velocity_kick[:] = 0.0
        return kick

    def consume_tilt_angle_kick(self) -> np.ndarray:
        kick = self.tilt_angle_kick.copy()
        self.tilt_angle_kick[:] = 0.0
        return kick

    def consume_tilt_rate_kick(self) -> np.ndarray:
        kick = self.tilt_rate_kick.copy()
        self.tilt_rate_kick[:] = 0.0
        return kick

    def manual_accel(self) -> np.ndarray:
        accel = np.zeros(2, dtype=np.float32)
        if glfw.KEY_W in self.held_keys:
            accel += np.array([self.args.manual_accel, 0.0], dtype=np.float32)
        if glfw.KEY_S in self.held_keys:
            accel += np.array([-self.args.manual_accel, 0.0], dtype=np.float32)
        if glfw.KEY_A in self.held_keys:
            accel += np.array([0.0, self.args.manual_accel], dtype=np.float32)
        if glfw.KEY_D in self.held_keys:
            accel += np.array([0.0, -self.args.manual_accel], dtype=np.float32)
        return accel

    def pendulum_torque(self) -> np.ndarray:
        torque = np.zeros(2, dtype=np.float32)
        if glfw.KEY_J in self.held_keys:
            torque += np.array([self.args.manual_torque, 0.0], dtype=np.float32)
        if glfw.KEY_L in self.held_keys:
            torque += np.array([-self.args.manual_torque, 0.0], dtype=np.float32)
        if glfw.KEY_I in self.held_keys:
            torque += np.array([0.0, self.args.manual_torque], dtype=np.float32)
        if glfw.KEY_K in self.held_keys:
            torque += np.array([0.0, -self.args.manual_torque], dtype=np.float32)
        return torque

    def command_accel(self, sim_time: float, base_velocity: np.ndarray) -> np.ndarray:
        if self.args.command == "idle":
            return np.zeros(2, dtype=np.float32)
        if self.args.command == "hold":
            target_velocity = np.zeros(2, dtype=np.float32)
        elif self.args.command == "sweep":
            phase = 2.0 * math.pi * sim_time / self.args.sweep_period
            target_velocity = self.args.sweep_speed * np.array(
                [math.sin(phase), 0.55 * math.sin(0.5 * phase + 0.7)],
                dtype=np.float32,
            )
        else:
            target_velocity = np.zeros(2, dtype=np.float32)
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
    shell_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "shell")

    controller = ViewerController(args, model)
    sim_dt = float(model.opt.timestep)
    control_steps = max(1, round(args.control_dt / sim_dt))
    step_count = 0

    print(
        "Controls: 1 idle, 2 sweep, 0 no controller, P PD controller, "
        "WASD shell kicks, IJKL pendulum kicks, H hold, F follow camera, R reset, Space pause."
    )
    print(
        "Defaults: follow_camera=True, gentle kicks. Use --no-follow-camera for a fixed world view."
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
                    shell_position_kick = controller.consume_shell_position_kick()
                    shell_kick = controller.consume_shell_velocity_kick()
                    tilt_angle_kick = controller.consume_tilt_angle_kick()
                    tilt_kick = controller.consume_tilt_rate_kick()
                    if np.any(shell_position_kick):
                        data.qpos[qpos["shell_x"]] += float(shell_position_kick[0])
                        data.qpos[qpos["shell_y"]] += float(shell_position_kick[1])
                    if np.any(shell_kick):
                        data.qvel[qvel["shell_x"]] += float(shell_kick[0])
                        data.qvel[qvel["shell_y"]] += float(shell_kick[1])
                    if np.any(tilt_angle_kick):
                        data.qpos[qpos["roll"]] = float(
                            np.clip(
                                data.qpos[qpos["roll"]] + tilt_angle_kick[0],
                                -1.1,
                                1.1,
                            )
                        )
                        data.qpos[qpos["pitch"]] = float(
                            np.clip(
                                data.qpos[qpos["pitch"]] + tilt_angle_kick[1],
                                -1.1,
                                1.1,
                            )
                        )
                    if np.any(tilt_kick):
                        data.qvel[qvel["roll"]] += float(tilt_kick[0])
                        data.qvel[qvel["pitch"]] += float(tilt_kick[1])
                    if (
                        np.any(shell_position_kick)
                        or np.any(shell_kick)
                        or np.any(tilt_angle_kick)
                        or np.any(tilt_kick)
                    ):
                        mujoco.mj_forward(model, data)

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

                    shell_accel = controller.manual_accel()
                    if np.any(shell_accel):
                        data.qvel[qvel["shell_x"]] += float(shell_accel[0] * sim_dt)
                        data.qvel[qvel["shell_y"]] += float(shell_accel[1] * sim_dt)

                    torque = controller.pendulum_torque()
                    data.qfrc_applied[:] = 0.0
                    data.qfrc_applied[qvel["roll"]] = float(torque[0])
                    data.qfrc_applied[qvel["pitch"]] = float(torque[1])
                    mujoco.mj_step(model, data)
                    step_count += 1

                elapsed_time -= sim_dt

            if current_time - previous_render_time >= frame_dt:
                previous_render_time = current_time
                if controller.follow_camera:
                    cam.lookat[0] = float(data.xpos[shell_body_id, 0])
                    cam.lookat[1] = float(data.xpos[shell_body_id, 1])
                    cam.lookat[2] = float(data.xpos[shell_body_id, 2])
                else:
                    cam.lookat[0] = 0.0
                    cam.lookat[1] = 0.0
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
