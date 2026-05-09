"""MuJoCo/Gymnasium environment for residual BB-8 balance control.

The model is intentionally abstract. It treats the shell/chassis pair as a
2-axis damped pendulum mounted to a horizontally actuated base. The policy does
not learn navigation; it learns a bounded acceleration residual that damps sway
while a conventional velocity tracker follows Nav2 commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from balance_rl.utils.math_utils import clip_norm

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover - import-time guidance
    raise ImportError(
        "BB8BalanceEnv requires gymnasium. Install requirements-train.txt."
    ) from exc

try:
    import mujoco
except ImportError as exc:  # pragma: no cover - import-time guidance
    raise ImportError(
        "BB8BalanceEnv requires mujoco. Install requirements-train.txt."
    ) from exc


@dataclass(frozen=True)
class BalanceEnvConfig:
    """Configuration for the abstract balance environment."""

    model_path: str | None = None
    control_dt: float = 0.02
    episode_seconds: float = 10.0
    max_initial_tilt_rad: float = 0.16
    max_initial_rate_rad_s: float = 0.35
    terminate_tilt_rad: float = 0.75
    max_command_speed_m_s: float = 0.8
    max_command_accel_m_s2: float = 1.8
    command_interval_min_s: float = 0.45
    command_interval_max_s: float = 1.4
    velocity_tracker_kp: float = 16.0
    max_balance_accel_m_s2: float = 6.0
    max_total_accel_m_s2: float = 22.0
    obs_noise_std: float = 0.01
    disturbance_probability: float = 0.025
    max_disturbance_torque_nm: float = 0.18
    action_lag_alpha_min: float = 0.55
    action_lag_alpha_max: float = 0.95
    actuator_scale_min: float = 0.75
    actuator_scale_max: float = 1.25
    mass_scale_min: float = 0.7
    mass_scale_max: float = 1.35
    damping_scale_min: float = 0.55
    damping_scale_max: float = 1.8


class BB8BalanceEnv(gym.Env):
    """Train a residual acceleration policy for rocking suppression."""

    metadata = {"render_modes": []}

    def __init__(self, config: BalanceEnvConfig | None = None):
        super().__init__()
        self.config = config or BalanceEnvConfig()
        model_path = self.config.model_path
        if model_path is None:
            model_path = str(
                Path(__file__).resolve().parents[1]
                / "assets"
                / "bb8_balance_abstract.xml"
            )

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.frame_skip = max(1, round(self.config.control_dt / self.model.opt.timestep))
        self.dt = self.frame_skip * float(self.model.opt.timestep)
        self.episode_steps = int(self.config.episode_seconds / self.dt)

        self._joint_qpos = {
            name: self.model.jnt_qposadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in ("shell_x", "shell_y", "pitch", "roll")
        }
        self._joint_qvel = {
            name: self.model.jnt_dofadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in ("shell_x", "shell_y", "pitch", "roll")
        }
        self._actuator_id = {
            name: mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            for name in ("x_force", "y_force")
        }
        self._chassis_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis_mass"
        )
        self._pendulum_dofs = np.array(
            [self._joint_qvel["roll"], self._joint_qvel["pitch"]], dtype=np.int32
        )

        self._base_body_mass = self.model.body_mass.copy()
        self._base_dof_damping = self.model.dof_damping.copy()
        self._nominal_system_mass = float(np.sum(self.model.body_mass))

        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        obs_high = np.array(
            [
                np.pi,
                np.pi,
                20.0,
                20.0,
                5.0,
                5.0,
                2.0,
                2.0,
                10.0,
                10.0,
                1.0,
                1.0,
            ],
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(
            low=-obs_high,
            high=obs_high,
            dtype=np.float32,
        )

        self.step_count = 0
        self.command_velocity = np.zeros(2, dtype=np.float32)
        self.command_target = np.zeros(2, dtype=np.float32)
        self.command_accel = np.zeros(2, dtype=np.float32)
        self.previous_action = np.zeros(2, dtype=np.float32)
        self.filtered_action = np.zeros(2, dtype=np.float32)
        self.next_command_step = 0
        self.action_lag_alpha = 0.8
        self.actuator_scale = 1.0

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        del options

        self._randomize_model()
        mujoco.mj_resetData(self.model, self.data)

        self.data.qpos[self._joint_qpos["roll"]] = self.np_random.uniform(
            -self.config.max_initial_tilt_rad, self.config.max_initial_tilt_rad
        )
        self.data.qpos[self._joint_qpos["pitch"]] = self.np_random.uniform(
            -self.config.max_initial_tilt_rad, self.config.max_initial_tilt_rad
        )
        self.data.qvel[self._joint_qvel["roll"]] = self.np_random.uniform(
            -self.config.max_initial_rate_rad_s, self.config.max_initial_rate_rad_s
        )
        self.data.qvel[self._joint_qvel["pitch"]] = self.np_random.uniform(
            -self.config.max_initial_rate_rad_s, self.config.max_initial_rate_rad_s
        )

        self.step_count = 0
        self.command_velocity[:] = 0.0
        self.command_target = self._sample_command()
        self.command_accel[:] = 0.0
        self.previous_action[:] = 0.0
        self.filtered_action[:] = 0.0
        self.next_command_step = self._sample_next_command_step()

        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        raw_action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        old_action = self.previous_action.copy()
        self.filtered_action = (
            self.action_lag_alpha * self.filtered_action
            + (1.0 - self.action_lag_alpha) * raw_action
        )

        self._update_command()
        base_velocity = self._base_velocity()
        tracker_accel = (
            self.config.velocity_tracker_kp * (self.command_velocity - base_velocity)
        )
        balance_accel = self.filtered_action * self.config.max_balance_accel_m_s2
        total_accel = clip_norm(
            tracker_accel + balance_accel,
            self.config.max_total_accel_m_s2,
        )

        force = total_accel * self._nominal_system_mass * self.actuator_scale
        self.data.ctrl[self._actuator_id["x_force"]] = float(force[0])
        self.data.ctrl[self._actuator_id["y_force"]] = float(force[1])
        self.data.qfrc_applied[:] = 0.0
        if self.np_random.random() < self.config.disturbance_probability:
            disturbance = self.np_random.uniform(
                -self.config.max_disturbance_torque_nm,
                self.config.max_disturbance_torque_nm,
                size=2,
            )
            self.data.qfrc_applied[self._joint_qvel["roll"]] = disturbance[0]
            self.data.qfrc_applied[self._joint_qvel["pitch"]] = disturbance[1]

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        self.previous_action = raw_action
        obs = self._get_obs()
        reward = self._reward(raw_action, old_action)
        tilt_norm = float(np.linalg.norm(self._tilt()))
        terminated = tilt_norm > self.config.terminate_tilt_rad
        truncated = self.step_count >= self.episode_steps
        if terminated:
            reward -= 5.0

        info = {
            "tilt_norm_rad": tilt_norm,
            "command_speed_m_s": float(np.linalg.norm(self.command_velocity)),
            "base_speed_m_s": float(np.linalg.norm(self._base_velocity())),
            "raw_action_norm": float(np.linalg.norm(raw_action)),
        }
        return obs, float(reward), terminated, truncated, info

    def _randomize_model(self) -> None:
        self.model.body_mass[:] = self._base_body_mass
        self.model.dof_damping[:] = self._base_dof_damping

        mass_scale = self.np_random.uniform(
            self.config.mass_scale_min, self.config.mass_scale_max
        )
        self.model.body_mass[self._chassis_body_id] *= mass_scale

        damping_scale = self.np_random.uniform(
            self.config.damping_scale_min, self.config.damping_scale_max
        )
        self.model.dof_damping[self._pendulum_dofs] *= damping_scale
        mujoco.mj_setConst(self.model, self.data)
        self._nominal_system_mass = float(np.sum(self.model.body_mass))
        self.actuator_scale = float(
            self.np_random.uniform(
                self.config.actuator_scale_min,
                self.config.actuator_scale_max,
            )
        )
        self.action_lag_alpha = float(
            self.np_random.uniform(
                self.config.action_lag_alpha_min,
                self.config.action_lag_alpha_max,
            )
        )

    def _update_command(self) -> None:
        if self.step_count >= self.next_command_step:
            self.command_target = self._sample_command()
            self.next_command_step = self.step_count + self._sample_next_command_step()

        previous = self.command_velocity.copy()
        delta = self.command_target - self.command_velocity
        max_delta = self.config.max_command_accel_m_s2 * self.dt
        self.command_velocity += clip_norm(delta, max_delta)
        self.command_accel = (self.command_velocity - previous) / self.dt

    def _sample_command(self) -> np.ndarray:
        angle = self.np_random.uniform(-np.pi, np.pi)
        speed = self.np_random.uniform(0.0, self.config.max_command_speed_m_s)
        return np.array([np.cos(angle) * speed, np.sin(angle) * speed], dtype=np.float32)

    def _sample_next_command_step(self) -> int:
        interval = self.np_random.uniform(
            self.config.command_interval_min_s,
            self.config.command_interval_max_s,
        )
        return max(1, int(interval / self.dt))

    def _get_obs(self) -> np.ndarray:
        roll, pitch = self._tilt()
        roll_rate, pitch_rate = self._tilt_rate()
        obs = np.array(
            [
                roll,
                pitch,
                roll_rate,
                pitch_rate,
                *self._base_velocity(),
                *self.command_velocity,
                *self.command_accel,
                *self.previous_action,
            ],
            dtype=np.float32,
        )
        if self.config.obs_noise_std > 0.0:
            noise = self.np_random.normal(
                0.0, self.config.obs_noise_std, size=obs.shape
            ).astype(np.float32)
            noise[10:] = 0.0
            obs += noise
        return obs

    def _reward(self, action: np.ndarray, old_action: np.ndarray) -> float:
        tilt = self._tilt()
        rate = self._tilt_rate()
        velocity_error = self.command_velocity - self._base_velocity()
        action_delta = action - old_action
        return (
            1.0
            - 8.0 * float(np.dot(tilt, tilt))
            - 0.18 * float(np.dot(rate, rate))
            - 0.35 * float(np.dot(velocity_error, velocity_error))
            - 0.035 * float(np.dot(action, action))
            - 0.025 * float(np.dot(action_delta, action_delta))
        )

    def _tilt(self) -> np.ndarray:
        return np.array(
            [
                self.data.qpos[self._joint_qpos["roll"]],
                self.data.qpos[self._joint_qpos["pitch"]],
            ],
            dtype=np.float32,
        )

    def _tilt_rate(self) -> np.ndarray:
        return np.array(
            [
                self.data.qvel[self._joint_qvel["roll"]],
                self.data.qvel[self._joint_qvel["pitch"]],
            ],
            dtype=np.float32,
        )

    def _base_velocity(self) -> np.ndarray:
        return np.array(
            [
                self.data.qvel[self._joint_qvel["shell_x"]],
                self.data.qvel[self._joint_qvel["shell_y"]],
            ],
            dtype=np.float32,
        )
