"""Export a trained SB3 PPO policy for the ROS controller."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import torch

from balance_rl.envs import BB8BalanceEnv


class NormalizedActor(torch.nn.Module):
    """Deterministic actor with VecNormalize statistics baked in."""

    def __init__(
        self,
        policy: torch.nn.Module,
        obs_mean: np.ndarray,
        obs_var: np.ndarray,
        clip_obs: float,
        epsilon: float,
        action_bias: np.ndarray | None = None,
    ):
        super().__init__()
        self.policy = policy
        self.register_buffer("obs_mean", torch.as_tensor(obs_mean, dtype=torch.float32))
        self.register_buffer("obs_var", torch.as_tensor(obs_var, dtype=torch.float32))
        if action_bias is None:
            action_bias = np.zeros(2, dtype=np.float32)
        self.register_buffer(
            "action_bias", torch.as_tensor(action_bias, dtype=torch.float32)
        )
        self.clip_obs = float(clip_obs)
        self.epsilon = float(epsilon)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        norm_obs = (obs - self.obs_mean) / torch.sqrt(self.obs_var + self.epsilon)
        norm_obs = torch.clamp(norm_obs, -self.clip_obs, self.clip_obs)

        try:
            features = self.policy.extract_features(
                norm_obs, self.policy.pi_features_extractor
            )
        except TypeError:
            features = self.policy.extract_features(norm_obs)
        if isinstance(features, tuple):
            features = features[0]
        latent_pi = self.policy.mlp_extractor.forward_actor(features)
        action = self.policy.action_net(latent_pi) - self.action_bias
        return torch.clamp(action, -1.0, 1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--vec-normalize", type=Path, default=None)
    parser.add_argument("--onnx-out", type=Path, default=None)
    parser.add_argument("--npz-out", type=Path, default=None)
    parser.add_argument(
        "--zero-idle-action",
        action="store_true",
        help="Subtract the actor output for an all-zero observation during export.",
    )
    return parser.parse_args()


def export_npz(
    policy: torch.nn.Module,
    obs_mean: np.ndarray,
    obs_var: np.ndarray,
    clip_obs: float,
    epsilon: float,
    output_path: Path,
    action_bias: np.ndarray | None = None,
) -> None:
    """Export a Stable-Baselines3 MLP actor as plain NumPy arrays."""
    modules = list(policy.mlp_extractor.policy_net) + [policy.action_net]
    weights = {}
    activations: list[str] = []
    layer_index = 0

    for module in modules:
        if isinstance(module, torch.nn.Linear):
            weights[f"w_{layer_index}"] = module.weight.detach().cpu().numpy()
            weights[f"b_{layer_index}"] = module.bias.detach().cpu().numpy()
            layer_index += 1
        elif isinstance(module, torch.nn.Tanh):
            activations.append("tanh")
        elif isinstance(module, torch.nn.ReLU):
            activations.append("relu")
        elif isinstance(module, torch.nn.Identity):
            continue
        else:
            raise TypeError(f"Unsupported policy module for NPZ export: {module}")

    if action_bias is not None:
        weights[f"b_{layer_index - 1}"] = (
            weights[f"b_{layer_index - 1}"] - action_bias.astype(np.float32)
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        obs_mean=obs_mean.astype(np.float32),
        obs_var=obs_var.astype(np.float32),
        clip_obs=np.array([clip_obs], dtype=np.float32),
        epsilon=np.array([epsilon], dtype=np.float32),
        layer_count=np.array([layer_index], dtype=np.int32),
        activations=np.array(activations, dtype=str),
        **weights,
    )
    print(f"Exported {output_path}")


def export_onnx(
    policy: torch.nn.Module,
    obs_mean: np.ndarray,
    obs_var: np.ndarray,
    clip_obs: float,
    epsilon: float,
    output_path: Path,
    action_bias: np.ndarray | None = None,
) -> None:
    actor = NormalizedActor(policy, obs_mean, obs_var, clip_obs, epsilon, action_bias)
    actor.eval()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    dummy_obs = torch.zeros((1, obs_mean.shape[0]), dtype=torch.float32)
    torch.onnx.export(
        actor,
        dummy_obs,
        str(output_path),
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
        opset_version=17,
    )
    print(f"Exported {output_path}")


def idle_action_bias(
    policy: torch.nn.Module,
    obs_mean: np.ndarray,
    obs_var: np.ndarray,
    clip_obs: float,
    epsilon: float,
) -> np.ndarray:
    """Return the deterministic actor output for a perfect idle observation."""
    actor = NormalizedActor(policy, obs_mean, obs_var, clip_obs, epsilon)
    actor.eval()
    idle_obs = torch.zeros((1, obs_mean.shape[0]), dtype=torch.float32)
    with torch.no_grad():
        action = actor(idle_obs)[0]
    return action.detach().cpu().numpy().astype(np.float32)


def main() -> None:
    args = parse_args()
    if args.onnx_out is None and args.npz_out is None:
        raise SystemExit("Provide --npz-out, --onnx-out, or both.")
    cache_parent = args.npz_out or args.onnx_out
    os.environ.setdefault("MPLCONFIGDIR", str(cache_parent.parent / "matplotlib"))

    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    model = PPO.load(args.model, device="cpu")
    obs_dim = int(model.observation_space.shape[0])
    obs_mean = np.zeros(obs_dim, dtype=np.float32)
    obs_var = np.ones(obs_dim, dtype=np.float32)
    clip_obs = 10.0
    epsilon = 1e-8

    if args.vec_normalize is not None:
        dummy_env = DummyVecEnv([lambda: BB8BalanceEnv()])
        vec_norm = VecNormalize.load(str(args.vec_normalize), dummy_env)
        obs_mean = vec_norm.obs_rms.mean.astype(np.float32)
        obs_var = vec_norm.obs_rms.var.astype(np.float32)
        clip_obs = float(vec_norm.clip_obs)
        epsilon = float(vec_norm.epsilon)

    action_bias = None
    if args.zero_idle_action:
        action_bias = idle_action_bias(model.policy, obs_mean, obs_var, clip_obs, epsilon)
        print(f"Subtracting idle action bias {action_bias.tolist()}")

    if args.npz_out is not None:
        export_npz(
            model.policy,
            obs_mean,
            obs_var,
            clip_obs,
            epsilon,
            args.npz_out,
            action_bias,
        )
    if args.onnx_out is not None:
        export_onnx(
            model.policy,
            obs_mean,
            obs_var,
            clip_obs,
            epsilon,
            args.onnx_out,
            action_bias,
        )


if __name__ == "__main__":
    main()
