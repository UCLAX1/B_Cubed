"""Train the residual balance policy with Stable-Baselines3 PPO."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from balance_rl.envs import BB8BalanceEnv, BalanceEnvConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--num-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--run-dir", type=Path, default=Path("runs/bb8_balance_ppo"))
    parser.add_argument("--subproc", action="store_true")
    parser.add_argument("--n-steps", type=int, default=1024)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--n-epochs", type=int, default=10)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--torch-threads", type=int, default=1)
    parser.add_argument("--progress-bar", action="store_true")
    parser.add_argument("--tensorboard", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.run_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(args.run_dir / "matplotlib"))

    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
    import torch

    torch.set_num_threads(max(1, args.torch_threads))

    rollout_size = args.n_steps * args.num_envs
    if args.batch_size > rollout_size:
        raise ValueError(
            "--batch-size must be <= --n-steps * --num-envs "
            f"({args.batch_size} > {rollout_size})."
        )

    env_config = BalanceEnvConfig()

    def make_env() -> BB8BalanceEnv:
        return BB8BalanceEnv(env_config)

    vec_env_cls = SubprocVecEnv if args.subproc else None
    env = make_vec_env(
        make_env,
        n_envs=args.num_envs,
        seed=args.seed,
        vec_env_cls=vec_env_cls,
    )
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    model = PPO(
        "MlpPolicy",
        env,
        seed=args.seed,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        gamma=0.995,
        gae_lambda=0.95,
        learning_rate=args.learning_rate,
        ent_coef=0.001,
        vf_coef=0.5,
        max_grad_norm=0.7,
        tensorboard_log=str(args.run_dir / "tb") if args.tensorboard else None,
        device=args.device,
        verbose=1,
    )
    model.learn(total_timesteps=args.timesteps, progress_bar=args.progress_bar)
    model.save(args.run_dir / "final_model")
    env.save(str(args.run_dir / "vec_normalize.pkl"))


if __name__ == "__main__":
    main()
