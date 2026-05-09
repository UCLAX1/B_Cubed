# B_Cubed RL Balancing

This branch contains a first pass at the balance-control stack for the BB-8-style robot.

The intended architecture is:

1. Nav2 produces a holonomic velocity command.
2. `bb8_balance_controller` reads that command plus IMU/tilt feedback.
3. A small RL policy, or a tunable PD fallback, produces a bounded residual correction.
4. The node publishes a corrected `Twist` for logging and a `Float32MultiArray` `jet_cmd` compatible with the existing low-level kiwi-drive runner.

The RL package intentionally trains a residual stabilizer instead of replacing Nav2. That keeps navigation, safety limits, and balance authority separated.

## Layout

- `balance_rl/`: MuJoCo/Gymnasium training environment and training/export scripts.
- `ros2_ws/src/bb8_balance_controller/`: ROS 2 node for on-robot inference.
- `docs/rl_balance_plan.md`: system design, training plan, sim-to-real notes, and bring-up checklist.

## Training Machine Setup

```bash
python3 -m venv .venv-balance
source .venv-balance/bin/activate
pip install -r requirements-train.txt
python -m balance_rl.scripts.train_ppo --timesteps 2000000 --num-envs 8
python -m balance_rl.scripts.export_policy \
  --model runs/bb8_balance_ppo/final_model.zip \
  --vec-normalize runs/bb8_balance_ppo/vec_normalize.pkl \
  --npz-out policies/bb8_balance.npz
```

The included environment is deliberately abstract: it models the internal chassis as a damped 2-axis pendulum attached to a moving spherical body. It is fast and domain-randomized, which is a better starting point for residual balance than the older direct-freejoint velocity sims.

On the Jetson, keep smoke tests small because desktop apps and IDEs consume most of the RAM:

```bash
python -m balance_rl.scripts.train_ppo \
  --timesteps 64 \
  --num-envs 1 \
  --n-steps 32 \
  --batch-size 32 \
  --n-epochs 1 \
  --device cpu \
  --torch-threads 1 \
  --run-dir /tmp/bb8_balance_ppo_smoke

python -m balance_rl.scripts.export_policy \
  --model /tmp/bb8_balance_ppo_smoke/final_model.zip \
  --vec-normalize /tmp/bb8_balance_ppo_smoke/vec_normalize.pkl \
  --npz-out /tmp/bb8_balance_ppo_smoke/policy.npz
```

TensorBoard logging is opt-in with `--tensorboard`.

Quick simulation checks:

```bash
python - <<'PY'
from gymnasium.utils.env_checker import check_env
from balance_rl.envs import BB8BalanceEnv
check_env(BB8BalanceEnv(), skip_render_check=True)
print("gym check ok")
PY

WARP_CACHE_PATH=/tmp/warp_cache python - <<'PY'
import mujoco
import mujoco_warp as mjw
m = mujoco.MjModel.from_xml_path("balance_rl/assets/bb8_balance_abstract.xml")
wm = mjw.put_model(m)
wd = mjw.make_data(m, nworld=4)
mjw.step(wm, wd)
print("mjwarp qpos shape", wd.qpos.shape)
PY
```

`WARP_CACHE_PATH` is optional on a normal writable home directory, but it avoids Warp trying to create a cache under a read-only home path.

To look at the model interactively:

```bash
python -m balance_rl.scripts.view_balance_sim
```

The viewer controls are printed in the terminal and handled by a custom GLFW key callback. The visible model is an abstract balance model: the transparent sphere is the shell, the dark lower block is the bottom-heavy internal chassis, and the two slide-joint actuators represent the net x/y force that the real kiwi drive can apply to the shell.

## Jetson/ROS 2 Setup

Install the ROS package in the existing workspace:

```bash
cd ros2_ws
colcon build --packages-select bb8_balance_controller
source install/setup.bash
ros2 launch bb8_balance_controller bb8_balance_controller.launch.py
```

The node can run without a trained policy. If `policy_path` is empty or the configured policy cannot be loaded, it uses the PD fallback configured in `config/bb8_balance_controller.yaml`. Prefer the `.npz` export on the Jetson because it only requires NumPy; ONNX is still supported for teams that already have ONNX Runtime installed.
