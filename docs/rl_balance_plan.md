# RL Balance Plan

## Control Strategy

Train a residual balance controller, not a replacement for Nav2.

Nav2 should continue to output desired body-frame velocity. The balance policy receives IMU tilt/rate, estimated body velocity, the current Nav2 command, command acceleration, and the previous action. It outputs a bounded 2D corrective acceleration. The on-robot ROS node integrates that into a small velocity residual, clamps it, and sends the combined command to the existing kiwi-drive low-level runner.

This keeps the learned policy inside a narrow authority envelope:

- Nav2 owns path following and obstacle behavior.
- The RL policy owns fast sway damping.
- Hardware safety code owns limits, stale-sensor stops, and emergency shutdown.

## Why This Starts With An Abstract MuJoCo Model

The previous MuJoCo sims moved the body with direct freejoint velocity actuators. That is fine for demos but hides the inertia and delay that matter for balance. The included model, `balance_rl/assets/bb8_balance_abstract.xml`, instead treats the shell/chassis as a horizontally actuated base with a 2-axis damped pendulum. It is not meant to be CAD-accurate. It is meant to train the feedback behavior that should transfer: move the shell under the rocking mass while respecting command tracking and acceleration limits.

The important sim-to-real work is parameter spread, not perfect mesh fidelity:

- pendulum mass and inertia scale
- damping
- actuator strength
- controller lag
- IMU noise
- intermittent sway impulses
- command acceleration profiles
- output acceleration and speed clamps

## Training Workflow

1. Collect short robot logs while manually commanding low speeds:
   - body-frame velocity command
   - IMU roll/pitch/rates
   - odometry if available
   - motor command output
2. Fit rough ranges for sway natural frequency, damping, and response delay.
3. Update the environment randomization ranges.
4. Train PPO offboard with many vectorized environments.
5. Export a NumPy `.npz` actor, or ONNX if that runtime is already installed, and deploy to the Jetson.
6. Start with low `max_balance_accel_m_s2` and `max_correction_speed_m_s`.
7. Increase authority only after verifying stale-sensor stop, tilt cutoff, and manual kill.

## MJWarp/MJLab Path

The abstract MJCF uses simple slide joints, hinge joints, and motor actuators so it can be migrated to GPU-batched training. MJWarp is appropriate once the reward and observation contract are stable. The ROS deployment does not depend on MJWarp; it only needs the exported `.npz` or ONNX actor.

On this Jetson sandbox, MJWarp imports and steps the model when `WARP_CACHE_PATH=/tmp/warp_cache` is set, but Warp reports CUDA driver access as unavailable and falls back to CPU. GPU-batched training should be validated on the offboard training machine with CUDA visible to Warp.

## ROS Graph

Recommended on-robot graph:

```text
Nav2 / safety gate -> /cmd_vel
IMU or legacy tilt -> bb8_balance_controller
Odometry optional -> bb8_balance_controller
bb8_balance_controller -> /cmd_vel_balanced
bb8_balance_controller -> /jet_cmd
lowlevelrunner -> CAN motor controllers
```

The current `lowlevelrunner` expects:

```text
[x_velocity, y_velocity, angular_velocity, servo_angle_1, servo_angle_2, servo_angle_3, state]
```

`bb8_balance_controller` publishes that array directly, using neutral servo angles from parameters.

## First Bring-Up Settings

Use PD fallback before running a learned policy:

- `max_balance_accel_m_s2`: `0.6`
- `max_correction_speed_m_s`: `0.15`
- `tilt_cutoff_rad`: `0.55`
- `require_attitude`: `true`
- `zero_on_stale_nav`: `true`

Tune the PD sign first. If a forward pitch makes the correction push the robot farther out of balance, flip `pitch_correction_sign`.
