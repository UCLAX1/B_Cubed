import numpy as np
import pytest

from bb8_balance_controller.policy import NumpyMLPPolicy, PDPolicy, load_policy_or_pd


def test_numpy_mlp_policy_runs_exported_arrays(tmp_path):
    policy_path = tmp_path / "policy.npz"
    np.savez(
        policy_path,
        obs_mean=np.zeros(12, dtype=np.float32),
        obs_var=np.ones(12, dtype=np.float32),
        clip_obs=np.array([10.0], dtype=np.float32),
        epsilon=np.array([1e-8], dtype=np.float32),
        layer_count=np.array([2], dtype=np.int32),
        activations=np.array(["tanh"], dtype=str),
        w_0=np.ones((4, 12), dtype=np.float32) * 0.1,
        b_0=np.zeros(4, dtype=np.float32),
        w_1=np.ones((2, 4), dtype=np.float32) * 0.2,
        b_1=np.zeros(2, dtype=np.float32),
    )

    policy = NumpyMLPPolicy(str(policy_path))
    action = policy(np.ones(12, dtype=np.float32))
    assert action.shape == (2,)
    assert np.all(action <= 1.0)
    assert np.all(action >= -1.0)


def test_pd_policy_is_clipped():
    policy = PDPolicy(
        kp=10.0,
        kd=2.0,
        max_balance_accel=1.0,
        roll_correction_sign=-1.0,
        pitch_correction_sign=1.0,
    )
    action = policy(np.array([2.0, 2.0, 0.0, 0.0] + [0.0] * 8, dtype=np.float32))
    assert np.all(action <= 1.0)
    assert np.all(action >= -1.0)


def test_policy_loader_can_refuse_pd_fallback():
    with pytest.raises(RuntimeError):
        load_policy_or_pd(
            "",
            False,
            3.0,
            0.45,
            0.8,
            -1.0,
            1.0,
            allow_pd_fallback=False,
        )
