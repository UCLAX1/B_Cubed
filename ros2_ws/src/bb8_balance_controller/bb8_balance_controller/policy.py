"""Policy backends for the ROS balance controller."""

from __future__ import annotations

from pathlib import Path

import numpy as np


class ONNXPolicy:
    """Run an exported deterministic actor through ONNX Runtime."""

    def __init__(self, model_path: str, prefer_cuda: bool = False):
        import onnxruntime as ort

        providers = ["CPUExecutionProvider"]
        if prefer_cuda:
            available = set(ort.get_available_providers())
            if "CUDAExecutionProvider" in available:
                providers.insert(0, "CUDAExecutionProvider")
        self.session = ort.InferenceSession(str(model_path), providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        model_input = obs.astype(np.float32, copy=False)[None, :]
        action = self.session.run(
            [self.output_name],
            {self.input_name: model_input},
        )[0][0]
        return np.clip(action.astype(np.float32), -1.0, 1.0)


class NumpyMLPPolicy:
    """Run an exported SB3 actor with only NumPy."""

    def __init__(self, model_path: str):
        data = np.load(model_path)
        self.obs_mean = data["obs_mean"].astype(np.float32)
        self.obs_var = data["obs_var"].astype(np.float32)
        self.clip_obs = float(data["clip_obs"][0])
        self.epsilon = float(data["epsilon"][0])
        layer_count = int(data["layer_count"][0])
        self.weights = [data[f"w_{index}"].astype(np.float32) for index in range(layer_count)]
        self.biases = [data[f"b_{index}"].astype(np.float32) for index in range(layer_count)]
        self.activations = [str(value) for value in data["activations"]]

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        x = obs.astype(np.float32, copy=False)
        x = (x - self.obs_mean) / np.sqrt(self.obs_var + self.epsilon)
        x = np.clip(x, -self.clip_obs, self.clip_obs)
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases)):
            x = weight @ x + bias
            if index < len(self.activations):
                activation = self.activations[index]
                if activation == "tanh":
                    x = np.tanh(x)
                elif activation == "relu":
                    x = np.maximum(x, 0.0)
        return np.clip(x.astype(np.float32), -1.0, 1.0)


class PDPolicy:
    """Fallback controller used before a trained policy is available."""

    def __init__(
        self,
        kp: float,
        kd: float,
        max_balance_accel: float,
        roll_correction_sign: float,
        pitch_correction_sign: float,
    ):
        self.kp = float(kp)
        self.kd = float(kd)
        self.max_balance_accel = max(float(max_balance_accel), 1e-6)
        self.roll_correction_sign = float(roll_correction_sign)
        self.pitch_correction_sign = float(pitch_correction_sign)

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        roll = float(obs[0])
        pitch = float(obs[1])
        roll_rate = float(obs[2])
        pitch_rate = float(obs[3])
        accel_x = self.pitch_correction_sign * (
            self.kp * pitch + self.kd * pitch_rate
        )
        accel_y = self.roll_correction_sign * (
            self.kp * roll + self.kd * roll_rate
        )
        action = np.array([accel_x, accel_y], dtype=np.float32) / self.max_balance_accel
        return np.clip(action, -1.0, 1.0)


def load_policy_or_pd(
    model_path: str,
    prefer_cuda: bool,
    pd_kp: float,
    pd_kd: float,
    max_balance_accel: float,
    roll_correction_sign: float,
    pitch_correction_sign: float,
):
    """Load ONNX when configured, otherwise return the PD fallback."""
    if model_path:
        path = Path(model_path).expanduser()
        if path.exists():
            try:
                if path.suffix.lower() == ".npz":
                    return NumpyMLPPolicy(str(path)), "npz"
                if path.suffix.lower() == ".onnx":
                    return ONNXPolicy(str(path), prefer_cuda=prefer_cuda), "onnx"
                print(f"Unsupported policy extension for {path}; expected .npz or .onnx")
            except Exception as exc:  # pragma: no cover - runtime logging handles this
                print(f"Failed to load policy at {path}: {exc}")
        else:
            print(f"Configured policy_path does not exist: {path}")

    return (
        PDPolicy(
            pd_kp,
            pd_kd,
            max_balance_accel,
            roll_correction_sign,
            pitch_correction_sign,
        ),
        "pd",
    )
