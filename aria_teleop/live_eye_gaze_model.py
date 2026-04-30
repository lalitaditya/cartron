"""
Live wrapper for the open-source Project Aria eye-tracking model.

The model package is optional because it pulls in PyTorch and the GitHub repo.
This module keeps that dependency lazy and gives the live viewer a small,
single-frame async interface so EyeTrack callbacks do not block the DDS thread.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib.util
import os
from pathlib import Path
import threading
import time
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class LiveEyeGaze:
    timestamp_ns: int
    yaw_rad: float
    pitch_rad: float
    yaw_low_rad: Optional[float] = None
    pitch_low_rad: Optional[float] = None
    yaw_high_rad: Optional[float] = None
    pitch_high_rad: Optional[float] = None
    inference_ms: float = 0.0

    @property
    def vector_cpf(self) -> np.ndarray:
        """Unit gaze vector in Central Pupil Frame: X left, Y up, Z forward."""
        vec = np.array(
            [np.tan(self.yaw_rad), np.tan(self.pitch_rad), 1.0],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(vec)):
            return np.array([0.0, 0.0, 1.0], dtype=np.float64)
        norm = np.linalg.norm(vec)
        return vec / norm if norm > 1e-9 else vec


def _as_pair(values) -> Tuple[float, float]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.size < 2:
        raise ValueError(f"expected at least yaw/pitch values, got shape {np.asarray(values).shape}")
    return float(arr[0]), float(arr[1])


def _candidate_model_paths() -> list[tuple[Path, Path]]:
    spec = importlib.util.find_spec("projectaria_eyetracking")
    if spec is None:
        return []

    package_roots = []
    if spec.origin:
        package_roots.append(Path(spec.origin).resolve().parent)
    if spec.submodule_search_locations:
        package_roots.extend(Path(path).resolve() for path in spec.submodule_search_locations)

    if not package_roots:
        return []

    cwd = Path.cwd()
    package_roots.extend(
        [
            cwd / "projectaria_eyetracking",
            cwd / "vendor" / "projectaria_eyetracking",
            cwd / "third_party" / "projectaria_eyetracking",
        ]
    )

    candidates = []
    for package_root in package_roots:
        repo_root = package_root.parent
        for model_dir in (
            package_root / "inference" / "model" / "pretrained_weights",
            package_root / "model",
            package_root / "models",
            package_root / "pretrained_models",
            package_root / "projectaria_eyetracking" / "inference" / "model" / "pretrained_weights",
            package_root / "projectaria_eyetracking" / "model",
            package_root / "projectaria_eyetracking" / "models",
            repo_root / "model",
            repo_root / "models",
            repo_root / "pretrained_models",
            repo_root / "projectaria_eyetracking" / "inference" / "model" / "pretrained_weights",
            repo_root / "projectaria_eyetracking" / "model",
            repo_root / "projectaria_eyetracking" / "models",
        ):
            if not model_dir.is_dir():
                continue
            for config in model_dir.rglob("*.yaml"):
                for weight_ext in ("*.pt", "*.pth", "*.ckpt"):
                    for weights in model_dir.rglob(weight_ext):
                        candidates.append((weights, config))
    return candidates


def find_default_model_files() -> tuple[Optional[str], Optional[str]]:
    candidates = _candidate_model_paths()
    if not candidates:
        return None, None

    def score(pair: tuple[Path, Path]) -> tuple[int, int]:
        weights, config = pair
        joined = f"{weights} {config}".lower()
        return (
            int("uncertainty" in joined) + int("social" in joined) + int("gaze" in joined),
            -len(str(weights)),
        )

    weights, config = max(candidates, key=score)
    return str(weights), str(config)


class OpenSourceEyeGazeEstimator:
    def __init__(
        self,
        model_path: Optional[str] = None,
        model_config: Optional[str] = None,
        device: str = "cpu",
    ):
        try:
            import torch
            from easydict import EasyDict
            from projectaria_eyetracking.inference.infer import EyeGazeInference
        except ImportError as exc:
            raise RuntimeError(
                "Open-source Project Aria eye tracking is not installed. "
                "Install it with: pip install git+https://github.com/facebookresearch/projectaria_eyetracking.git"
            ) from exc

        if hasattr(torch.serialization, "add_safe_globals"):
            torch.serialization.add_safe_globals([EasyDict])

        if not model_path or not model_config:
            default_model, default_config = find_default_model_files()
            model_path = model_path or default_model
            model_config = model_config or default_config

        if not model_path or not model_config:
            raise RuntimeError(
                "Could not find the Project Aria eye-tracking model files. "
                "Pass --gaze-model-path and --gaze-model-config explicitly."
            )

        if not os.path.isfile(model_path):
            raise RuntimeError(f"Eye-gaze model path does not exist: {model_path}")
        if not os.path.isfile(model_config):
            raise RuntimeError(f"Eye-gaze model config does not exist: {model_config}")

        self._torch = torch
        self.device = device
        self.model_path = model_path
        self.model_config = model_config
        original_torch_load = torch.load

        def torch_load_compat(*args, **kwargs):
            kwargs.setdefault("weights_only", False)
            return original_torch_load(*args, **kwargs)

        torch.load = torch_load_compat
        try:
            self._inference = EyeGazeInference(model_path, model_config, device)
        finally:
            torch.load = original_torch_load

    def predict(self, image: np.ndarray, timestamp_ns: int) -> LiveEyeGaze:
        start = time.perf_counter()
        tensor = self._torch.as_tensor(np.ascontiguousarray(image), device=self.device)
        with self._torch.no_grad():
            preds, lower, upper = self._inference.predict(tensor)

        yaw, pitch = _as_pair(preds)
        yaw_low, pitch_low = _as_pair(lower)
        yaw_high, pitch_high = _as_pair(upper)
        return LiveEyeGaze(
            timestamp_ns=timestamp_ns,
            yaw_rad=yaw,
            pitch_rad=pitch,
            yaw_low_rad=yaw_low,
            pitch_low_rad=pitch_low,
            yaw_high_rad=yaw_high,
            pitch_high_rad=pitch_high,
            inference_ms=(time.perf_counter() - start) * 1000.0,
        )


class AsyncEyeGazeRunner:
    """Single-slot async model runner; newest EyeTrack frame wins."""

    def __init__(self, estimator: OpenSourceEyeGazeEstimator):
        self._estimator = estimator
        self._condition = threading.Condition()
        self._pending: Optional[tuple[np.ndarray, int]] = None
        self._latest: Optional[LiveEyeGaze] = None
        self._last_error: Optional[str] = None
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="aria-eye-gaze", daemon=True)
        self._thread.start()

    @property
    def model_description(self) -> str:
        return f"{self._estimator.model_path} ({self._estimator.device})"

    def submit(self, image: np.ndarray, timestamp_ns: int) -> None:
        with self._condition:
            if self._closed:
                return
            self._pending = (image.copy(), timestamp_ns)
            self._condition.notify()

    def latest(self) -> tuple[Optional[LiveEyeGaze], Optional[str]]:
        with self._condition:
            return self._latest, self._last_error

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                image, timestamp_ns = self._pending
                self._pending = None

            try:
                result = self._estimator.predict(image, timestamp_ns)
                with self._condition:
                    self._latest = result
                    self._last_error = None
            except Exception as exc:
                with self._condition:
                    self._last_error = str(exc)
