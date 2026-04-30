"""
Live Aria RGB viewer with an eye-tracking dot overlay.

This uses the working classic Project Aria SDK path (`aria.sdk`), not the
Gen2 HTTP streaming sample. It subscribes to RGB and EyeTrack frames, shows the
RGB camera in a Matplotlib window, and draws a clear gaze-proxy dot estimated
from the pupil locations in the EyeTrack camera.

Usage from WSL:
  cd /mnt/c/Users/aryan/cartron
  source aria_env/bin/activate
  python aria_teleop/aria_eye_viewer.py

Keep `watch_aria_usb.ps1` running in an Administrator PowerShell, or launch
this through `run_aria_eye_viewer.sh` so the USB-NCM interface is configured.
"""

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import threading
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARTRON_ROOT = os.path.dirname(SCRIPT_DIR)
FASTDDS_XML = os.path.join(CARTRON_ROOT, "fastdds_aria_unicast.xml")


def requested_interface() -> str:
    for index, arg in enumerate(sys.argv):
        if arg == "--interface" and index + 1 < len(sys.argv):
            return sys.argv[index + 1]
        if arg.startswith("--interface="):
            return arg.split("=", 1)[1]
    return "usb"


def ensure_fastdds_unicast():
    if requested_interface() == "wifi":
        for env_name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
            env_value = os.environ.get(env_name, "")
            if os.path.basename(env_value).startswith("fastdds_aria_unicast"):
                os.environ.pop(env_name, None)
        print("[eye-viewer] Wi-Fi mode: not using the USB-NCM FastDDS profile.")
        return

    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        print(f"[eye-viewer] Using FastDDS profile from env: {os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']}")
    elif os.path.isfile(FASTDDS_XML):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = FASTDDS_XML
        os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = FASTDDS_XML
        print(f"[eye-viewer] Using FastDDS unicast profile: {FASTDDS_XML}")


ensure_fastdds_unicast()

import cv2
import numpy as np
import matplotlib.pyplot as plt

import aria.sdk as aria

try:
    from live_eye_gaze_model import AsyncEyeGazeRunner, OpenSourceEyeGazeEstimator
except ImportError:
    from aria_teleop.live_eye_gaze_model import AsyncEyeGazeRunner, OpenSourceEyeGazeEstimator

try:
    from projectaria_tools.core import calibration as aria_calibration
    from projectaria_tools.core.mps import EyeGaze
    from projectaria_tools.core.mps.utils import get_gaze_vector_reprojection
except ImportError:
    aria_calibration = None
    EyeGaze = None
    get_gaze_vector_reprojection = None


def ping_command(ip_address: str):
    if sys.platform.startswith("win"):
        return ["ping", "-n", "2", "-w", "2000", ip_address]
    return ["ping", "-c", "2", "-W", "2", ip_address]


class EyeTrackObserver:
    def __init__(self, gaze_runner=None):
        self._lock = threading.Lock()
        self._gaze_runner = gaze_runner
        self.latest_eye_image = None
        self.latest_rgb_image = None
        self.latest_any_image = None
        self.latest_any_camera_id = None
        self.latest_eye_timestamp_ns = 0
        self.latest_rgb_timestamp_ns = 0
        self.eye_frames = 0
        self.rgb_frames = 0
        self.imu_frames = 0
        self.last_eye_frame_time = 0.0
        self.last_rgb_frame_time = 0.0
        self.camera_counts = {}

    def on_image_received(self, image: np.ndarray, record):
        with self._lock:
            self.camera_counts[record.camera_id] = self.camera_counts.get(record.camera_id, 0) + 1
            self.latest_any_camera_id = record.camera_id
            self.latest_any_image = image.copy()

        if record.camera_id == aria.CameraId.Rgb:
            # Match the orientation used by the official SDK sample.
            rgb_image = np.rot90(image, -1)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            with self._lock:
                self.latest_rgb_image = rgb_image.copy()
                self.latest_rgb_timestamp_ns = record.capture_timestamp_ns
                self.rgb_frames += 1
                self.last_rgb_frame_time = time.time()
            return

        if record.camera_id != aria.CameraId.EyeTrack:
            return

        if self._gaze_runner is not None:
            self._gaze_runner.submit(image, record.capture_timestamp_ns)

        # SDK sample rotates EyeTrack by 180 degrees for natural viewing.
        image = np.rot90(image, 2)
        with self._lock:
            self.latest_eye_image = image.copy()
            self.latest_eye_timestamp_ns = record.capture_timestamp_ns
            self.eye_frames += 1
            self.last_eye_frame_time = time.time()

    def on_streaming_client_failure(self, reason, message: str):
        print(f"[eye-viewer] Streaming failure: {reason}: {message}")

    def on_imu_received(self, samples, imu_idx: int):
        with self._lock:
            self.imu_frames += len(samples) if samples is not None else 1

    def get_latest(self):
        with self._lock:
            counts = dict(self.camera_counts)
            any_image = None if self.latest_any_image is None else self.latest_any_image.copy()
            gaze, gaze_error = (None, None)
            if self._gaze_runner is not None:
                gaze, gaze_error = self._gaze_runner.latest()
            return (
                None if self.latest_rgb_image is None else self.latest_rgb_image.copy(),
                None if self.latest_eye_image is None else self.latest_eye_image.copy(),
                any_image,
                self.latest_any_camera_id,
                self.latest_rgb_timestamp_ns,
                self.latest_eye_timestamp_ns,
                self.rgb_frames,
                self.eye_frames,
                self.imu_frames,
                self.last_rgb_frame_time,
                self.last_eye_frame_time,
                counts,
                gaze,
                gaze_error,
            )

    def close(self):
        if self._gaze_runner is not None:
            self._gaze_runner.close()


def detect_dark_dot(gray: np.ndarray, x_offset: int = 0):
    """Return the center of the most pupil-like dark blob in a grayscale ROI."""
    if gray.size == 0:
        return None

    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    cutoff = int(np.percentile(blur, 7))
    cutoff = max(8, min(cutoff, 80))
    mask = cv2.inRange(blur, 0, cutoff)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    h, w = gray.shape[:2]
    best = None
    best_score = -1.0
    center_bias = np.array([w / 2.0, h / 2.0])

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 12 or area > (w * h * 0.20):
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        distance = np.linalg.norm(np.array([cx, cy]) - center_bias)

        # Prefer round-ish, dark blobs near the middle of each eye crop.
        score = area * max(circularity, 0.1) - 0.15 * distance
        if score > best_score:
            best_score = score
            best = (int(cx + x_offset), int(cy), int(area))

    return best


def overlay_eye_dots(image: np.ndarray):
    if image.ndim == 2:
        gray = image
        display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        display = image.copy()
        gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)

    h, w = gray.shape[:2]
    split = w // 2
    rois = [(0, split), (split, w)]
    dots = []

    for x0, x1 in rois:
        dot = detect_dark_dot(gray[:, x0:x1], x_offset=x0)
        if dot is None:
            continue
        x, y, area = dot
        dots.append(dot)
        cv2.circle(display, (x, y), 9, (0, 255, 0), 2)
        cv2.circle(display, (x, y), 3, (0, 0, 255), -1)
        cv2.drawMarker(display, (x, y), (255, 255, 0), cv2.MARKER_CROSS, 18, 1)
        cv2.putText(
            display,
            f"dot area={area}",
            (max(5, x - 55), max(18, y - 14)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    cv2.line(display, (split, 0), (split, h), (80, 80, 80), 1)
    return display, dots


def estimate_rgb_dot_from_eye_dots(rgb_shape, eye_shape, dots):
    """
    Convert pupil positions in the EyeTrack image into an approximate RGB dot.

    This is intentionally a simple live proxy, not a calibrated Project Aria MPS
    eye-gaze solution. It gives a visible dot that moves with the eyes while we
    work on the deeper calibrated gaze projection path.
    """
    if not dots:
        return None

    rgb_h, rgb_w = rgb_shape[:2]
    eye_h, eye_w = eye_shape[:2]
    split = eye_w / 2.0

    normalized = []
    for x, y, _area in dots:
        if x < split:
            cx = split / 2.0
            half_width = split / 2.0
        else:
            cx = split + split / 2.0
            half_width = split / 2.0
        cy = eye_h / 2.0
        nx = (x - cx) / max(half_width, 1.0)
        ny = (y - cy) / max(eye_h / 2.0, 1.0)
        normalized.append((nx, ny))

    nx = float(np.mean([p[0] for p in normalized]))
    ny = float(np.mean([p[1] for p in normalized]))

    # The scale is intentionally large so movement is obvious while testing.
    dot_x = int(np.clip((0.5 + nx * 0.45) * rgb_w, 0, rgb_w - 1))
    dot_y = int(np.clip((0.5 + ny * 0.45) * rgb_h, 0, rgb_h - 1))
    return dot_x, dot_y


class GazeRgbProjector:
    """Project CPF yaw/pitch onto the live RGB image using Aria factory calibration."""

    def __init__(self, device_calibration, rgb_label, rgb_camera_calibration, depth_m: float, make_upright: bool):
        self.device_calibration = device_calibration
        self.rgb_label = rgb_label
        self.rgb_camera_calibration = rgb_camera_calibration
        self.depth_m = depth_m
        self.make_upright = make_upright
        self.last_debug = {}
        self._scaled_calibration_cache = {}

    def _scaled_camera_calibration(self, rgb_shape):
        rgb_h, rgb_w = rgb_shape[:2]
        cache_key = (int(rgb_w), int(rgb_h))
        if cache_key in self._scaled_calibration_cache:
            scaled, debug = self._scaled_calibration_cache[cache_key]
            self.last_debug.update(debug)
            return scaled

        original_size = np.asarray(self.rgb_camera_calibration.get_image_size(), dtype=np.float64).reshape(-1)
        if original_size.size < 2 or not np.all(np.isfinite(original_size[:2])):
            debug = {"calib_size": "unknown", "live_size": (int(rgb_w), int(rgb_h)), "scale": 1.0}
            self.last_debug.update(debug)
            self._scaled_calibration_cache[cache_key] = (self.rgb_camera_calibration, debug)
            return self.rgb_camera_calibration

        original_w, original_h = float(original_size[0]), float(original_size[1])
        if original_w <= 0 or original_h <= 0:
            debug = {"calib_size": "invalid", "live_size": (int(rgb_w), int(rgb_h)), "scale": 1.0}
            self.last_debug.update(debug)
            self._scaled_calibration_cache[cache_key] = (self.rgb_camera_calibration, debug)
            return self.rgb_camera_calibration

        scale_x = rgb_w / original_w
        scale_y = rgb_h / original_h
        if abs(scale_x - 1.0) < 1e-6 and abs(scale_y - 1.0) < 1e-6:
            scaled = self.rgb_camera_calibration
            scale = 1.0
        elif abs(scale_x - scale_y) < 1e-3:
            scale = float((scale_x + scale_y) * 0.5)
            scaled = self.rgb_camera_calibration.rescale(
                np.array([rgb_w, rgb_h], dtype=np.int32),
                scale,
                np.array([0.0, 0.0], dtype=np.float64),
            )
        else:
            # This stream should be a uniform resize of the square RGB image.
            # If that assumption breaks, keep the original calibration and make
            # the mismatch obvious in the projection debug text.
            scale = float((scale_x + scale_y) * 0.5)
            scaled = self.rgb_camera_calibration

        self.last_debug.update(
            {
                "calib_size": (int(round(original_w)), int(round(original_h))),
                "live_size": (int(rgb_w), int(rgb_h)),
                "scale": scale,
            }
        )
        self._scaled_calibration_cache[cache_key] = (scaled, dict(self.last_debug))
        return scaled

    def project(self, rgb_shape, gaze):
        self.last_debug = {}
        if gaze is None or EyeGaze is None or get_gaze_vector_reprojection is None:
            return None
        if not (np.isfinite(gaze.yaw_rad) and np.isfinite(gaze.pitch_rad)):
            return None

        return self.project_yaw_pitch(rgb_shape, gaze.yaw_rad, gaze.pitch_rad, update_debug=True)

    def project_yaw_pitch(self, rgb_shape, yaw_rad, pitch_rad, update_debug=False):
        if EyeGaze is None or get_gaze_vector_reprojection is None:
            return None
        if not (np.isfinite(yaw_rad) and np.isfinite(pitch_rad)):
            return None
        if update_debug:
            self.last_debug = {}

        camera_calibration = self._scaled_camera_calibration(rgb_shape)
        eye_gaze = EyeGaze()
        eye_gaze.yaw = float(yaw_rad)
        eye_gaze.pitch = float(pitch_rad)
        projected = get_gaze_vector_reprojection(
            eye_gaze,
            self.rgb_label,
            self.device_calibration,
            camera_calibration,
            self.depth_m,
            make_upright=self.make_upright,
        )
        self.last_debug.update(
            {
                "mode": "upright" if self.make_upright else "raw",
                "yaw_deg": float(np.degrees(yaw_rad)),
                "pitch_deg": float(np.degrees(pitch_rad)),
            }
        )
        if projected is None:
            self.last_debug["pixel"] = None
            self.last_debug["in_frame"] = False
            return None

        point = np.asarray(projected, dtype=np.float64).reshape(-1)
        if point.size < 2 or not np.all(np.isfinite(point[:2])):
            self.last_debug["pixel"] = None
            self.last_debug["in_frame"] = False
            return None

        rgb_h, rgb_w = rgb_shape[:2]
        x = float(point[0])
        y = float(point[1])
        in_frame = 0.0 <= x < rgb_w and 0.0 <= y < rgb_h
        self.last_debug["pixel"] = (x, y)
        self.last_debug["in_frame"] = in_frame
        if not in_frame:
            return None
        x = int(round(x))
        y = int(round(y))
        return x, y


def _camera_calibration_area(camera_calibration):
    try:
        size = np.asarray(camera_calibration.get_image_size(), dtype=np.float64).reshape(-1)
        if size.size >= 2 and np.all(np.isfinite(size[:2])):
            return float(size[0] * size[1])
    except Exception:
        pass
    return 0.0


def choose_rgb_camera_calibration(device_calibration):
    labels = list(device_calibration.get_camera_labels())
    if not labels:
        raise RuntimeError("device calibration has no camera labels")

    calibrations = []
    for label in labels:
        try:
            calibrations.append((label, device_calibration.get_camera_calib(label)))
        except Exception:
            continue
    if not calibrations:
        raise RuntimeError(f"could not load any camera calibration from labels: {labels}")

    preferred = []
    for label, camera_calibration in calibrations:
        label_text = str(label).lower()
        score = 0
        if "rgb" in label_text:
            score += 100
        if "214" in label_text:
            score += 50
        if "camera" in label_text:
            score += 10
        preferred.append((score, _camera_calibration_area(camera_calibration), label, camera_calibration))

    _score, _area, label, camera_calibration = max(preferred, key=lambda item: (item[0], item[1]))
    return label, camera_calibration, labels


def make_gaze_rgb_projector(device, args):
    if aria_calibration is None or EyeGaze is None or get_gaze_vector_reprojection is None:
        print("[eye-viewer] WARNING: projectaria_tools MPS projection utilities are unavailable; using angle-pad gaze display.")
        return None

    try:
        calibration_json = device.factory_calibration_json
        if not calibration_json:
            raise RuntimeError("device.factory_calibration_json is empty")
        device_calibration = aria_calibration.device_calibration_from_json_string(calibration_json)
        rgb_label, rgb_camera_calibration, labels = choose_rgb_camera_calibration(device_calibration)
    except Exception as exc:
        print(f"[eye-viewer] WARNING: failed to load RGB factory calibration for gaze projection: {exc}")
        return None

    print(
        "[eye-viewer] RGB gaze projection enabled: "
        f"label={rgb_label}, depth={args.gaze_projection_depth_m:.2f}m, "
        f"upright={not args.disable_upright_gaze_projection}, labels={labels}"
    )
    return GazeRgbProjector(
        device_calibration=device_calibration,
        rgb_label=rgb_label,
        rgb_camera_calibration=rgb_camera_calibration,
        depth_m=args.gaze_projection_depth_m,
        make_upright=not args.disable_upright_gaze_projection,
    )


def estimate_rgb_dot_from_gaze(rgb_shape, gaze, max_angle_rad: float, projector=None):
    """
    Project open-model yaw/pitch onto the RGB image.

    The preferred path uses the official Project Aria MPS reprojection helper
    with device factory calibration. If that is unavailable, fall back to a
    simple angle pad so the live display still moves while debugging.
    """
    if gaze is None:
        return None
    if not (np.isfinite(gaze.yaw_rad) and np.isfinite(gaze.pitch_rad)):
        return None

    if projector is not None:
        return projector.project(rgb_shape, gaze)

    rgb_h, rgb_w = rgb_shape[:2]
    nx = np.clip(-gaze.yaw_rad / max(max_angle_rad, 1e-6), -1.0, 1.0)
    ny = np.clip(-gaze.pitch_rad / max(max_angle_rad, 1e-6), -1.0, 1.0)
    dot_x = int(np.clip((0.5 + 0.45 * nx) * rgb_w, 0, rgb_w - 1))
    dot_y = int(np.clip((0.5 + 0.45 * ny) * rgb_h, 0, rgb_h - 1))
    return dot_x, dot_y


def estimate_rgb_dot_from_head(rgb_shape, projector=None):
    """Project the CPF/head-forward ray onto the RGB image."""
    if projector is not None:
        dot = projector.project_yaw_pitch(rgb_shape, 0.0, 0.0, update_debug=False)
        if dot is not None:
            return dot

    rgb_h, rgb_w = rgb_shape[:2]
    return int(round((rgb_w - 1) * 0.5)), int(round((rgb_h - 1) * 0.5))


def format_gaze_text(gaze, gaze_error=None):
    if gaze is None:
        return f"gaze=waiting ({gaze_error})" if gaze_error else "gaze=waiting"
    if not (np.isfinite(gaze.yaw_rad) and np.isfinite(gaze.pitch_rad)):
        return f"gaze=invalid infer={gaze.inference_ms:.0f}ms"

    yaw_deg = np.degrees(gaze.yaw_rad)
    pitch_deg = np.degrees(gaze.pitch_rad)
    vec = gaze.vector_cpf
    return (
        f"gaze yaw={yaw_deg:+.1f}deg pitch={pitch_deg:+.1f}deg "
        f"cpf=[{vec[0]:+.2f},{vec[1]:+.2f},{vec[2]:+.2f}] "
        f"infer={gaze.inference_ms:.0f}ms"
    )


class DotAffineCorrection:
    """2D affine correction from predicted dot pixels to target pixels."""

    def __init__(self, matrix=None, rmse_px=None, sample_count=0):
        self.matrix = None if matrix is None else np.array(matrix, dtype=np.float64).reshape(3, 2)
        self.rmse_px = rmse_px
        self.sample_count = sample_count

    @property
    def available(self):
        return self.matrix is not None

    def apply(self, dot, image_shape=None):
        if dot is None or self.matrix is None:
            return dot

        corrected = np.array([float(dot[0]), float(dot[1]), 1.0]) @ self.matrix
        if image_shape is not None:
            height, width = image_shape[:2]
            corrected[0] = np.clip(corrected[0], 0, width - 1)
            corrected[1] = np.clip(corrected[1], 0, height - 1)
        return tuple(int(round(v)) for v in corrected)

    def to_json(self):
        return {
            "matrix": None if self.matrix is None else self.matrix.tolist(),
            "rmse_px": self.rmse_px,
            "sample_count": self.sample_count,
        }

    @classmethod
    def from_json(cls, payload):
        if not payload or payload.get("matrix") is None:
            return cls()
        return cls(
            matrix=payload["matrix"],
            rmse_px=payload.get("rmse_px"),
            sample_count=payload.get("sample_count", 0),
        )

    @classmethod
    def fit(cls, samples):
        valid = []
        for predicted, target in samples:
            if predicted is None or target is None:
                continue
            px, py = float(predicted[0]), float(predicted[1])
            tx, ty = float(target[0]), float(target[1])
            if all(np.isfinite(v) for v in (px, py, tx, ty)):
                valid.append(((px, py), (tx, ty)))

        if len(valid) < 3:
            return cls(sample_count=len(valid))

        a = np.array([[p[0], p[1], 1.0] for p, _target in valid], dtype=np.float64)
        b = np.array([[target[0], target[1]] for _p, target in valid], dtype=np.float64)
        matrix, _residuals, rank, _singular_values = np.linalg.lstsq(a, b, rcond=None)
        if rank < 3:
            return cls(sample_count=len(valid))

        residuals = a @ matrix - b
        rmse_px = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
        return cls(matrix=matrix, rmse_px=rmse_px, sample_count=len(valid))


class GazeCalibration:
    def __init__(self, model=None, old=None, metadata=None):
        self.model = model or DotAffineCorrection()
        self.old = old or DotAffineCorrection()
        self.metadata = metadata or {}

    @property
    def available(self):
        return self.model.available or self.old.available

    def apply_model(self, dot, image_shape=None):
        return self.model.apply(dot, image_shape=image_shape)

    def apply_old(self, dot, image_shape=None):
        return self.old.apply(dot, image_shape=image_shape)

    def summary(self):
        parts = []
        if self.model.available:
            parts.append(f"model rmse={self.model.rmse_px:.1f}px n={self.model.sample_count}")
        if self.old.available:
            parts.append(f"old rmse={self.old.rmse_px:.1f}px n={self.old.sample_count}")
        return ", ".join(parts) if parts else "none"

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        payload = {
            "version": 1,
            "created_unix_s": time.time(),
            "metadata": self.metadata,
            "model": self.model.to_json(),
            "old": self.old.to_json(),
        }
        with open(path, "w", encoding="utf-8") as file:
            json.dump(payload, file, indent=2)

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as file:
            payload = json.load(file)
        return cls(
            model=DotAffineCorrection.from_json(payload.get("model")),
            old=DotAffineCorrection.from_json(payload.get("old")),
            metadata=payload.get("metadata") or {},
        )

    @classmethod
    def fit(cls, model_samples, old_samples, metadata=None):
        return cls(
            model=DotAffineCorrection.fit(model_samples),
            old=DotAffineCorrection.fit(old_samples),
            metadata=metadata,
        )


def reject_bad_calibration(calibration, args, context):
    max_rmse = getattr(args, "max_calibration_rmse_px", None)
    if max_rmse is None or max_rmse <= 0:
        return calibration

    model = calibration.model
    old = calibration.old
    rejected = []

    if model.available and (model.rmse_px is None or model.rmse_px > max_rmse):
        rejected.append(f"model rmse={model.rmse_px:.1f}px")
        model = DotAffineCorrection()
    if old.available and (old.rmse_px is None or old.rmse_px > max_rmse):
        rejected.append(f"old rmse={old.rmse_px:.1f}px")
        old = DotAffineCorrection()

    if rejected:
        print(
            f"[calibration] {context}: rejecting correction(s) above "
            f"{max_rmse:.1f}px RMSE: {', '.join(rejected)}"
        )

    return GazeCalibration(model=model, old=old, metadata=calibration.metadata)


def load_gaze_calibration(args):
    if args.disable_gaze_calibration:
        return GazeCalibration()
    if not args.calibration_file or not os.path.isfile(args.calibration_file):
        return GazeCalibration()

    try:
        calibration = GazeCalibration.load(args.calibration_file)
    except Exception as exc:
        print(f"[calibration] Failed to load {args.calibration_file}: {exc}")
        return GazeCalibration()

    expected_model_projection = getattr(args, "gaze_projection_mode", None)
    saved_model_projection = calibration.metadata.get("model_projection")
    if (
        args.gaze_source == "open-model"
        and expected_model_projection
        and saved_model_projection != expected_model_projection
    ):
        print(
            f"[calibration] Ignoring {args.calibration_file}: "
            f"saved model_projection={saved_model_projection or 'unknown'}, current={expected_model_projection}"
        )
        return GazeCalibration()

    calibration = reject_bad_calibration(calibration, args, "loaded")
    if not calibration.available:
        return calibration

    print(f"[calibration] Loaded {args.calibration_file}: {calibration.summary()}")
    return calibration


def make_calibration_targets(image_shape, grid_size):
    height, width = image_shape[:2]
    grid_size = max(2, int(grid_size))
    fractions = np.linspace(0.16, 0.84, grid_size)
    targets = []
    for fy in fractions:
        for fx in fractions:
            targets.append((int(round(fx * (width - 1))), int(round(fy * (height - 1)))))
    return targets


def mean_dot(dots):
    if not dots:
        return None
    return np.mean(np.array(dots, dtype=np.float64), axis=0)


def format_dot(dot):
    if dot is None:
        return "none"
    return f"({dot[0]:.1f}, {dot[1]:.1f})"


def format_delta(delta):
    if delta is None:
        return "none"
    return f"dx={delta[0]:+.1f}px dy={delta[1]:+.1f}px"


def format_projection_debug(projector):
    if projector is None or not getattr(projector, "last_debug", None):
        return "projection=none"

    debug = projector.last_debug
    pixel = debug.get("pixel")
    if pixel is None:
        pixel_text = "none"
    else:
        pixel_text = format_dot(pixel)

    return (
        f"projection mode={debug.get('mode', '?')} pixel={pixel_text} "
        f"in_frame={int(bool(debug.get('in_frame', False)))} "
        f"calib_size={debug.get('calib_size', '?')} "
        f"live_size={debug.get('live_size', '?')} "
        f"scale={debug.get('scale', 1.0):.3f}"
    )


def print_target_calibration_debug(label, target_index, target, predicted_points):
    predicted_mean = mean_dot(predicted_points)
    target_vec = np.array(target, dtype=np.float64)
    if predicted_mean is None:
        print(
            f"[calibration-debug] target={target_index:02d} {label}: "
            f"actual={format_dot(target_vec)} predicted=none samples=0"
        )
        return

    adjustment = target_vec - predicted_mean
    error_px = float(np.linalg.norm(adjustment))
    print(
        f"[calibration-debug] target={target_index:02d} {label}: "
        f"actual={format_dot(target_vec)} predicted_mean={format_dot(predicted_mean)} "
        f"adjust_needed={format_delta(adjustment)} error={error_px:.1f}px "
        f"samples={len(predicted_points)}"
    )


def print_fit_residual_debug(label, correction, samples):
    if not samples:
        print(f"[calibration-debug] {label} fit: no samples")
        return
    if not correction.available:
        print(f"[calibration-debug] {label} fit: unavailable, samples={len(samples)}")
        return

    residuals = []
    for predicted, target in samples:
        corrected = correction.apply(predicted)
        if corrected is None:
            continue
        target_vec = np.array(target, dtype=np.float64)
        corrected_vec = np.array(corrected, dtype=np.float64)
        residuals.append(target_vec - corrected_vec)

    if not residuals:
        print(f"[calibration-debug] {label} fit: no valid corrected samples")
        return

    residuals = np.array(residuals, dtype=np.float64)
    error = np.linalg.norm(residuals, axis=1)
    mean_residual = np.mean(residuals, axis=0)
    print(
        f"[calibration-debug] {label} after affine correction: "
        f"mean_residual={format_delta(mean_residual)} "
        f"mean_error={float(np.mean(error)):.1f}px max_error={float(np.max(error)):.1f}px "
        f"rmse={correction.rmse_px:.1f}px samples={correction.sample_count}"
    )


def _write_dot(image_bgr, dot, color, label):
    if dot is None:
        return
    x, y = int(dot[0]), int(dot[1])
    cv2.circle(image_bgr, (x, y), 18, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.circle(image_bgr, (x, y), 13, color, 3, cv2.LINE_AA)
    cv2.circle(image_bgr, (x, y), 4, color, -1, cv2.LINE_AA)
    cv2.putText(
        image_bgr,
        label,
        (max(5, x + 16), max(20, y - 16)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def make_recording_rgb_frame(rgb_image, model_dot, head_dot, gaze, gaze_error):
    frame_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    _write_dot(frame_bgr, model_dot, (255, 0, 255), "eye model")
    _write_dot(frame_bgr, head_dot, (255, 255, 0), "head")
    cv2.putText(
        frame_bgr,
        format_gaze_text(gaze, gaze_error),
        (16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return frame_bgr


class StreamRecorder:
    def __init__(self, output_dir, fps=30.0):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.abspath(os.path.join(output_dir, f"aria_eye_{timestamp}"))
        os.makedirs(self.output_dir, exist_ok=True)
        self.fps = fps
        self.rgb_writer = None
        self.eye_writer = None
        self.last_rgb_ts_ns = None
        self.last_eye_ts_ns = None
        self.csv_path = os.path.join(self.output_dir, "gaze_stream.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "wall_time_s",
                "rgb_timestamp_ns",
                "eye_timestamp_ns",
                "rgb_frames",
                "eye_frames",
                "model_dot_x",
                "model_dot_y",
                "head_dot_x",
                "head_dot_y",
                "yaw_rad",
                "pitch_rad",
                "yaw_deg",
                "pitch_deg",
                "cpf_x_left",
                "cpf_y_up",
                "cpf_z_forward",
                "gaze_inference_ms",
                "gaze_error",
            ]
        )

        # Per-written-RGB-frame manifest. One row per frame actually written to
        # rgb_overlay.avi, in the same order, so downstream synchronization can
        # map a video frame index to an exact wall time and device timestamp.
        self.rgb_video_frame_index = -1
        self.rgb_manifest_path = os.path.join(self.output_dir, "rgb_overlay_frames.csv")
        self.rgb_manifest_file = open(self.rgb_manifest_path, "w", newline="")
        self.rgb_manifest_writer = csv.writer(self.rgb_manifest_file)
        self.rgb_manifest_writer.writerow(
            [
                "video_frame_index",
                "wall_time_s",
                "rgb_timestamp_ns",
                "rgb_frames",
                "model_dot_x",
                "model_dot_y",
                "head_dot_x",
                "head_dot_y",
                "yaw_rad",
                "pitch_rad",
            ]
        )
        print(f"[recorder] Recording to {self.output_dir}")

    @staticmethod
    def _open_writer(path, frame, fps):
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(
            path,
            cv2.VideoWriter_fourcc(*"MJPG"),
            fps,
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video writer: {path}")
        return writer

    @staticmethod
    def _dot_x(dot):
        return "" if dot is None else int(dot[0])

    @staticmethod
    def _dot_y(dot):
        return "" if dot is None else int(dot[1])

    def write(
        self,
        rgb_image,
        eye_display_bgr,
        rgb_ts_ns,
        eye_ts_ns,
        rgb_frames,
        eye_frames,
        model_dot,
        old_dot,
        gaze,
        gaze_error,
    ):
        # Capture one wall-time stamp per call so both the gaze_stream.csv row
        # and the rgb_overlay_frames.csv row (if any) share the same clock
        # reading. This matters for cross-machine pairing with ROS bags.
        now_wall_s = time.time()

        wrote_any = False
        wrote_rgb = False
        if rgb_image is not None and rgb_ts_ns != self.last_rgb_ts_ns:
            rgb_frame = make_recording_rgb_frame(rgb_image, model_dot, old_dot, gaze, gaze_error)
            if self.rgb_writer is None:
                self.rgb_writer = self._open_writer(
                    os.path.join(self.output_dir, "rgb_overlay.avi"),
                    rgb_frame,
                    self.fps,
                )
            self.rgb_writer.write(rgb_frame)
            self.last_rgb_ts_ns = rgb_ts_ns
            wrote_any = True
            wrote_rgb = True

        if eye_display_bgr is not None and eye_ts_ns != self.last_eye_ts_ns:
            if self.eye_writer is None:
                self.eye_writer = self._open_writer(
                    os.path.join(self.output_dir, "eyetrack_overlay.avi"),
                    eye_display_bgr,
                    self.fps,
                )
            self.eye_writer.write(eye_display_bgr)
            self.last_eye_ts_ns = eye_ts_ns
            wrote_any = True

        if not wrote_any:
            return

        if gaze is None:
            yaw_rad = pitch_rad = yaw_deg = pitch_deg = inference_ms = ""
            cpf = ["", "", ""]
        else:
            yaw_rad = gaze.yaw_rad
            pitch_rad = gaze.pitch_rad
            yaw_deg = np.degrees(gaze.yaw_rad) if np.isfinite(gaze.yaw_rad) else ""
            pitch_deg = np.degrees(gaze.pitch_rad) if np.isfinite(gaze.pitch_rad) else ""
            cpf_vec = gaze.vector_cpf
            cpf = [cpf_vec[0], cpf_vec[1], cpf_vec[2]]
            inference_ms = gaze.inference_ms

        self.csv_writer.writerow(
            [
                now_wall_s,
                rgb_ts_ns,
                eye_ts_ns,
                rgb_frames,
                eye_frames,
                self._dot_x(model_dot),
                self._dot_y(model_dot),
                self._dot_x(old_dot),
                self._dot_y(old_dot),
                yaw_rad,
                pitch_rad,
                yaw_deg,
                pitch_deg,
                cpf[0],
                cpf[1],
                cpf[2],
                inference_ms,
                gaze_error or "",
            ]
        )
        self.csv_file.flush()

        # One manifest row per frame actually written to rgb_overlay.avi.
        # video_frame_index is 0-based and matches the frame index inside the
        # AVI container, so downstream tools can seek by this index.
        if wrote_rgb:
            self.rgb_video_frame_index += 1
            if gaze is None:
                manifest_yaw_rad = ""
                manifest_pitch_rad = ""
            else:
                manifest_yaw_rad = gaze.yaw_rad if np.isfinite(gaze.yaw_rad) else ""
                manifest_pitch_rad = gaze.pitch_rad if np.isfinite(gaze.pitch_rad) else ""
            self.rgb_manifest_writer.writerow(
                [
                    self.rgb_video_frame_index,
                    now_wall_s,
                    rgb_ts_ns,
                    rgb_frames,
                    self._dot_x(model_dot),
                    self._dot_y(model_dot),
                    self._dot_x(old_dot),
                    self._dot_y(old_dot),
                    manifest_yaw_rad,
                    manifest_pitch_rad,
                ]
            )
            self.rgb_manifest_file.flush()

    def close(self):
        if self.rgb_writer is not None:
            self.rgb_writer.release()
        if self.eye_writer is not None:
            self.eye_writer.release()
        self.csv_file.close()
        if self.rgb_manifest_file is not None:
            self.rgb_manifest_file.close()
            self.rgb_manifest_file = None
        print(f"[recorder] Saved recording to {self.output_dir}")


def start_streaming(args):
    aria.set_log_level(aria.Level.Info)

    print("[eye-viewer] Connecting to Aria glasses...")
    device_client = aria.DeviceClient()
    client_config = aria.DeviceClientConfig()
    if args.device_ip:
        client_config.ip_v4_address = args.device_ip
    if args.serial:
        client_config.device_serial = args.serial
    device_client.set_client_config(client_config)

    device = device_client.connect()
    print("[eye-viewer] Connected.")

    streaming_manager = device.streaming_manager

    if args.stop_previous:
        try:
            print("[eye-viewer] Stopping any previous streaming session...")
            streaming_manager.stop_streaming()
            time.sleep(2.0)
        except Exception as exc:
            print(f"[eye-viewer] Previous stream stop skipped: {exc}")
    else:
        print("[eye-viewer] Skipping previous-stream stop; starting directly.")

    streaming_config = aria.StreamingConfig()
    streaming_config.profile_name = args.profile
    if args.interface == "usb":
        streaming_config.streaming_interface = aria.StreamingInterface.Usb
    streaming_config.security_options.use_ephemeral_certs = args.ephemeral_certs
    streaming_manager.streaming_config = streaming_config

    print(f"[eye-viewer] Starting stream profile={args.profile} interface={args.interface}...")
    streaming_manager.start_streaming()
    print(f"[eye-viewer] Streaming state: {streaming_manager.streaming_state}")

    print("[eye-viewer] Waiting 5s for USB-NCM/DDS to settle...")
    time.sleep(5.0)
    try:
        ping_ip = args.device_ip or "192.168.42.129"
        ret = subprocess.run(
            ping_command(ping_ip),
            capture_output=True,
            timeout=10,
        )
        if ret.returncode == 0:
            print(f"[eye-viewer] Aria glasses reachable at {ping_ip}.")
        else:
            print(f"[eye-viewer] WARNING: ping to {ping_ip} failed; continuing anyway.")
    except Exception as exc:
        print(f"[eye-viewer] WARNING: ping check failed: {exc}")

    # Create/access the DDS streaming client only after USB-NCM exists.
    # In WSL, creating it too early can leave FastDDS bound to the wrong
    # interface and the subscription receives zero callbacks.
    if args.client_mode == "standalone":
        print("[eye-viewer] Creating standalone StreamingClient after USB-NCM is up.")
        streaming_client = aria.StreamingClient()
    else:
        print("[eye-viewer] Using StreamingManager-provided StreamingClient.")
        streaming_client = streaming_manager.streaming_client

    config = streaming_client.subscription_config
    if args.debug_rgb_slam_only:
        config.subscriber_data_type = (
            aria.StreamingDataType.Rgb
            | aria.StreamingDataType.Slam
            | aria.StreamingDataType.Imu
        )
        config.message_queue_size[aria.StreamingDataType.Rgb] = 1
        config.message_queue_size[aria.StreamingDataType.Slam] = 1
        config.message_queue_size[aria.StreamingDataType.Imu] = 1
        print("[eye-viewer] Debug mode: subscribing to RGB+SLAM+IMU only.")
    elif args.subscribe_all:
        config.subscriber_data_type = (
            aria.StreamingDataType.Rgb
            | aria.StreamingDataType.Slam
            | aria.StreamingDataType.EyeTrack
            | aria.StreamingDataType.Imu
        )
        config.message_queue_size[aria.StreamingDataType.Rgb] = 1
        config.message_queue_size[aria.StreamingDataType.Slam] = 1
        config.message_queue_size[aria.StreamingDataType.Imu] = 1
    else:
        config.subscriber_data_type = aria.StreamingDataType.Rgb | aria.StreamingDataType.EyeTrack
        config.message_queue_size[aria.StreamingDataType.Rgb] = 1
    config.message_queue_size[aria.StreamingDataType.EyeTrack] = 1

    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = args.ephemeral_certs
    config.security_options = options
    streaming_client.subscription_config = config

    gaze_runner = None
    args.gaze_projector = None
    args.gaze_projection_mode = "none"
    if args.gaze_source == "open-model":
        args.gaze_projector = make_gaze_rgb_projector(device, args)
        args.gaze_projection_mode = "rgb_factory_projection_scaled" if args.gaze_projector is not None else "angle_pad_fallback"
        print("[eye-viewer] Loading open-source Project Aria eye-tracking model...")
        estimator = OpenSourceEyeGazeEstimator(
            model_path=args.gaze_model_path,
            model_config=args.gaze_model_config,
            device=args.gaze_model_device,
        )
        gaze_runner = AsyncEyeGazeRunner(estimator)
        print(f"[eye-viewer] Open-model gaze enabled: {gaze_runner.model_description}")

    observer = EyeTrackObserver(gaze_runner=gaze_runner)
    streaming_client.set_streaming_client_observer(observer)
    streaming_client.subscribe()
    print("[eye-viewer] Subscribed to streaming callbacks.")

    return device_client, device, streaming_manager, streaming_client, observer


def _set_scatter(scatter_items, dot):
    if dot is None:
        offsets = np.empty((0, 2))
    else:
        offsets = np.array([[dot[0], dot[1]]], dtype=float)
    for item in scatter_items:
        item.set_offsets(offsets)


def wait_for_rgb_frame(observer, rgb_artist, rgb_ax, status, timeout_sec=20.0):
    start = time.time()
    last_shape = None
    while time.time() - start < timeout_sec:
        (
            rgb_image,
            _eye_image,
            _any_image,
            _any_camera_id,
            _rgb_ts_ns,
            _eye_ts_ns,
            _rgb_frames,
            _eye_frames,
            _imu_frames,
            _last_rgb_time,
            _last_eye_time,
            _camera_counts,
            _gaze,
            _gaze_error,
        ) = observer.get_latest()
        if rgb_image is not None:
            if rgb_image.shape != last_shape:
                rgb_artist.set_data(rgb_image)
                rgb_artist.set_extent((0, rgb_image.shape[1], rgb_image.shape[0], 0))
                rgb_ax.set_xlim(0, rgb_image.shape[1])
                rgb_ax.set_ylim(rgb_image.shape[0], 0)
                last_shape = rgb_image.shape
            else:
                rgb_artist.set_data(rgb_image)
            return rgb_image

        status.set_text("Waiting for RGB stream before calibration...")
        plt.pause(0.03)

    return None


def collect_gaze_calibration(
    observer,
    args,
    fig,
    rgb_ax,
    eye_ax,
    rgb_artist,
    eye_artist,
    eye_dot_scatter,
    dot_scatter_items,
    old_dot_scatter_items,
    status,
):
    rgb_image = wait_for_rgb_frame(observer, rgb_artist, rgb_ax, status)
    if rgb_image is None:
        print("[calibration] No RGB frame arrived; skipping calibration.")
        return GazeCalibration()

    targets = make_calibration_targets(rgb_image.shape, args.calibration_grid)
    model_samples = []
    old_samples = []
    target_scatter = rgb_ax.scatter([], [], s=1200, c="#ff1f1f", marker="+", linewidths=4)
    target_ring = rgb_ax.scatter([], [], s=900, facecolors="none", edgecolors="#ff1f1f", marker="o", linewidths=3)

    print(
        f"[calibration] Starting {len(targets)} targets. "
        f"Stare at each red target until it moves."
    )

    for target_index, target in enumerate(targets, start=1):
        target_offsets = np.array([[target[0], target[1]]], dtype=float)
        target_scatter.set_offsets(target_offsets)
        target_ring.set_offsets(target_offsets)

        sample_start = time.time()
        model_count = 0
        old_count = 0
        target_model_points = []
        target_old_points = []
        while True:
            elapsed = time.time() - sample_start
            if elapsed >= args.calibration_settle_sec + args.calibration_sample_sec:
                break

            (
                rgb_image,
                eye_image,
                any_image,
                _any_camera_id,
                _rgb_ts_ns,
                _eye_ts_ns,
                _rgb_frames,
                _eye_frames,
                _imu_frames,
                _last_rgb_time,
                _last_eye_time,
                _camera_counts,
                gaze,
                _gaze_error,
            ) = observer.get_latest()

            dots = []
            if rgb_image is not None:
                rgb_artist.set_data(rgb_image)
                rgb_artist.set_extent((0, rgb_image.shape[1], rgb_image.shape[0], 0))
                rgb_ax.set_xlim(0, rgb_image.shape[1])
                rgb_ax.set_ylim(rgb_image.shape[0], 0)

            if eye_image is not None:
                eye_display_bgr, dots = overlay_eye_dots(eye_image)
                eye_display = cv2.cvtColor(eye_display_bgr, cv2.COLOR_BGR2RGB)
                eye_artist.set_data(eye_display)
                eye_artist.set_extent((0, eye_display.shape[1], eye_display.shape[0], 0))
                eye_ax.set_xlim(0, eye_display.shape[1])
                eye_ax.set_ylim(eye_display.shape[0], 0)
                if dots:
                    eye_dot_scatter.set_offsets(np.array([[x, y] for x, y, _area in dots], dtype=float))
                else:
                    eye_dot_scatter.set_offsets(np.empty((0, 2)))
            elif any_image is not None:
                fallback = any_image
                if fallback.ndim == 2:
                    fallback = cv2.cvtColor(fallback, cv2.COLOR_GRAY2RGB)
                eye_artist.set_data(np.rot90(fallback))

            old_dot = None
            if rgb_image is not None and args.gaze_source == "open-model":
                old_dot = estimate_rgb_dot_from_head(
                    rgb_image.shape,
                    projector=getattr(args, "gaze_projector", None),
                )
            elif rgb_image is not None and eye_image is not None:
                old_dot = estimate_rgb_dot_from_eye_dots(rgb_image.shape, eye_image.shape, dots)

            model_dot = None
            if rgb_image is not None and args.gaze_source == "open-model":
                model_dot = estimate_rgb_dot_from_gaze(
                    rgb_image.shape,
                    gaze,
                    np.radians(args.gaze_max_angle_deg),
                    projector=getattr(args, "gaze_projector", None),
                )

            _set_scatter(dot_scatter_items, model_dot if args.gaze_source == "open-model" else old_dot)
            _set_scatter(old_dot_scatter_items, old_dot if args.gaze_source == "open-model" else None)

            collecting = elapsed >= args.calibration_settle_sec
            if collecting:
                if model_dot is not None:
                    model_samples.append((model_dot, target))
                    target_model_points.append(model_dot)
                    model_count += 1
                if old_dot is not None and args.gaze_source != "open-model":
                    old_samples.append((old_dot, target))
                    target_old_points.append(old_dot)
                    old_count += 1

            phase = "collecting" if collecting else "settling"
            status.set_text(
                f"Calibration target {target_index}/{len(targets)} {phase}: "
                f"look at the red target | model samples={model_count} old samples={old_count}"
            )
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.03)

        if args.gaze_source == "open-model":
            print_target_calibration_debug("model", target_index, target, target_model_points)
        if args.gaze_source != "open-model":
            print_target_calibration_debug("old", target_index, target, target_old_points)

    _set_scatter([target_scatter, target_ring], None)

    metadata = {
        "grid": int(args.calibration_grid),
        "settle_sec": float(args.calibration_settle_sec),
        "sample_sec": float(args.calibration_sample_sec),
        "image_shape": list(rgb_image.shape) if rgb_image is not None else None,
        "gaze_source": args.gaze_source,
        "model_projection": getattr(args, "gaze_projection_mode", "none"),
        "gaze_projection_depth_m": float(getattr(args, "gaze_projection_depth_m", 1.0)),
    }
    calibration = GazeCalibration.fit(model_samples, old_samples, metadata=metadata)

    print(f"[calibration] Finished: {calibration.summary()}")
    if args.gaze_source == "open-model":
        print_fit_residual_debug("model", calibration.model, model_samples)
    if args.gaze_source != "open-model":
        print_fit_residual_debug("old", calibration.old, old_samples)
    calibration = reject_bad_calibration(calibration, args, "fit")
    if args.calibration_file:
        calibration.save(args.calibration_file)
        print(f"[calibration] Saved {args.calibration_file}: {calibration.summary()}")

    return calibration


def run_matplotlib_window(observer: EyeTrackObserver, args):
    plt.ion()
    fig, (rgb_ax, eye_ax) = plt.subplots(
        1,
        2,
        figsize=(13, 7),
        gridspec_kw={"width_ratios": [3.2, 1.2]},
    )
    fig.canvas.manager.set_window_title("Aria RGB + Eye Tracking Dot")

    rgb_canvas = np.zeros((704, 704, 3), dtype=np.uint8)
    eye_canvas = np.zeros((240, 640, 3), dtype=np.uint8)

    rgb_artist = rgb_ax.imshow(rgb_canvas)
    eye_artist = eye_ax.imshow(eye_canvas)

    if args.gaze_source == "open-model":
        rgb_ax.set_title("Aria RGB stream - magenta=open-model eye, cyan=head forward")
    else:
        rgb_ax.set_title("Aria RGB stream - magenta dot is live pupil proxy")
    eye_ax.set_title("EyeTrack camera")
    rgb_ax.axis("off")
    eye_ax.axis("off")

    # Layered markers make the dot visible on bright or dark scenes.
    dot_outer = rgb_ax.scatter([], [], s=900, c="black", marker="o", alpha=0.85)
    dot_mid = rgb_ax.scatter([], [], s=520, c="#ff00ff", marker="o", alpha=0.95)
    dot_inner = rgb_ax.scatter([], [], s=120, c="#fff200", marker="o", alpha=1.0)
    old_dot_outer = rgb_ax.scatter([], [], s=520, c="black", marker="s", alpha=0.85)
    old_dot_mid = rgb_ax.scatter([], [], s=300, c="#00ffff", marker="s", alpha=0.95)
    old_dot_inner = rgb_ax.scatter([], [], s=80, c="#003a3a", marker="s", alpha=1.0)
    eye_dot_scatter = eye_ax.scatter([], [], s=90, c="#00ff66", marker="o")

    status = fig.text(
        0.02,
        0.02,
        "Waiting for RGB/EyeTrack callbacks...",
        fontsize=10,
        family="monospace",
    )

    start = time.time()
    last_print = 0.0
    last_rgb_shape = rgb_canvas.shape
    recorder = StreamRecorder(args.record_dir, fps=args.record_fps) if args.record_dir else None
    calibration = load_gaze_calibration(args)

    print("[eye-viewer] Matplotlib window opened. Close it or press Ctrl+C to stop.")
    if calibration.available:
        print(f"[calibration] Applying correction: {calibration.summary()}")

    if args.calibrate_gaze:
        calibration = collect_gaze_calibration(
            observer,
            args,
            fig,
            rgb_ax,
            eye_ax,
            rgb_artist,
            eye_artist,
            eye_dot_scatter,
            [dot_outer, dot_mid, dot_inner],
            [old_dot_outer, old_dot_mid, old_dot_inner],
            status,
        )

    try:
        while plt.fignum_exists(fig.number):
            (
                rgb_image,
                eye_image,
                any_image,
                any_camera_id,
                rgb_ts_ns,
                eye_ts_ns,
                rgb_frames,
                eye_frames,
                imu_frames,
                last_rgb_time,
                last_eye_time,
                camera_counts,
                gaze,
                gaze_error,
            ) = observer.get_latest()

            dots = []
            eye_display_bgr = None
            if rgb_image is not None:
                if rgb_image.shape != last_rgb_shape:
                    rgb_artist.set_data(rgb_image)
                    rgb_artist.set_extent((0, rgb_image.shape[1], rgb_image.shape[0], 0))
                    rgb_ax.set_xlim(0, rgb_image.shape[1])
                    rgb_ax.set_ylim(rgb_image.shape[0], 0)
                    last_rgb_shape = rgb_image.shape
                else:
                    rgb_artist.set_data(rgb_image)

            if eye_image is not None:
                eye_display_bgr, dots = overlay_eye_dots(eye_image)
                eye_display = cv2.cvtColor(eye_display_bgr, cv2.COLOR_BGR2RGB)
                eye_artist.set_data(eye_display)
                eye_artist.set_extent((0, eye_display.shape[1], eye_display.shape[0], 0))
                eye_ax.set_xlim(0, eye_display.shape[1])
                eye_ax.set_ylim(eye_display.shape[0], 0)
                if dots:
                    eye_dot_scatter.set_offsets(np.array([[x, y] for x, y, _area in dots], dtype=float))
                else:
                    eye_dot_scatter.set_offsets(np.empty((0, 2)))
            elif any_image is not None:
                fallback = any_image
                if fallback.ndim == 2:
                    fallback = cv2.cvtColor(fallback, cv2.COLOR_GRAY2RGB)
                eye_artist.set_data(np.rot90(fallback))

            old_rgb_dot = None
            if rgb_image is not None and args.gaze_source == "open-model":
                old_rgb_dot = estimate_rgb_dot_from_head(
                    rgb_image.shape,
                    projector=getattr(args, "gaze_projector", None),
                )
            elif rgb_image is not None and eye_image is not None:
                old_rgb_dot = estimate_rgb_dot_from_eye_dots(rgb_image.shape, eye_image.shape, dots)
            old_rgb_dot_raw = old_rgb_dot
            if rgb_image is not None and args.gaze_source != "open-model":
                old_rgb_dot = calibration.apply_old(old_rgb_dot, image_shape=rgb_image.shape)
            old_rgb_dot_corrected = old_rgb_dot

            rgb_dot = None
            if rgb_image is not None and args.gaze_source == "open-model":
                rgb_dot = estimate_rgb_dot_from_gaze(
                    rgb_image.shape,
                    gaze,
                    np.radians(args.gaze_max_angle_deg),
                    projector=getattr(args, "gaze_projector", None),
                )
            else:
                rgb_dot = old_rgb_dot_raw
            rgb_dot_raw = rgb_dot
            if rgb_image is not None:
                if args.gaze_source == "open-model":
                    rgb_dot = calibration.apply_model(rgb_dot, image_shape=rgb_image.shape)
                else:
                    rgb_dot = calibration.apply_old(rgb_dot, image_shape=rgb_image.shape)
            rgb_dot_corrected = rgb_dot

            if rgb_dot is None:
                empty = np.empty((0, 2))
                dot_outer.set_offsets(empty)
                dot_mid.set_offsets(empty)
                dot_inner.set_offsets(empty)
            else:
                dot_offsets = np.array([[rgb_dot[0], rgb_dot[1]]], dtype=float)
                dot_outer.set_offsets(dot_offsets)
                dot_mid.set_offsets(dot_offsets)
                dot_inner.set_offsets(dot_offsets)

            if args.gaze_source == "open-model" and old_rgb_dot is not None:
                old_dot_offsets = np.array([[old_rgb_dot[0], old_rgb_dot[1]]], dtype=float)
                old_dot_outer.set_offsets(old_dot_offsets)
                old_dot_mid.set_offsets(old_dot_offsets)
                old_dot_inner.set_offsets(old_dot_offsets)
            else:
                empty = np.empty((0, 2))
                old_dot_outer.set_offsets(empty)
                old_dot_mid.set_offsets(empty)
                old_dot_inner.set_offsets(empty)

            if recorder is not None:
                recorder.write(
                    rgb_image,
                    eye_display_bgr,
                    rgb_ts_ns,
                    eye_ts_ns,
                    rgb_frames,
                    eye_frames,
                    rgb_dot,
                    old_rgb_dot if args.gaze_source == "open-model" else None,
                    gaze,
                    gaze_error,
                )

            now = time.time()
            rgb_fps = rgb_frames / max(now - start, 1e-6)
            eye_fps = eye_frames / max(now - start, 1e-6)
            rgb_age_ms = (now - last_rgb_time) * 1000.0 if last_rgb_time else -1.0
            eye_age_ms = (now - last_eye_time) * 1000.0 if last_eye_time else -1.0
            if args.gaze_source == "open-model":
                dot_text = f"model_dot={rgb_dot} head_dot={old_rgb_dot} | {format_gaze_text(gaze, gaze_error)}"
            else:
                dot_text = f"dot={rgb_dot}" if rgb_dot is not None else "dot=waiting"
            status.set_text(
                f"RGB frames={rgb_frames} fps={rgb_fps:.1f} age={rgb_age_ms:.0f}ms | "
                f"EyeTrack frames={eye_frames} fps={eye_fps:.1f} age={eye_age_ms:.0f}ms | "
                f"{dot_text} | rgb_ts={rgb_ts_ns} eye_ts={eye_ts_ns}"
            )

            if now - last_print > 2.0:
                last_print = now
                counts = ", ".join(f"{camera_id}={count}" for camera_id, count in camera_counts.items())
                print(
                    "[eye-viewer] "
                    f"rgb_frames={rgb_frames} eye_frames={eye_frames} imu_frames={imu_frames} "
                    f"| camera_counts: {counts or 'none'}"
                )
                if args.gaze_source == "open-model":
                    model_adjust = None
                    if rgb_dot_raw is not None and rgb_dot_corrected is not None:
                        model_adjust = np.array(rgb_dot_corrected, dtype=np.float64) - np.array(rgb_dot_raw, dtype=np.float64)
                    old_adjust = None
                    if old_rgb_dot_raw is not None and old_rgb_dot_corrected is not None:
                        old_adjust = np.array(old_rgb_dot_corrected, dtype=np.float64) - np.array(old_rgb_dot_raw, dtype=np.float64)
                    print(
                        f"[eye-viewer] {format_gaze_text(gaze, gaze_error)} "
                        f"model_dot raw={rgb_dot_raw} corrected={rgb_dot_corrected} "
                        f"adjust={format_delta(model_adjust)} "
                        f"head_dot raw={old_rgb_dot_raw} corrected={old_rgb_dot_corrected} "
                        f"adjust={format_delta(old_adjust)} "
                        f"{format_projection_debug(getattr(args, 'gaze_projector', None))}"
                    )
                if any_camera_id is not None and not (rgb_frames or eye_frames):
                    print(f"[eye-viewer] Last callback camera was {any_camera_id}")

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.03)
    finally:
        if recorder is not None:
            recorder.close()


def run_window(observer: EyeTrackObserver, args):
    window = "Aria EyeTrack - pupil dot overlay"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 480)

    last_print = 0.0
    start = time.time()

    while True:
        (
            _rgb_image,
            image,
            any_image,
            any_camera_id,
            _rgb_ts_ns,
            ts_ns,
            _rgb_frames,
            frames,
            imu_frames,
            _last_rgb_frame_time,
            last_frame_time,
            camera_counts,
            gaze,
            gaze_error,
        ) = observer.get_latest()

        if image is None:
            if any_image is not None:
                if any_image.ndim == 2:
                    canvas = cv2.cvtColor(any_image, cv2.COLOR_GRAY2BGR)
                else:
                    canvas = any_image.copy()
                canvas = np.ascontiguousarray(np.rot90(canvas))
                cv2.putText(
                    canvas,
                    f"No EyeTrack yet - showing debug camera {any_camera_id}",
                    (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            else:
                canvas = np.zeros((240, 640, 3), dtype=np.uint8)
                cv2.putText(
                    canvas,
                    "Waiting for streaming callbacks...",
                    (40, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.putText(
                canvas,
                "q/Esc to quit",
                (40, canvas.shape[0] - 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(window, canvas)
        else:
            display, dots = overlay_eye_dots(image)
            fps = frames / max(time.time() - start, 1e-6)
            age_ms = (time.time() - last_frame_time) * 1000.0
            cv2.putText(
                display,
                f"EyeTrack frames={frames} fps={fps:.1f} age={age_ms:.0f}ms dots={len(dots)} ts={ts_ns}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            if args.gaze_source == "open-model":
                cv2.putText(
                    display,
                    format_gaze_text(gaze, gaze_error),
                    (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 0, 255),
                    1,
                    cv2.LINE_AA,
                )
            cv2.imshow(window, display)

        now = time.time()
        if now - last_print > 2.0:
            last_print = now
            status = "receiving" if image is not None else "waiting"
            counts = ", ".join(f"{camera_id}={count}" for camera_id, count in camera_counts.items())
            print(f"[eye-viewer] {status} | eyetrack_frames={frames} | camera_counts: {counts or 'none'}")
            print(f"[eye-viewer] imu_frames={imu_frames}")
            if args.gaze_source == "open-model":
                print(f"[eye-viewer] {format_gaze_text(gaze, gaze_error)}")

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break

    cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(description="Live Aria EyeTrack viewer with dot overlay")
    parser.add_argument("--profile", default="profile18", help="Streaming profile (default: profile18)")
    parser.add_argument("--interface", choices=["usb", "wifi"], default="usb")
    parser.add_argument("--device-ip", default=None, help="Aria IP for Wi-Fi mode")
    parser.add_argument("--serial", default=None, help="Optional device serial")
    parser.add_argument(
        "--ephemeral-certs",
        action="store_true",
        help="Use ephemeral certs instead of persistent local certs",
    )
    parser.add_argument(
        "--stop-previous",
        action="store_true",
        help="Try to stop a previous streaming session before starting.",
    )
    parser.add_argument(
        "--subscribe-all",
        action="store_true",
        help="Subscribe to RGB+SLAM+EyeTrack for debugging; only EyeTrack is displayed",
    )
    parser.add_argument(
        "--debug-rgb-slam-only",
        action="store_true",
        help="Subscribe exactly like aria_debug_stream.py: RGB+SLAM+IMU, no EyeTrack",
    )
    parser.add_argument(
        "--display",
        choices=["matplotlib", "opencv-eye"],
        default="matplotlib",
        help="Display mode. matplotlib shows RGB with a large dot; opencv-eye shows only EyeTrack.",
    )
    parser.add_argument(
        "--client-mode",
        choices=["standalone", "manager"],
        default="standalone",
        help="Use a standalone StreamingClient after USB-NCM is up, or the manager-provided client.",
    )
    parser.add_argument(
        "--gaze-source",
        choices=["heuristic", "open-model"],
        default="heuristic",
        help="Use the old pupil heuristic or the open-source Project Aria eye-tracking model.",
    )
    parser.add_argument(
        "--gaze-model-path",
        default=None,
        help="Path to the open-source Project Aria eye-tracking model weights.",
    )
    parser.add_argument(
        "--gaze-model-config",
        default=None,
        help="Path to the open-source Project Aria eye-tracking model YAML config.",
    )
    parser.add_argument(
        "--gaze-model-device",
        default="cpu",
        help="Torch device for open-model inference, for example cpu or cuda:0.",
    )
    parser.add_argument(
        "--gaze-max-angle-deg",
        type=float,
        default=30.0,
        help="Fallback only: yaw/pitch angle that maps to the edge of the live gaze dot display.",
    )
    parser.add_argument(
        "--gaze-projection-depth-m",
        type=float,
        default=1.0,
        help="Fixed depth used when reprojecting CPF gaze into the RGB image.",
    )
    parser.add_argument(
        "--disable-upright-gaze-projection",
        action="store_true",
        help="Project onto the raw RGB orientation instead of the upright/rotated RGB view.",
    )
    parser.add_argument(
        "--calibrate-gaze",
        action="store_true",
        help="Run on-screen target calibration and save an affine correction for both dots.",
    )
    parser.add_argument(
        "--calibration-file",
        default=os.path.join(CARTRON_ROOT, "calibrations", "aria_eye_calibration.json"),
        help="Path for loading/saving gaze-dot calibration.",
    )
    parser.add_argument(
        "--disable-gaze-calibration",
        action="store_true",
        help="Do not load or apply a saved gaze-dot calibration.",
    )
    parser.add_argument(
        "--max-calibration-rmse-px",
        type=float,
        default=250.0,
        help="Reject saved or newly fitted dot calibration above this RMSE. Use 0 to allow any fit.",
    )
    parser.add_argument(
        "--calibration-grid",
        type=int,
        default=3,
        help="Number of calibration targets per axis. 3 gives 9 targets.",
    )
    parser.add_argument(
        "--calibration-settle-sec",
        type=float,
        default=0.8,
        help="Seconds to wait after each target appears before collecting samples.",
    )
    parser.add_argument(
        "--calibration-sample-sec",
        type=float,
        default=1.2,
        help="Seconds of gaze samples collected per calibration target.",
    )
    parser.add_argument(
        "--record-dir",
        default=None,
        help="Optional directory for recording RGB overlay video, EyeTrack video, and gaze CSV.",
    )
    parser.add_argument(
        "--record-fps",
        type=float,
        default=30.0,
        help="Video FPS metadata for recordings written with --record-dir.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    device_client = None
    device = None
    streaming_manager = None
    streaming_client = None
    observer = None

    def handle_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        device_client, device, streaming_manager, streaming_client, observer = start_streaming(args)
        if args.display == "opencv-eye":
            run_window(observer, args)
        else:
            run_matplotlib_window(observer, args)
    except KeyboardInterrupt:
        print("\n[eye-viewer] Stopping...")
    finally:
        try:
            if streaming_client is not None:
                streaming_client.unsubscribe()
        except Exception:
            pass
        try:
            if streaming_manager is not None:
                streaming_manager.stop_streaming()
        except Exception:
            pass
        try:
            if device_client is not None and device is not None:
                device_client.disconnect(device)
        except Exception:
            pass
        try:
            if observer is not None:
                observer.close()
        except Exception:
            pass
        print("[eye-viewer] Done.")


if __name__ == "__main__":
    main()
