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


def ping_command(ip_address: str):
    if sys.platform.startswith("win"):
        return ["ping", "-n", "2", "-w", "2000", ip_address]
    return ["ping", "-c", "2", "-W", "2", ip_address]


class EyeTrackObserver:
    def __init__(self):
        self._lock = threading.Lock()
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
            )


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

    observer = EyeTrackObserver()
    streaming_client.set_streaming_client_observer(observer)
    streaming_client.subscribe()
    print("[eye-viewer] Subscribed to streaming callbacks.")

    return device_client, device, streaming_manager, streaming_client, observer


def run_matplotlib_window(observer: EyeTrackObserver):
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

    rgb_ax.set_title("Aria RGB stream - magenta dot is live pupil proxy")
    eye_ax.set_title("EyeTrack camera")
    rgb_ax.axis("off")
    eye_ax.axis("off")

    # Layered markers make the dot visible on bright or dark scenes.
    dot_outer = rgb_ax.scatter([], [], s=900, c="black", marker="o", alpha=0.85)
    dot_mid = rgb_ax.scatter([], [], s=520, c="#ff00ff", marker="o", alpha=0.95)
    dot_inner = rgb_ax.scatter([], [], s=120, c="#fff200", marker="o", alpha=1.0)
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

    print("[eye-viewer] Matplotlib window opened. Close it or press Ctrl+C to stop.")
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
        ) = observer.get_latest()

        dots = []
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
            eye_display, dots = overlay_eye_dots(eye_image)
            eye_display = cv2.cvtColor(eye_display, cv2.COLOR_BGR2RGB)
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

        rgb_dot = None
        if rgb_image is not None and eye_image is not None:
            rgb_dot = estimate_rgb_dot_from_eye_dots(rgb_image.shape, eye_image.shape, dots)

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

        now = time.time()
        rgb_fps = rgb_frames / max(now - start, 1e-6)
        eye_fps = eye_frames / max(now - start, 1e-6)
        rgb_age_ms = (now - last_rgb_time) * 1000.0 if last_rgb_time else -1.0
        eye_age_ms = (now - last_eye_time) * 1000.0 if last_eye_time else -1.0
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
            if any_camera_id is not None and not (rgb_frames or eye_frames):
                print(f"[eye-viewer] Last callback camera was {any_camera_id}")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.03)


def run_window(observer: EyeTrackObserver):
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
            cv2.imshow(window, display)

        now = time.time()
        if now - last_print > 2.0:
            last_print = now
            status = "receiving" if image is not None else "waiting"
            counts = ", ".join(f"{camera_id}={count}" for camera_id, count in camera_counts.items())
            print(f"[eye-viewer] {status} | eyetrack_frames={frames} | camera_counts: {counts or 'none'}")
            print(f"[eye-viewer] imu_frames={imu_frames}")

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
    return parser.parse_args()


def main():
    args = parse_args()

    device_client = None
    device = None
    streaming_manager = None
    streaming_client = None

    def handle_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)

    try:
        device_client, device, streaming_manager, streaming_client, observer = start_streaming(args)
        if args.display == "opencv-eye":
            run_window(observer)
        else:
            run_matplotlib_window(observer)
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
        print("[eye-viewer] Done.")


if __name__ == "__main__":
    main()
