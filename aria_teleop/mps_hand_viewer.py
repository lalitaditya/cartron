"""
MPS Hand Tracking Viewer
========================
Loads MPS hand_tracking_results.csv and plays back the 21-keypoint hand
skeleton in a 3D matplotlib window, synchronized to recording timestamps.

Usage:
    python mps_hand_viewer.py <path/to/hand_tracking/hand_tracking_results.csv>

    Example:
    python mps_hand_viewer.py ~/Recordings/mps_<id>/hand_tracking/hand_tracking_results.csv
"""
import sys
import time
import argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import projectaria_tools.core.mps as mps

# ─── Hand skeleton connections (21-point model) ─────────────────────────────
# Indices match projectaria_tools.core.mps.hand_tracking.HandLandmark
FINGER_CONNECTIONS = [
    # Thumb
    (5, 6), (6, 7), (7, 0),
    # Index
    (5, 8), (8, 9), (9, 10), (10, 1),
    # Middle
    (5, 11), (11, 12), (12, 13), (13, 2),
    # Ring
    (5, 14), (14, 15), (15, 16), (16, 3),
    # Pinky
    (5, 17), (17, 18), (18, 19), (19, 4),
    # Palm
    (5, 20),
]

FINGER_COLORS = {
    "thumb":  "#FF6B6B",
    "index":  "#4ECDC4",
    "middle": "#45B7D1",
    "ring":   "#96CEB4",
    "pinky":  "#FFEAA7",
    "palm":   "#DDA0DD",
}

# Map connection index ranges to finger names
def get_finger_color(conn_idx):
    if conn_idx < 3: return FINGER_COLORS["thumb"]
    if conn_idx < 7: return FINGER_COLORS["index"]
    if conn_idx < 11: return FINGER_COLORS["middle"]
    if conn_idx < 15: return FINGER_COLORS["ring"]
    if conn_idx < 19: return FINGER_COLORS["pinky"]
    return FINGER_COLORS["palm"]


def load_hand_tracking(csv_path: str):
    """Load MPS hand tracking results from CSV."""
    print(f"[viewer] Loading: {csv_path}")
    results = mps.hand_tracking.read_hand_tracking_results(csv_path)
    print(f"[viewer] Loaded {len(results)} frames")
    return results


def extract_landmarks(one_side):
    """Extract 21x3 numpy array from a per-hand result."""
    if one_side is None or one_side.confidence < 0.1:
        return None
    pts = one_side.landmark_positions_device
    if pts is None or len(pts) == 0:
        return None
    return np.array(pts)


def setup_axes(ax, title):
    """Configure a 3D axes for hand visualization."""
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(-0.3, 0.3)


def draw_hand(ax, landmarks, label, alpha=1.0):
    """Draw a hand skeleton on a 3D axes."""
    if landmarks is None:
        return

    # Draw joints
    ax.scatter(
        landmarks[:, 0], landmarks[:, 1], landmarks[:, 2],
        c="white", edgecolors="gray", s=30, zorder=5, alpha=alpha,
    )

    # Draw bones
    for i, (a, b) in enumerate(FINGER_CONNECTIONS):
        if a < len(landmarks) and b < len(landmarks):
            ax.plot(
                [landmarks[a, 0], landmarks[b, 0]],
                [landmarks[a, 1], landmarks[b, 1]],
                [landmarks[a, 2], landmarks[b, 2]],
                color=get_finger_color(i), linewidth=2.5, alpha=alpha,
            )

    # Mark wrist (index 5) specially
    ax.scatter(
        [landmarks[5, 0]], [landmarks[5, 1]], [landmarks[5, 2]],
        c="red", s=60, marker="^", zorder=6, alpha=alpha,
    )


def run_viewer(csv_path: str, speed: float = 1.0):
    """Main visualization loop: plays back hand tracking data frame-by-frame."""
    results = load_hand_tracking(csv_path)

    if not results:
        print("[viewer] No hand tracking data found!")
        return

    # Set up dark-themed figure
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(12, 6))
    fig.suptitle("MPS Hand Tracking Playback", fontsize=14, fontweight="bold")
    ax_left = fig.add_subplot(121, projection="3d")
    ax_right = fig.add_subplot(122, projection="3d")
    setup_axes(ax_left, "Left Hand")
    setup_axes(ax_right, "Right Hand")

    plt.ion()
    plt.show()

    # Get timestamps for pacing
    timestamps = []
    for r in results:
        # tracking_timestamp_ns is the timestamp field
        ts = getattr(r, "tracking_timestamp_ns", None)
        if ts is None:
            ts = getattr(r, "tracking_timestamp", 0)
        timestamps.append(ts)

    print(f"[viewer] Playing back {len(results)} frames at {speed}x speed...")
    print("[viewer] Close the window or Ctrl+C to stop.\n")

    frame_count = 0
    left_count = 0
    right_count = 0

    try:
        for i, result in enumerate(results):
            if not plt.fignum_exists(fig.number):
                break

            ax_left.cla()
            ax_right.cla()
            setup_axes(ax_left, "Left Hand")
            setup_axes(ax_right, "Right Hand")

            # Extract landmarks
            left_lm = extract_landmarks(result.left_hand)
            right_lm = extract_landmarks(result.right_hand)

            if left_lm is not None:
                draw_hand(ax_left, left_lm, "Left")
                left_count += 1
                conf = result.left_hand.confidence
                ax_left.set_title(f"Left Hand (conf: {conf:.2f})", fontsize=12)
            else:
                ax_left.set_title("Left Hand (not detected)", fontsize=12, color="gray")

            if right_lm is not None:
                draw_hand(ax_right, right_lm, "Right")
                right_count += 1
                conf = result.right_hand.confidence
                ax_right.set_title(f"Right Hand (conf: {conf:.2f})", fontsize=12)
            else:
                ax_right.set_title("Right Hand (not detected)", fontsize=12, color="gray")

            fig.suptitle(
                f"MPS Hand Tracking — Frame {i+1}/{len(results)}",
                fontsize=14, fontweight="bold",
            )

            plt.draw()
            plt.pause(0.001)

            # Pace playback using real timestamps
            if i + 1 < len(results) and timestamps[i] and timestamps[i + 1]:
                dt_ns = timestamps[i + 1] - timestamps[i]
                dt_s = max(dt_ns / 1e9, 0) / speed
                if dt_s < 1.0:  # Sanity cap
                    time.sleep(dt_s)

            frame_count += 1

            # Print progress every 50 frames
            if frame_count % 50 == 0:
                print(f"  Frame {frame_count}/{len(results)} | "
                      f"Left detected: {left_count} | Right detected: {right_count}")

    except KeyboardInterrupt:
        print("\n[viewer] Stopped by user.")

    print(f"\n[viewer] Done! Played {frame_count} frames.")
    print(f"  Left hand detected in {left_count} frames")
    print(f"  Right hand detected in {right_count} frames")

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize MPS hand tracking results in 3D"
    )
    parser.add_argument(
        "csv_path",
        help="Path to hand_tracking_results.csv from MPS output",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Playback speed multiplier (default: 1.0)",
    )
    args = parser.parse_args()
    run_viewer(args.csv_path, args.speed)
