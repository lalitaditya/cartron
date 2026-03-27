"""
Simulated Hand Tracker
======================
Generates synthetic hand tracking data for testing the Aria -> Piper
pipeline without actual Aria glasses.

Uses Aria Gen 2 landmark indices (THUMB_FINGERTIP=0, INDEX_FINGERTIP=1, WRIST=5, etc.)

Modes:
  1. "circle" - wrist traces a slow circle (default)
  2. "static" - wrist stays at a fixed position
  3. "sweep"  - wrist sweeps left-right with periodic pinch

Used by: aria_piper_bridge.py --sim
"""
import time
import math
import threading
import numpy as np
from typing import Callable, Optional

from aria_hand_tracker import HandData, NUM_LANDMARKS, WRIST, THUMB_FINGERTIP, INDEX_FINGERTIP


class SimHandTracker:
    """
    Generates synthetic hand tracking data at ~30Hz.
    Same interface as AriaHandTracker and UDPHandTracker.
    """

    def __init__(self, on_hand_update: Callable[[HandData], None],
                 mode: str = "circle",
                 hz: float = 30.0):
        self._on_hand_update = on_hand_update
        self._mode = mode
        self._hz = hz
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()
        print(f"[sim_tracker] Started simulation mode='{self._mode}' at {self._hz}Hz")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        print("[sim_tracker] Stopped.")

    @property
    def is_running(self) -> bool:
        return self._running

    def _sim_loop(self):
        t0 = time.time()
        period = 1.0 / self._hz
        while self._running:
            t = time.time() - t0
            self._on_hand_update(self._generate_frame(t))
            sleep_time = period - (time.time() - (t0 + t))
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _generate_frame(self, t: float) -> HandData:
        hand = HandData()
        hand.timestamp = time.time()
        hand.confidence = 1.0
        hand.handedness = "right"

        # Base wrist: ~35cm in front of camera, Aria frame (X=right, Y=down, Z=forward)
        base_x, base_y, base_z = 0.0, -0.1, 0.35

        if self._mode == "circle":
            r = 0.08
            wx = base_x + r * math.cos(2 * math.pi * t / 10.0)
            wy = base_y + r * math.sin(2 * math.pi * t / 10.0)
            wz = base_z
            pinch_closed = (t % 5.0) < 2.0
        elif self._mode == "sweep":
            wx = base_x + 0.12 * math.sin(2 * math.pi * t / 8.0)
            wy = base_y
            wz = base_z + 0.05 * math.sin(2 * math.pi * t / 4.0)
            pinch_closed = (t % 3.0) < 1.0
        else:
            wx, wy, wz = base_x, base_y, base_z
            pinch_closed = False

        landmarks = np.zeros((NUM_LANDMARKS, 3), dtype=np.float64)
        wrist_pos = np.array([wx, wy, wz])
        landmarks[WRIST] = wrist_pos  # index 5

        pinch_dist = 0.015 if pinch_closed else 0.06
        landmarks[THUMB_FINGERTIP] = wrist_pos + np.array([0.02, -0.05, pinch_dist / 2])
        landmarks[INDEX_FINGERTIP] = wrist_pos + np.array([-0.01, -0.06, -pinch_dist / 2])

        # Fill remaining (Aria Gen 2 order)
        offsets = {
            2:  [-0.005, -0.08, 0.0],     3:  [-0.015, -0.075, -0.01],
            4:  [-0.025, -0.068, -0.02],   6:  [0.025, -0.04, 0.0],
            7:  [0.03, -0.03, 0.0],        8:  [0.01, -0.06, 0.01],
            9:  [0.0, -0.07, 0.01],        10: [-0.005, -0.075, 0.01],
            11: [-0.005, -0.06, 0.0],      12: [-0.005, -0.07, 0.0],
            13: [-0.005, -0.075, 0.0],     14: [-0.015, -0.055, -0.01],
            15: [-0.015, -0.065, -0.01],   16: [-0.015, -0.07, -0.01],
            17: [-0.025, -0.05, -0.02],    18: [-0.025, -0.058, -0.02],
            19: [-0.025, -0.063, -0.02],   20: [0.0, -0.03, 0.0],
        }
        for idx, offset in offsets.items():
            landmarks[idx] = wrist_pos + np.array(offset)

        hand.landmarks = landmarks
        hand.wrist_position = wrist_pos.copy()
        hand.palm_position = wrist_pos + np.array([0.0, -0.03, 0.0])
        hand.palm_normal = np.array([0.0, 0.0, -1.0])
        hand.wrist_normal = np.array([0.0, 1.0, 0.0])
        return hand
