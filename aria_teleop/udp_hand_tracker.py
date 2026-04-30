"""
UDP Hand Tracker
================
Receives hand tracking data over UDP from aria_udp_streamer.py (running on
Linux/WSL where the Aria SDK is available).

This is the Windows-side counterpart that makes the Aria pipeline work
without needing the Aria SDK installed locally.

Protocol:
  JSON packets over UDP containing:
    - "landmarks": [[x,y,z], ...] (21 points, meters, device frame)
    - "wrist_pos": [x,y,z] (meters)
    - "confidence": float
    - "handedness": "left" or "right"
"""
import json
import socket
import time
import threading
import numpy as np
from typing import Callable, Optional

from aria_hand_tracker import HandData, NUM_LANDMARKS


class UDPHandTracker:
    """
    Receives hand tracking data from aria_udp_streamer.py over UDP.
    Same interface as AriaHandTracker and SimHandTracker.
    """

    def __init__(self, on_hand_update: Callable[[HandData], None],
                 bind_ip: str = "0.0.0.0",
                 bind_port: int = 5010):
        self._on_hand_update = on_hand_update
        self._bind_ip = bind_ip
        self._bind_port = bind_port
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0

    def start(self):
        """Start listening for UDP hand tracking packets."""
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        print(f"[udp_tracker] Listening on {self._bind_ip}:{self._bind_port}")
        print(f"[udp_tracker] Waiting for hand data from aria_udp_streamer.py...")

    def stop(self):
        """Stop the UDP receiver."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        print("[udp_tracker] Stopped.")

    @property
    def is_running(self) -> bool:
        return self._running

    def _recv_loop(self):
        """Background thread: receive UDP packets and fire callbacks."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self._bind_ip, self._bind_port))
        sock.settimeout(0.5)  # 500ms timeout for clean shutdown

        last_print = 0.0

        while self._running:
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[udp_tracker] Recv error: {e}")
                continue

            try:
                packet = json.loads(data.decode('utf-8'))
                hand = self._parse_packet(packet)
                if hand is not None:
                    self._frame_count += 1
                    self._on_hand_update(hand)

                    # Log periodically
                    now = time.time()
                    if now - last_print > 5.0:
                        last_print = now
                        print(f"[udp_tracker] Receiving from {addr[0]} "
                              f"({self._frame_count} frames total)")

            except Exception as e:
                print(f"[udp_tracker] Parse error: {e}")

        sock.close()

    def _parse_packet(self, packet: dict) -> Optional[HandData]:
        """Convert a JSON packet into a HandData object."""
        hand = HandData()
        hand.timestamp = packet.get("ts", time.time())

        # Landmarks
        landmarks_raw = packet.get("landmarks")
        if landmarks_raw and len(landmarks_raw) >= NUM_LANDMARKS:
            hand.landmarks = np.array(landmarks_raw[:NUM_LANDMARKS],
                                      dtype=np.float64)
            hand.wrist_position = hand.landmarks[0].copy()
        else:
            return None  # No usable data

        # Wrist position override
        wrist = packet.get("wrist_pos")
        if wrist:
            hand.wrist_position = np.array(wrist, dtype=np.float64)

        # Confidence and handedness
        hand.confidence = float(packet.get("confidence", 1.0))
        hand.handedness = str(packet.get("handedness", "right"))

        # Arm landmarks (Shoulder, Elbow)
        shoulder = packet.get("shoulder")
        if shoulder: hand.shoulder_position = np.array(shoulder, dtype=np.float64)
        
        elbow = packet.get("elbow")
        if elbow: hand.elbow_position = np.array(elbow, dtype=np.float64)

        return hand
