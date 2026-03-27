"""
Aria Hand Tracker - Diagnostic Version (2 Windows)
==================================================
1. Raw Stream Window: Shows frames as fast as they arrive (tests Network).
2. Processed Window: Shows MediaPipe landmarks (tests CPU).
"""
import time
import sys
import os
import json
import socket
import threading
import numpy as np
import cv2
from typing import Callable, Optional

# ── Force FastDDS unicast (multicast is broken in WSL2 NAT) ──
_FASTDDS_XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fastdds_aria_unicast.xml")
if os.path.isfile(_FASTDDS_XML) and "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ:
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _FASTDDS_XML
    print(f"[fastdds] Using unicast profile: {_FASTDDS_XML}")

# ─── MediaPipe Initialization ──────────────────────────────────────────────────
try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False

# ─── Aria SDK Initialization ──────────────────────────────────────────────────
try:
    import aria.sdk as aria
    ARIA_SDK_AVAILABLE = True
except ImportError:
    ARIA_SDK_AVAILABLE = False

# ─── Constants ────────────────────────────────────────────────────────────────
NUM_LANDMARKS = 21
THUMB_FINGERTIP = 0
INDEX_FINGERTIP = 1
WRIST = 5
PALM_CENTER = 20
_MP_TO_ARIA = {0: 5, 4: 0, 8: 1, 12: 2, 16: 3, 20: 4, 1: 5, 2: 6, 3: 7, 5: 8, 6: 9, 7: 10, 9: 11, 10: 12, 11: 13, 13: 14, 14: 15, 15: 16, 17: 17, 18: 18, 19: 19}
_HAND_CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (0, 9), (9, 10), (10, 11), (11, 12), (0, 13), (13, 14), (14, 15), (15, 16), (0, 17), (17, 18), (18, 19), (19, 20)]

def _ensure_models():
    base = os.path.dirname(os.path.abspath(__file__))
    models = {
        "hand": (os.path.join(base, "models", "hand_landmarker.task"), "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"),
        "pose": (os.path.join(base, "models", "pose_landmarker_lite.task"), "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task")
    }
    for name, (path, url) in models.items():
        if not os.path.isfile(path):
            import urllib.request
            os.makedirs(os.path.dirname(path), exist_ok=True)
            print(f"[aria_tracker] Downloading {name} model...")
            urllib.request.urlretrieve(url, path)
    return models["hand"][0], models["pose"][0]

class HandData:
    __slots__ = ('landmarks', 'wrist_position', 'wrist_normal', 'palm_position', 'palm_normal', 'confidence', 'handedness', 'timestamp', 'shoulder_position', 'elbow_position')
    def __init__(self):
        self.landmarks = self.wrist_position = self.wrist_normal = self.palm_position = self.palm_normal = None
        self.shoulder_position = self.elbow_position = None
        self.confidence, self.handedness, self.timestamp = 0.0, "unknown", 0.0

def _result_to_hand_data(result, hand_idx: int) -> HandData:
    hand = HandData(); hand.timestamp = time.time()
    cat = result.handedness[hand_idx][0]
    hand.handedness = "right" if cat.category_name == "Left" else "left"
    hand.confidence = cat.score
    world_lms = result.hand_world_landmarks[hand_idx]
    lms = np.zeros((NUM_LANDMARKS, 3), dtype=np.float64)
    for mp_idx, aria_idx in _MP_TO_ARIA.items():
        if mp_idx < len(world_lms):
            lm = world_lms[mp_idx]
            lms[aria_idx] = [lm.x, lm.y, lm.z]
    hand.landmarks = lms
    hand.wrist_position = lms[5].copy()
    hand.palm_position = np.mean(lms[[8, 11, 14, 17]], axis=0)
    return hand

def _draw_landmarks_on_frame(frame_bgr, hand_res, pose_res=None):
    h, w = frame_bgr.shape[:2]
    # 1. Draw Arm (Pose)
    if pose_res and pose_res.pose_landmarks:
        lms = pose_res.pose_landmarks[0]
        # Landmarks: 11 (R Shoulder), 13 (R Elbow), 15 (R Wrist), 12 (L Shoulder), 14 (L Elbow), 16 (L Wrist)
        # Use a higher threshold for visibility if available, otherwise just coords
        pts = {i: (int(lms[i].x * w), int(lms[i].y * h)) for i in range(len(lms))}
        
        # Right Arm (Blue) - MediaPipe index 12, 14, 16 is often Right from Ego view
        # We'll draw both to be sure.
        for s, e, w_idx, color in [(12, 14, 16, (255, 0, 0)), (11, 13, 15, (0, 255, 0))]:
            if 0 <= pts[s][0] < w and 0 <= pts[s][1] < h and \
               0 <= pts[e][0] < w and 0 <= pts[e][1] < h:
                cv2.line(frame_bgr, pts[s], pts[e], color, 4)
                cv2.circle(frame_bgr, pts[s], 6, color, -1); cv2.putText(frame_bgr, "S", pts[s], 1, 1, (255,255,255), 1)
                cv2.circle(frame_bgr, pts[e], 6, color, -1); cv2.putText(frame_bgr, "E", pts[e], 1, 1, (255,255,255), 1)
            if 0 <= pts[e][0] < w and 0 <= pts[e][1] < h and \
               0 <= pts[w_idx][0] < w and 0 <= pts[w_idx][1] < h:
                cv2.line(frame_bgr, pts[e], pts[w_idx], color, 4)
                cv2.circle(frame_bgr, pts[w_idx], 6, color, -1); cv2.putText(frame_bgr, "W", pts[w_idx], 1, 1, (255,255,255), 1)

    # 2. Draw Hand (Hands)
    if hand_res:
        for i in range(len(hand_res.hand_landmarks)):
            pts = [(int(lm.x * w), int(lm.y * h)) for lm in hand_res.hand_landmarks[i]]
            color = (0, 255, 0) if hand_res.handedness[i][0].category_name == "Left" else (255, 0, 0)
            for p in pts: cv2.circle(frame_bgr, p, 4, color, -1)
            for a, b in _HAND_CONNECTIONS:
                if a < len(pts) and b < len(pts): cv2.line(frame_bgr, pts[a], pts[b], color, 2)
    return frame_bgr

class StreamObserver:
    def __init__(self, on_hand_update: Callable[[HandData], None]):
        self._on_hand_update = on_hand_update
        self._running = True
        self._lock = threading.Lock()
        
        # Data references
        self._latest_raw = None
        self._latest_processed = None # landmarks + metadata
        self._latest_result = None
        
        # Stats
        self._cb_count = 0
        self._cb_fps = 0.0
        self._proc_count = 0
        self._proc_fps = 0.0
        self._last_stats_time = time.time()

        if MEDIAPIPE_AVAILABLE:
            h_model, p_model = _ensure_models()
            h_opts = mp_vision.HandLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=h_model),
                num_hands=2, running_mode=mp_vision.RunningMode.VIDEO,
                min_hand_detection_confidence=0.5, min_tracking_confidence=0.5,
            )
            p_opts = mp_vision.PoseLandmarkerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=p_model),
                running_mode=mp_vision.RunningMode.VIDEO,
                min_pose_detection_confidence=0.5, min_tracking_confidence=0.5,
            )
            self._hand_landmarker = mp_vision.HandLandmarker.create_from_options(h_opts)
            self._pose_landmarker = mp_vision.PoseLandmarker.create_from_options(p_opts)
            self._worker = threading.Thread(target=self._processing_worker, daemon=True)
            self._worker.start()
        else: self._hand_landmarker = self._pose_landmarker = None

    def _processing_worker(self):
        frame_ts_ms = 0
        while self._running:
            raw = None
            with self._lock:
                if self._latest_raw is not None:
                    raw = self._latest_raw
                    self._latest_raw = None # Clear so we only process fresh frames
            if raw is None: time.sleep(0.001); continue
            
            try:
                # 1. Prepare frame
                frame = np.rot90(raw, -1)
                res = cv2.resize(frame, (640, 640))
                rgb = cv2.cvtColor(res, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
                frame_ts_ms += 33
                
                # 2. Dual-Inference
                h_res = self._hand_landmarker.detect_for_video(mp_img, frame_ts_ms)
                p_res = self._pose_landmarker.detect_for_video(mp_img, frame_ts_ms)
                
                # 3. Combine Data & Render
                vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                vis = _draw_landmarks_on_frame(vis, h_res, p_res)
                
                if h_res and h_res.hand_world_landmarks:
                    for i in range(len(h_res.hand_world_landmarks)):
                        hand = _result_to_hand_data(h_res, i)
                        # Attach arm info from Pose (Right side = idx 11, 13, 15)
                        if p_res and p_res.pose_world_landmarks:
                            p_world = p_res.pose_world_landmarks[0]
                            if hand.handedness == "right":
                                hand.shoulder_position = [p_world[12].x, p_world[12].y, p_world[12].z]
                                hand.elbow_position = [p_world[14].x, p_world[14].y, p_world[14].z]
                            else:
                                hand.shoulder_position = [p_world[11].x, p_world[11].y, p_world[11].z]
                                hand.elbow_position = [p_world[13].x, p_world[13].y, p_world[13].z]
                        self._on_hand_update(hand)
                
                self._latest_processed = vis
                self._latest_result = (h_res, p_res)
                self._proc_count += 1
            except Exception as e: print(f"Proc Error: {e}")

    def on_image_received(self, image: np.ndarray, record) -> None:
        if record.camera_id != aria.CameraId.Rgb: return
        with self._lock:
            self._latest_raw = image
            self._cb_count += 1
        
        # Update Stats every 1s
        now = time.time()
        if now - self._last_stats_time >= 1.0:
            dt = now - self._last_stats_time
            self._cb_fps = self._cb_count / dt
            self._proc_fps = self._proc_count / dt
            self._cb_count = 0
            self._proc_count = 0
            self._last_stats_time = now

    def on_imu_received(self, samples, idx): pass
    def on_magneto_received(self, sample): pass
    def on_baro_received(self, sample): pass
    def close(self):
        self._running = False
        try:
            if hasattr(self, '_worker') and self._worker.is_alive():
                self._worker.join(timeout=1.0)
            if hasattr(self, '_hand_landmarker') and self._hand_landmarker: 
                self._hand_landmarker.close()
            if hasattr(self, '_pose_landmarker') and self._pose_landmarker: 
                self._pose_landmarker.close()
        except Exception as e: print(f"Close Error: {e}")

class AriaHandTracker:
    def __init__(self, on_hand_update, udp_target=None):
        self._on_hand_update = on_hand_update
        self._udp_target, self._udp_sock = udp_target, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if udp_target else None
        self._device = self._sm = self._sc = self._obs = None

    def start(self):
        self._dc = aria.DeviceClient(); cfg = aria.DeviceClientConfig(); cfg.device_serial = "1WM103500C1272"
        self._dc.set_client_config(cfg); self._device = self._dc.connect()
        self._sm = self._device.streaming_manager; self._sc = self._sm.streaming_client
        
        # ── Fix: Clear any previous session ──
        try:
            self._sm.stop_streaming()
            time.sleep(1)
        except Exception: pass

        scfg = aria.StreamingConfig(); scfg.profile_name = "profile18"; scfg.security_options.use_ephemeral_certs = True
        self._sm.streaming_config = scfg; self._sm.start_streaming()
        sub = self._sc.subscription_config; sub.subscriber_data_type = aria.StreamingDataType.Rgb
        sub.security_options.use_ephemeral_certs = True; self._sc.subscription_config = sub
        self._obs = StreamObserver(self._on_hand_update)
        self._sc.set_streaming_client_observer(self._obs); self._sc.subscribe()
        print("[tracker] Streaming active.")

    def send_udp(self, hand):
        if self._udp_sock and self._udp_target:
            msg = {
                "type": "hand_tracking",
                "landmarks": hand.landmarks.tolist(),
                "handedness": hand.handedness,
                "confidence": hand.confidence,
                "shoulder": hand.shoulder_position if hand.shoulder_position else None,
                "elbow": hand.elbow_position if hand.elbow_position else None
            }
            self._udp_sock.sendto(json.dumps(msg).encode(), self._udp_target)

    def stop(self):
        if self._obs: self._obs.close()
        # Remove observer from client before stopping stream
        if self._sc:
            try:
                self._sc.set_streaming_client_observer(None)
                self._sc.unsubscribe()
            except Exception: pass
        if self._sm: 
            try: self._sm.stop_streaming()
            except Exception: pass
        if self._dc: 
            try: self._dc.disconnect(self._device)
            except Exception: pass
        print("[tracker] Stopped.")

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--udp-ip", default="127.0.0.1")
    p.add_argument("--viz", action="store_true") # restored --viz flag
    args = p.parse_args()
    tracker = AriaHandTracker(lambda h: tracker.send_udp(h), (args.udp_ip, 5010))
    try:
        tracker.start()
        while True:
            if tracker._obs:
                raw = tracker._obs._latest_raw
                proc = tracker._obs._latest_processed
                
                if raw is not None:
                    # 1. Raw Stream Window
                    raw_vis = np.rot90(raw, -1).copy()
                    cv2.putText(raw_vis, f"NETWORK (IN): {tracker._obs._cb_fps:.1f} FPS", (20, 50), 
                                cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 255), 2)
                    cv2.imshow("1. Aria Raw Stream (WiFi Test)", cv2.resize(raw_vis, (640, 640)))
                
                if proc is not None:
                    # 2. Processed Window
                    cv2.putText(proc, f"CPU (MP): {tracker._obs._proc_fps:.1f} FPS", (20, 50), 
                                cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)
                    
                    # Also draw the 3D result if available
                    if tracker._obs._latest_result:
                        h_res, p_res = tracker._obs._latest_result
                        proc = _draw_landmarks_on_frame(proc, h_res, p_res)
                        
                    cv2.imshow("2. Processed Output (CPU Test)", proc)
                
                if cv2.waitKey(1) & 0xFF == ord('q'): break
            time.sleep(0.01)
    except KeyboardInterrupt: pass
    finally: tracker.stop()
