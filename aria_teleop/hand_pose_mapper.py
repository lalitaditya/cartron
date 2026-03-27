"""
Hand Pose Mapper
================
Maps raw Aria hand landmarks into robot control signals:
  - Wrist position → end-effector target (mm) in robot base frame
  - Pinch distance (thumb-index) → gripper open/close (0–0.035 m)
  - Wrist orientation → end-effector Euler angles (optional)

Coordinate convention:
  Aria device frame: X-right, Y-down, Z-forward (camera convention)
  Robot base frame:  X-forward, Y-left, Z-up (ROS convention)
"""
import math
import numpy as np
from typing import Optional, Tuple, List

# Aria Gen 2 landmark indices (different from MediaPipe!)
# See: projectaria_tools.core.mps.hand_tracking.HandLandmark
ARIA_THUMB_TIP = 0       # THUMB_FINGERTIP
ARIA_INDEX_TIP = 1       # INDEX_FINGERTIP
ARIA_MIDDLE_TIP = 2      # MIDDLE_FINGERTIP
ARIA_RING_TIP = 3        # RING_FINGERTIP
ARIA_PINKY_TIP = 4       # PINKY_FINGERTIP
ARIA_WRIST = 5
ARIA_THUMB_INTERMEDIATE = 6
ARIA_THUMB_DISTAL = 7
ARIA_INDEX_PROXIMAL = 8
ARIA_INDEX_INTERMEDIATE = 9
ARIA_INDEX_DISTAL = 10
ARIA_MIDDLE_PROXIMAL = 11
ARIA_MIDDLE_INTERMEDIATE = 12
ARIA_MIDDLE_DISTAL = 13
ARIA_RING_PROXIMAL = 14
ARIA_RING_INTERMEDIATE = 15
ARIA_RING_DISTAL = 16
ARIA_PINKY_PROXIMAL = 17
ARIA_PINKY_INTERMEDIATE = 18
ARIA_PINKY_DISTAL = 19
ARIA_PALM_CENTER = 20

# Aliases used by the mapper
WRIST = ARIA_WRIST
THUMB_TIP = ARIA_THUMB_TIP
INDEX_TIP = ARIA_INDEX_TIP
INDEX_MCP = ARIA_INDEX_PROXIMAL
MIDDLE_MCP = ARIA_MIDDLE_PROXIMAL

# ─── Tunable parameters ───────────────────────────────────────────────────────
# Pinch detection
PINCH_CLOSED_THRESHOLD = 0.025   # meters — below this = fully closed
PINCH_OPEN_THRESHOLD = 0.06     # meters — above this = fully open

# Workspace scaling: map hand movement range to arm workspace
# Hand workspace: ~30cm range of motion in front of camera
# Arm workspace: ~60cm range, so scale factor ≈ 2.0
WORKSPACE_SCALE = 2.0  # multiplier from hand-meters to arm-mm-delta

# Arm home position (mm) — EE position at neutral pose
ARM_HOME_MM = [56.128, 0.0, 213.266]

# Exponential smoothing factor (0 = no smoothing, 1 = frozen)
SMOOTHING_ALPHA = 0.3


class HandPoseMapper:
    """
    Converts Aria hand landmarks to robot control targets.

    First frame captures the "origin" position; subsequent frames produce
    relative deltas that are scaled into the arm workspace.
    """

    def __init__(self,
                 workspace_scale: float = WORKSPACE_SCALE,
                 arm_home_mm: Optional[List[float]] = None,
                 smoothing: float = SMOOTHING_ALPHA):
        self._workspace_scale = workspace_scale
        self._arm_home = list(arm_home_mm or ARM_HOME_MM)
        self._smoothing = smoothing

        # Calibration: first-frame origin
        self._origin: Optional[np.ndarray] = None
        self._origin_set = False

        # Smoothed output
        self._smooth_pos = np.array(self._arm_home, dtype=np.float64)
        self._smooth_grip = 0.0

    def reset_origin(self):
        """Force re-calibration of the hand origin on next frame."""
        self._origin = None
        self._origin_set = False
        self._smooth_pos = np.array(self._arm_home, dtype=np.float64)
        self._smooth_grip = 0.0

    def map_landmarks(self, landmarks: np.ndarray,
                      confidence: float = 1.0
                      ) -> Tuple[List[float], float, dict]:
        """
        [DEPRECATED/RELATIVE MODE] 
        Map 21-landmark array to (target_pos_mm, grip_val, debug_info).
        ... original relative logic ...
        """
        # (Keeping original map_landmarks for backward compatibility)
        return self._map_relative(landmarks, confidence)

    def _map_relative(self, landmarks, confidence):
        debug = {}
        wrist = landmarks[WRIST]
        if not self._origin_set:
            self._origin = wrist.copy()
            self._origin_set = True
        delta = wrist - self._origin
        dx_robot = delta[2] * self._workspace_scale * 1000.0
        dy_robot = -delta[0] * self._workspace_scale * 1000.0
        dz_robot = -delta[1] * self._workspace_scale * 1000.0
        target_raw = np.array([self._arm_home[0] + dx_robot, self._arm_home[1] + dy_robot, self._arm_home[2] + dz_robot])
        
        # Pinch
        pinch_dist = float(np.linalg.norm(landmarks[THUMB_TIP] - landmarks[INDEX_TIP]))
        grip_raw = 0.035 if pinch_dist <= PINCH_CLOSED_THRESHOLD else (0.0 if pinch_dist >= PINCH_OPEN_THRESHOLD else 0.035 * (1.0 - (pinch_dist - PINCH_CLOSED_THRESHOLD) / (PINCH_OPEN_THRESHOLD - PINCH_CLOSED_THRESHOLD)))
        
        # Smooth & Clamp
        self._smooth_pos = self._smoothing * self._smooth_pos + (1 - self._smoothing) * target_raw
        self._smooth_grip = self._smoothing * self._smooth_grip + (1 - self._smoothing) * grip_raw
        target = np.clip(self._smooth_pos, [-100, -300, 0], [500, 300, 450])
        return target.tolist(), self._smooth_grip, {"pinch_dist_m": pinch_dist}

    def map_arm_to_robot(self, landmarks: np.ndarray, 
                         shoulder: np.ndarray, 
                         elbow: np.ndarray,
                         confidence: float = 1.0) -> Tuple[List[float], float, dict]:
        """
        [ABSOLUTE/MIRROR MODE]
        Maps human arm (Shoulder -> Wrist) to robot base frame.
        """
        debug = {}
        wrist = landmarks[WRIST]
        
        # 1. Shoulder-relative vector (meters)
        delta = wrist - shoulder
        
        # 2. Coordinate transform: Aria → Robot
        # Aria: X-right, Y-down, Z-forward
        # Robot: X-forward, Y-left, Z-up
        dx_robot = delta[2] * self._workspace_scale * 1000.0
        dy_robot = -delta[0] * self._workspace_scale * 1000.0
        dz_robot = -delta[1] * self._workspace_scale * 1000.0
        
        # We add the transformed delta to the robot's shoulder height
        # Typically the Piper shoulder is ~100-200mm above the base.
        # We'll use the ARM_HOME_MM[0] (X) as a base forward offset.
        target_raw = np.array([
            self._arm_home[0] + dx_robot,
            self._arm_home[1] + dy_robot,
            self._arm_home[2] + dz_robot
        ])
        
        # 3. Pinch (same as relative)
        pinch_dist = float(np.linalg.norm(landmarks[THUMB_TIP] - landmarks[INDEX_TIP]))
        grip_raw = 0.035 if pinch_dist <= PINCH_CLOSED_THRESHOLD else (0.0 if pinch_dist >= PINCH_OPEN_THRESHOLD else 0.035 * (1.0 - (pinch_dist - PINCH_CLOSED_THRESHOLD) / (PINCH_OPEN_THRESHOLD - PINCH_CLOSED_THRESHOLD)))
        
        # 4. Smooth & Clamp
        self._smooth_pos = self._smoothing * self._smooth_pos + (1 - self._smoothing) * target_raw
        self._smooth_grip = self._smoothing * self._smooth_grip + (1 - self._smoothing) * grip_raw
        target = np.clip(self._smooth_pos, [-150, -350, 0], [550, 350, 500])
        
        return target.tolist(), self._smooth_grip, {"pinch_dist_m": pinch_dist, "arm_mode": "absolute"}

    def map_wrist_direct(self, wrist_pos: np.ndarray,
                         pinch_dist: float
                         ) -> Tuple[List[float], float]:
        """
        Simplified mapping from wrist position + pinch distance.
        For use when full landmarks aren't available.

        Args:
            wrist_pos: (3,) wrist position in device frame, meters
            pinch_dist: distance between thumb and index tips, meters

        Returns:
            target_pos_mm, grip_val
        """
        # Create synthetic landmarks with just wrist, thumb, index
        landmarks = np.zeros((21, 3), dtype=np.float64)
        landmarks[WRIST] = wrist_pos
        # Place thumb and index tips such that their distance equals pinch_dist
        landmarks[THUMB_TIP] = wrist_pos + np.array([0, 0, pinch_dist / 2])
        landmarks[INDEX_TIP] = wrist_pos - np.array([0, 0, pinch_dist / 2])

        target, grip, _ = self.map_landmarks(landmarks)
        return target, grip

    def get_wrist_orientation(self, landmarks: np.ndarray
                              ) -> Optional[List[float]]:
        """
        Estimate wrist orientation from hand landmarks.
        Returns [roll, pitch, yaw] in radians or None if not estimable.
        
        Uses the plane formed by WRIST → INDEX_MCP and WRIST → MIDDLE_MCP
        to estimate the hand's orientation relative to the robot frame.
        """
        try:
            wrist = landmarks[WRIST]
            index_mcp = landmarks[INDEX_MCP]
            middle_mcp = landmarks[MIDDLE_MCP]

            # Vectors along the hand
            v1 = index_mcp - wrist    # roughly along fingers
            v2 = middle_mcp - wrist

            # Palm normal via cross product
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                return None
            normal = normal / norm

            # Forward direction (finger direction)
            forward = v1 / (np.linalg.norm(v1) + 1e-8)

            # Lateral direction
            lateral = np.cross(normal, forward)
            lateral = lateral / (np.linalg.norm(lateral) + 1e-8)

            # Build rotation matrix (Aria frame → Robot frame transform applied)
            # Aria: X-right, Y-down, Z-forward → Robot: X-forward, Y-left, Z-up
            R = np.array([
                [forward[2], -forward[0], -forward[1]],
                [lateral[2], -lateral[0], -lateral[1]],
                [normal[2],  -normal[0],  -normal[1]],
            ])

            # Extract Euler angles (ZYX convention, same as PiperIK)
            if abs(R[2, 0]) < 1.0 - 1e-6:
                pitch = -math.asin(R[2, 0])
                cp = math.cos(pitch)
                roll = math.atan2(R[2, 1] / cp, R[2, 2] / cp)
                yaw = math.atan2(R[1, 0] / cp, R[0, 0] / cp)
            else:
                yaw = 0.0
                if R[2, 0] <= -1.0:
                    pitch = math.pi / 2
                    roll = math.atan2(R[0, 1], R[0, 2])
                else:
                    pitch = -math.pi / 2
                    roll = math.atan2(-R[0, 1], -R[0, 2])

            return [roll, pitch, yaw]
        except Exception:
            return None
