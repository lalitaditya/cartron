"""
VR Piper Bridge — Analytical Jacobian IK
=========================================
Receives VR controller pose via UDP, then:
  1. Computes joint angles via analytical Jacobian pseudoinverse IK
     (ported from kineval IK — CSCI 5551).
  2. Publishes JointState to /joint_states for RViz visualization.
  3. Optionally sends Cartesian EndPose to gRPC for the physical robot.
"""
import argparse
import time
import socket
import json
import math
import os
import sys
import platform
import threading
from concurrent import futures
from typing import List, Optional, Tuple, Dict, Any

import numpy as np

# Add parent dir (cartron/) to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARTRON_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, CARTRON_ROOT)
# Add piper_teleop/ for gRPC protobuf imports
sys.path.insert(0, os.path.join(CARTRON_ROOT, "piper_teleop"))

# Optional ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# gRPC imports
try:
    import robot_teleop_pb2
    import robot_teleop_pb2_grpc
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False

# Piper SDK (for direct CAN control)
try:
    from piper_sdk import C_PiperInterface_V2
    import platform
    PIPER_SDK_AVAILABLE = True
except ImportError:
    PIPER_SDK_AVAILABLE = False

# ─── Piper DH Parameters (from piper_sdk/kinematics/piper_fk.py, 2° offset) ──
DH_A     = [0.0, 0.0, 285.03, -21.98, 0.0, 0.0]
DH_ALPHA = [0.0, -math.pi/2, 0.0, math.pi/2, -math.pi/2, math.pi/2]
DH_THETA_OFFSET = [0.0, -math.pi*172.22/180, -102.78/180*math.pi, 0.0, 0.0, 0.0]
DH_D     = [123.0, 0.0, 0.0, 250.75, 0.0, 91.0]

# ─── Joint limits from URDF ────────────────────────────────────────────────────
JOINT_LIMITS = [
    (-2.618,  2.618),   # joint1
    ( 0.0,    3.14),    # joint2
    (-2.967,  0.0),     # joint3
    (-1.745,  1.745),   # joint4
    (-1.22,   1.22),    # joint5
    (-2.0944, 2.0944),  # joint6
]
NUM_JOINTS = 6
NEUTRAL_JOINTS = [0.0, 1.0, -1.0, 0.0, 0.0, 0.0]

# SDK unit conversion: radians → millidegrees (0.001°)
# factor = 1000 * 180 / pi = 57295.7795
RAD_TO_MDEG = 57295.7795

# ─── Workspace mapping ─────────────────────────────────────────────────────────
ARM_HOME_MM = [56.128, 0.0, 213.266]   # EE position at neutral pose
WORKSPACE_SCALE = 600.0                 # mm per meter of VR movement

# ─── Matrix Utilities (ported from kineval_matrix.js) ──────────────────────────

def mat4_identity():
    return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]

def mat4_multiply(A, B):
    """4×4 matrix multiply — unrolled for speed (same as kineval_matrix.js)."""
    return [
        [A[0][0]*B[0][0]+A[0][1]*B[1][0]+A[0][2]*B[2][0]+A[0][3]*B[3][0],
         A[0][0]*B[0][1]+A[0][1]*B[1][1]+A[0][2]*B[2][1]+A[0][3]*B[3][1],
         A[0][0]*B[0][2]+A[0][1]*B[1][2]+A[0][2]*B[2][2]+A[0][3]*B[3][2],
         A[0][0]*B[0][3]+A[0][1]*B[1][3]+A[0][2]*B[2][3]+A[0][3]*B[3][3]],
        [A[1][0]*B[0][0]+A[1][1]*B[1][0]+A[1][2]*B[2][0]+A[1][3]*B[3][0],
         A[1][0]*B[0][1]+A[1][1]*B[1][1]+A[1][2]*B[2][1]+A[1][3]*B[3][1],
         A[1][0]*B[0][2]+A[1][1]*B[1][2]+A[1][2]*B[2][2]+A[1][3]*B[3][2],
         A[1][0]*B[0][3]+A[1][1]*B[1][3]+A[1][2]*B[2][3]+A[1][3]*B[3][3]],
        [A[2][0]*B[0][0]+A[2][1]*B[1][0]+A[2][2]*B[2][0]+A[2][3]*B[3][0],
         A[2][0]*B[0][1]+A[2][1]*B[1][1]+A[2][2]*B[2][1]+A[2][3]*B[3][1],
         A[2][0]*B[0][2]+A[2][1]*B[1][2]+A[2][2]*B[2][2]+A[2][3]*B[3][2],
         A[2][0]*B[0][3]+A[2][1]*B[1][3]+A[2][2]*B[2][3]+A[2][3]*B[3][3]],
        [A[3][0]*B[0][0]+A[3][1]*B[1][0]+A[3][2]*B[2][0]+A[3][3]*B[3][0],
         A[3][0]*B[0][1]+A[3][1]*B[1][1]+A[3][2]*B[2][1]+A[3][3]*B[3][1],
         A[3][0]*B[0][2]+A[3][1]*B[1][2]+A[3][2]*B[2][2]+A[3][3]*B[3][2],
         A[3][0]*B[0][3]+A[3][1]*B[1][3]+A[3][2]*B[2][3]+A[3][3]*B[3][3]],
    ]

def vector_cross(a, b):
    """Cross product — ported from kineval_matrix.js vector_cross."""
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def rotation_matrix_to_euler(R):
    """
    3×3 rotation matrix → [roll, pitch, yaw] radians (ZYX convention).
    Ported from kineval_inverse_kinematics.js rotationMatrixToEulerAngles.
    """
    if abs(R[2][0]) < 1.0 - 1e-6:
        pitch = -math.asin(R[2][0])
        cp = math.cos(pitch)
        roll  = math.atan2(R[2][1]/cp, R[2][2]/cp)
        yaw   = math.atan2(R[1][0]/cp, R[0][0]/cp)
    else:
        yaw = 0.0
        if R[2][0] <= -1.0:
            pitch = math.pi / 2
            roll  = math.atan2(R[0][1], R[0][2])
        else:
            pitch = -math.pi / 2
            roll  = math.atan2(-R[0][1], -R[0][2])
    return [roll, pitch, yaw]

def quat_to_euler(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) → [roll, pitch, yaw] radians (ZYX)."""
    roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
    yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return [roll, pitch, yaw]

def dh_transform(alpha, a, theta, d):
    """Single DH link transform (4×4) — same as piper_fk.py __LinkTransformtion."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return [
        [ct,    -st,   0.0,  a      ],
        [st*ca,  ct*ca,-sa,  -sa * d],
        [st*sa,  ct*sa, ca,   ca * d],
        [0.0,    0.0,   0.0,  1.0   ],
    ]


# ─── Analytical Jacobian IK (ported from kineval_inverse_kinematics.js) ───────

class PiperIK:
    """
    Analytical Jacobian IK for the Piper 6-DOF arm.

    Algorithm (from kineval iterateIK):
      1. FK → cumulative transforms T_0 … T_06
      2. Jacobian: for each revolute joint i,
           linear  col = z_i × (p_ee − p_i)
           angular col = z_i
      3. Error dx = [pos_error; orient_error]   (6×1)
      4. dq = J⁺ · dx                          (pseudoinverse)
      5. q += α · dq, clamped to limits
    """

    def __init__(self):
        self.current_joints = list(NEUTRAL_JOINTS)

    # ── Forward kinematics ────────────────────────────────────────────────
    def compute_fk(self, joints: List[float]):
        """Return 7 cumulative 4×4 transforms  [T_base, T_01, … T_06]."""
        frames = [mat4_identity()]
        for i in range(NUM_JOINTS):
            theta = joints[i] + DH_THETA_OFFSET[i]
            T_local = dh_transform(DH_ALPHA[i], DH_A[i], theta, DH_D[i])
            frames.append(mat4_multiply(frames[-1], T_local))
        return frames

    def get_ee_pos(self, joints):
        T = self.compute_fk(joints)[6]
        return [T[0][3], T[1][3], T[2][3]]

    # ── Analytical Jacobian ───────────────────────────────────────────────
    def build_jacobian(self, frames):
        """
        Build 6×6 analytical Jacobian.
        Ported from kineval iterateIK: walk each joint, compute
          axis   = z-column of frame[i]      (joint axis in world)
          origin = translation of frame[i]   (joint position in world)
          linear = axis × (ee − origin)
          angular = axis
        """
        ee = [frames[6][0][3], frames[6][1][3], frames[6][2][3]]
        cols = []
        for i in range(NUM_JOINTS):
            # In this DH convention, joint i's rotation axis is the z-column
            # of frames[i+1] (the frame AFTER the DH transform that includes
            # both the joint rotation and the alpha twist).
            T = frames[i + 1]
            # Joint axis in world frame: z-column of rotation matrix
            axis   = [T[0][2], T[1][2], T[2][2]]
            # Joint origin in world frame: translation column
            origin = [T[0][3], T[1][3], T[2][3]]
            diff   = [ee[k] - origin[k] for k in range(3)]
            linear = vector_cross(axis, diff)
            cols.append(linear + axis)          # 6-element column

        # Transpose col-list → 6×6 row-major
        return [[cols[j][i] for j in range(NUM_JOINTS)] for i in range(6)]

    # ── IK solver ─────────────────────────────────────────────────────────
    # Damping factor for damped least-squares (prevents divergence near singularities)
    DLS_LAMBDA = 2.0
    # Orientation weight: scales orientation error relative to position (mm vs rad).
    # Reduced to 30 to allow position priority during difficult reaches.
    ORIENT_WEIGHT = 30.0

    @staticmethod
    def _wrap_angle(a):
        """Wrap angle to [-pi, pi]."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def solve(self, target_pos_mm, target_euler_rad=None,
              step_length=0.5, max_iter=25, pos_tol=2.0,
              use_orientation=True, orient_weight=None):
        """
        Iterative IK using analytical Jacobian + damped least-squares.

        Uses DLS: dq = Jᵀ (J Jᵀ + λ²I)⁻¹ dx
        Orientation error is weighted and angle-wrapped to prevent divergence.
        orient_weight: per-call override for orientation scaling (default: self.ORIENT_WEIGHT).
        """
        joints = self.current_joints.copy()
        if target_euler_rad is None:
            use_orientation = False

        ow = orient_weight if orient_weight is not None else self.ORIENT_WEIGHT
        lam2 = self.DLS_LAMBDA ** 2

        for _ in range(max_iter):
            frames = self.compute_fk(joints)
            T_ee = frames[6]
            ee_pos = [T_ee[0][3], T_ee[1][3], T_ee[2][3]]

            # Position error (mm)
            pos_err = [target_pos_mm[k] - ee_pos[k] for k in range(3)]
            if math.sqrt(sum(e*e for e in pos_err)) < pos_tol:
                break

            # Orientation error (wrapped to [-pi,pi], then weighted)
            if use_orientation:
                R = [[T_ee[r][c] for c in range(3)] for r in range(3)]
                ee_euler = rotation_matrix_to_euler(R)
                orient_err = [self._wrap_angle(target_euler_rad[k] - ee_euler[k])
                              * ow for k in range(3)]
            else:
                orient_err = [0.0, 0.0, 0.0]

            dx = np.array(pos_err + orient_err, dtype=np.float64)

            # Damped least-squares: dq = Jᵀ (J Jᵀ + λ²I)⁻¹ dx
            J  = np.array(self.build_jacobian(frames), dtype=np.float64)
            JJt = J @ J.T + lam2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, dx)

            # Clamp joint velocity to prevent divergence / "teleporting"
            MAX_DQ = 0.15
            for j in range(NUM_JOINTS):
                dq_val = step_length * float(dq[j])
                dq_val = max(-MAX_DQ, min(MAX_DQ, dq_val))
                joints[j] += dq_val
                lo, hi = JOINT_LIMITS[j]
                joints[j] = max(lo, min(hi, joints[j]))

        self.current_joints = joints
        return joints

    # ── TCP-aware IK ──────────────────────────────────────────────────────
    GRIPPER_TCP_OFFSET = 100.0  # mm from link6 to gripper contact point

    def get_tcp_pos(self, joints):
        """Compute the gripper TCP (finger contact point) in world frame."""
        frames = self.compute_fk(joints)
        T_ee = frames[6]
        ee = [T_ee[0][3], T_ee[1][3], T_ee[2][3]]
        z_axis = [T_ee[0][2], T_ee[1][2], T_ee[2][2]]
        return [ee[i] + self.GRIPPER_TCP_OFFSET * z_axis[i] for i in range(3)]

    def solve_tcp(self, target_tcp_mm, step_length=0.1, max_iter=20,
                  tcp_tol=5.0, outer_iter=10):
        """
        Solve IK so the gripper TCP lands at `target_tcp_mm`.

        Iteratively adjusts the EE (link6) target until the gripper fingertips
        converge to the desired world position.
        """
        # Initial guess: EE target = TCP target (will be corrected)
        ee_target = list(target_tcp_mm)

        for i in range(outer_iter):
            joints = self.solve(
                ee_target,
                target_euler_rad=None,
                step_length=step_length,
                max_iter=max_iter,
                use_orientation=False
            )
            tcp_pos = self.get_tcp_pos(joints)
            tcp_err = [target_tcp_mm[k] - tcp_pos[k] for k in range(3)]
            tcp_dist = math.sqrt(sum(e*e for e in tcp_err))

            if tcp_dist < tcp_tol:
                break

            # Adjust EE target by the TCP error
            ee_target = [ee_target[k] + tcp_err[k] for k in range(3)]

        return joints, tcp_pos


# ─── Bridge ──────────────────────────────────────────────────────────────────────

class PiperBridgeNode:
    def __init__(self, args):
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.bind_ip, args.udp_port))
        self.sock.settimeout(0.1)

        self.ik = PiperIK()

        # VR origin — captured on first valid frame
        self.vr_origin = None

        # ROS2
        self.ros_node = None
        if args.ros and ROS2_AVAILABLE:
            rclpy.init()
            self.ros_node = Node('vr_piper_bridge')
            self.pose_pub = self.ros_node.create_publisher(PoseStamped, '/piper/pose_goal', 10)
            self.joint_pub = self.ros_node.create_publisher(JointState, '/joint_states', 10)
            # Subscribe to detected object position (from object_detector.py)
            from geometry_msgs.msg import PointStamped
            self._detected_obj_pos = None
            self._detected_obj_time = 0.0
            self.ros_node.create_subscription(
                PointStamped, '/detected_object/position',
                self._on_detected_object, 10)
            print("[bridge] ROS2 publishers initialized.")
            print("[bridge] Subscribed to /detected_object/position")

        # gRPC
        self.grpc_channel = None
        self.grpc_stub = None
        if args.grpc and GRPC_AVAILABLE:
            import grpc
            # Use TCP_NODELAY to disable Nagle's algorithm (critical for real-time)
            options = [('grpc.tcp_nodelay', 1)]
            self.grpc_channel = grpc.insecure_channel(f"{args.grpc_host}:{args.grpc_port}", 
                                                    options=options)
            self.grpc_stub = robot_teleop_pb2_grpc.RobotServiceStub(self.grpc_channel)
            # Thread pooling for non-blocking gRPC sends
            self.executor = futures.ThreadPoolExecutor(max_workers=1)
            self._last_grpc_joints = None
            self._last_grpc_grip = None
            self._grpc_lock = threading.Lock()
            print(f"[bridge] gRPC client initialized to {args.grpc_host}:{args.grpc_port}")

        # Direct CAN control (physical arm)
        self.piper = None
        if args.can:
            if not PIPER_SDK_AVAILABLE:
                print("[bridge] ERROR: piper_sdk not found, --can disabled.")
            else:
                self._init_can(args.can_port)

        self._last_print = 0.0

    # ── Detected object callback ────────────────────────────────────────────
    def _on_detected_object(self, msg):
        """Handle detected object position from object_detector.py."""
        # Convert from meters (ROS) to mm (IK)
        # The detector publishes in camera_optical_frame
        # For Phase 1, we use a simple mapping:
        # camera X → workspace Y (lateral), camera Y → workspace Z (vertical)
        # camera Z → workspace X (forward/depth from camera)
        # This is approximate — proper TF transform needed for Phase 2
        x_mm = msg.point.z * 1000.0   # depth → forward
        y_mm = -msg.point.x * 1000.0  # camera X → lateral (flipped)
        z_mm = -msg.point.y * 1000.0  # camera Y → vertical (flipped)

        # Offset to workspace frame (camera is on the arm, not at origin)
        # For now, use the raw detection as a relative offset from camera
        # and add to a base workspace position
        # This will be replaced with proper TF in Phase 2
        self._detected_obj_pos = [
            300.0 + x_mm,   # base forward + detected offset
            -100.0 + y_mm,  # base lateral + detected offset
            max(20.0, z_mm) # clamp to at least 20mm above ground
        ]
        self._detected_obj_time = time.time()

    # ── Coordinate transform ──────────────────────────────────────────────
    def transform_pose(self, pos, quat):
        """SteamVR (Y-up) → ROS (Z-up)."""
        tx = -pos[2]   # SteamVR -Z → ROS X
        ty = -pos[0]   # SteamVR -X → ROS Y
        tz =  pos[1]   # SteamVR  Y → ROS Z
        return [tx, ty, tz], quat

    def steamvr_quat_to_ros(self, quat):
        """SteamVR quaternion (xyzw) → ROS frame (xyzw)."""
        qx, qy, qz, qw = quat
        return [-qz, -qx, qy, qw]

    # ── CAN initialization ─────────────────────────────────────────────────
    def _init_can(self, can_port):
        """Connect to the physical arm via CAN bus and enable motors."""
        is_windows = platform.system() == "Windows"
        print(f"[bridge] Connecting to arm via CAN on {can_port}...")

        try:
            if is_windows:
                # Windows: SLCAN (serial CAN adapter)
                self.piper = C_PiperInterface_V2(can_port, can_auto_init=False)
                self.piper.CreateCanBus(can_port, bustype="slcan",
                                       expected_bitrate=1000000, judge_flag=False)
            else:
                # Linux: socketcan
                self.piper = C_PiperInterface_V2(can_port)

            self.piper.ConnectPort()
            print("[bridge] CAN connected. Enabling arm motors...")

            # Enable with timeout
            t0 = time.time()
            while not self.piper.EnablePiper():
                if time.time() - t0 > 10.0:
                    print("[bridge] WARNING: Arm enable timed out after 10s. "
                          "Continuing anyway — arm may not respond.")
                    break
                time.sleep(0.01)
            else:
                print("[bridge] Arm enabled!")

        except Exception as e:
            print(f"[bridge] CAN init failed: {e}")
            self.piper = None

    # ── Main loop ─────────────────────────────────────────────────────────
    def run(self):
        print(f"[bridge] Listening for UDP on {self.args.bind_ip}:{self.args.udp_port}...")
        print("[bridge] IK: analytical Jacobian + pseudoinverse (6-DOF)")
        try:
            while True:
                # Staged Auto-Grasp State Machine (TCP-Aware v6)
                if self.args.auto_grasp:
                    if not hasattr(self, '_auto_grasp_state'):
                        # Seed with a stable "Crane" pose
                        self.ik.current_joints = [0.0, 0.6, -0.8, 0.0, -1.1, 0.0]
                        self._auto_grasp_state = "APPROACH"
                        self._state_start_time = time.time()
                        # Fallback object position if no detection
                        self._obj_pos = [400.0, -100.0, 50.0]
                        print(f"[bridge] Auto-Grasp Started (TCP-aware)")
                        print(f"[bridge]    Default obj: X={self._obj_pos[0]:.0f} Y={self._obj_pos[1]:.0f} Z={self._obj_pos[2]:.0f} mm")
                        print(f"[bridge]    Listening for /detected_object/position...")

                    # Use detected position if available (within last 5s)
                    if (self._detected_obj_pos is not None and
                            time.time() - self._detected_obj_time < 5.0):
                        self._obj_pos = self._detected_obj_pos

                    obj = self._obj_pos
                    grip_val = 0.0
                    import math
                    
                    if self._auto_grasp_state == "APPROACH":
                        # Hover 100mm above the object
                        tcp_target = [obj[0], obj[1], obj[2] + 100.0]
                    elif self._auto_grasp_state == "DIVE":
                        # Directly at the object
                        tcp_target = [obj[0], obj[1], obj[2]]
                    elif self._auto_grasp_state == "GRASP":
                        tcp_target = [obj[0], obj[1], obj[2]]
                        grip_val = 0.035
                    elif self._auto_grasp_state == "LIFT":
                        tcp_target = [obj[0], obj[1], obj[2] + 100.0]
                        grip_val = 0.035
                    else:
                        tcp_target = [obj[0], obj[1], obj[2] + 100.0]

                    # TCP-aware IK: targets the gripper fingers, not link6
                    joints, tcp_pos = self.ik.solve_tcp(
                        tcp_target,
                        step_length=0.1,
                        max_iter=20,
                        tcp_tol=5.0,
                        outer_iter=8
                    )

                    # Distance from gripper TCP to target
                    tcp_dist = math.sqrt(sum((tcp_target[i] - tcp_pos[i])**2 for i in range(3)))

                    now = time.time()
                    if self._auto_grasp_state == "APPROACH" and tcp_dist < 15.0:
                        self._auto_grasp_state = "DIVE"
                        print(f"[bridge] -> DIVE (TCP_Dist={tcp_dist:.1f}mm)")
                        print(f"[bridge]    Gripper at: X={tcp_pos[0]:.1f} Y={tcp_pos[1]:.1f} Z={tcp_pos[2]:.1f}")
                    elif self._auto_grasp_state == "DIVE" and tcp_dist < 15.0:
                        self._auto_grasp_state = "GRASP"
                        self._state_start_time = now
                        print(f"[bridge] -> GRASP (TCP_Dist={tcp_dist:.1f}mm)")
                        print(f"[bridge]    Gripper at: X={tcp_pos[0]:.1f} Y={tcp_pos[1]:.1f} Z={tcp_pos[2]:.1f}")
                        print(f"[bridge]    Object at:  X={obj[0]:.1f} Y={obj[1]:.1f} Z={obj[2]:.1f}")
                    elif self._auto_grasp_state == "GRASP" and (now - self._state_start_time) > 2.0:
                        self._auto_grasp_state = "LIFT"
                        print("[bridge] -> LIFT (Gripper closed)")
                    elif self._auto_grasp_state == "LIFT" and tcp_dist < 15.0:
                        self._auto_grasp_state = "DONE"
                        print("[bridge] Auto-Grasp Sequence COMPLETE.")

                    if self._auto_grasp_state == "DONE":
                        # Keep publishing joint states so camera TF stays alive
                        if self.ros_node:
                            ros_pos = [tcp_target[0]/1000.0, tcp_target[1]/1000.0, tcp_target[2]/1000.0]
                            ros_quat = [0.0, -0.7071, 0.0, 0.7071]
                            self.publish_ros(ros_pos, ros_quat, joints, grip_val)
                            rclpy.spin_once(self.ros_node, timeout_sec=0)
                        time.sleep(0.05)
                        continue

                    # Logging
                    if now - self._last_print > 1.0:
                        self._last_print = now
                        ee = self.ik.get_ee_pos(joints)
                        print(f"[bridge] [AUTO:{self._auto_grasp_state}] TCP_Dist={tcp_dist:.1f}mm Grip={grip_val:.3f} | EE=({ee[0]:.0f},{ee[1]:.0f},{ee[2]:.0f}) TCP=({tcp_pos[0]:.0f},{tcp_pos[1]:.0f},{tcp_pos[2]:.0f})")

                    if self.ros_node:
                        ros_pos = [tcp_target[0]/1000.0, tcp_target[1]/1000.0, tcp_target[2]/1000.0]
                        ros_quat = [0.0, -0.7071, 0.0, 0.7071]
                        self.publish_ros(ros_pos, ros_quat, joints, grip_val)
                        rclpy.spin_once(self.ros_node, timeout_sec=0)

                    if self.piper:
                        self.send_can(joints, grip_val)
                    if self.grpc_stub:
                        self.send_grpc(joints, grip_val)
                    
                    time.sleep(0.01)
                    continue

                if self.ros_node:
                    rclpy.spin_once(self.ros_node, timeout_sec=0)

                # Drain the socket buffer to get ONLY the latest packet

                if msg is None:
                    # No new packet received in this tick, wait a bit
                    time.sleep(0.001)
                    continue

                controllers = msg.get("controllers", {})
                ctrl = controllers.get("right") or controllers.get("left")
                if not ctrl:
                    continue

                pos  = ctrl.get("pos_m")
                quat = ctrl.get("quat_xyzw")
                trigger = ctrl.get("trigger", 0.0)

                if pos and quat:
                    t_pos, _ = self.transform_pose(pos, quat)
                    r_quat = self.steamvr_quat_to_ros(quat)

                    # Compute IK once, share between ROS + CAN + gRPC
                    joints, grip_pos = self.compute_ik(t_pos, r_quat, trigger)

                    if self.ros_node:
                        self.publish_ros(t_pos, r_quat, joints, grip_pos)
                    if self.piper:
                        self.send_can(joints, grip_pos)
                    if self.grpc_stub:
                        self.send_grpc(joints, grip_pos)

        except KeyboardInterrupt:
            print("\n[bridge] Interrupted, shutting down.")
        finally:
            # Safety: disable arm first
            if self.piper:
                try:
                    self.piper.DisableArm(7)
                    print("[bridge] Arm disabled (safety).")
                except Exception:
                    pass
            if self.ros_node:
                self.ros_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            if self.grpc_channel:
                self.grpc_channel.close()

    # ── IK computation (shared between ROS and gRPC) ──────────────────────
    def compute_ik(self, pos, quat, trigger):
        """Compute IK joint angles and gripper position from VR controller pose."""
        # Capture VR origin on first frame
        if self.vr_origin is None:
            self.vr_origin = list(pos)
            print(f"[bridge] VR origin captured: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})m")

        # Target position: relative offset from VR origin → scaled into arm frame (mm)
        dx = (pos[0] - self.vr_origin[0]) * WORKSPACE_SCALE
        dy = (pos[1] - self.vr_origin[1]) * WORKSPACE_SCALE
        dz = (pos[2] - self.vr_origin[2]) * WORKSPACE_SCALE
        target_pos_mm = [ARM_HOME_MM[0]+dx, ARM_HOME_MM[1]+dy, ARM_HOME_MM[2]+dz]

        # Target orientation: Euler angles from ROS-frame quaternion
        target_euler = quat_to_euler(quat[0], quat[1], quat[2], quat[3])

        # IK solve
        joints = self.ik.solve(
            target_pos_mm,
            target_euler_rad=target_euler,
            step_length=self.args.ik_step,
            max_iter=self.args.ik_iter,
            use_orientation=self.args.ik_orientation,
        )

        grip_pos = max(0.0, min(0.035, float(trigger) * 0.035))

        # Log once per second
        now = time.time()
        if now - self._last_print > 1.0:
            self._last_print = now
            j_str = " ".join(f"J{i+1}={joints[i]:.3f}" for i in range(NUM_JOINTS))
            ee = self.ik.get_ee_pos(joints)
            print(f"[bridge] target=({target_pos_mm[0]:.0f},{target_pos_mm[1]:.0f},{target_pos_mm[2]:.0f})mm "
                  f"ee=({ee[0]:.0f},{ee[1]:.0f},{ee[2]:.0f})mm → {j_str} Grip={grip_pos:.3f}")

        return joints, grip_pos

    # ── ROS2 publishing ───────────────────────────────────────────────────
    def publish_ros(self, pos, quat, joints, grip_pos):
        header = Header()
        header.stamp = self.ros_node.get_clock().now().to_msg()
        header.frame_id = self.args.frame_id

        # 1. Pose goal (for debugging / MoveIt)
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(pose_msg)

        # 2. Publish joint state
        js_msg = JointState()
        js_msg.header = header
        js_msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6",
                       "joint7","joint8"]
        js_msg.position = joints + [grip_pos, -grip_pos]
        self.joint_pub.publish(js_msg)

    # ── Direct CAN control (physical arm) ──────────────────────────────────
    def send_can(self, joints, grip_pos):
        """Send IK joint angles directly to the physical arm via CAN.
        Calls MotionCtrl_2 every iteration (required by SDK).
        Converts radians → millidegrees (SDK unit = 0.001°).
        """
        if not self.piper:
            return
        # Convert joint angles: radians → millidegrees
        j_mdeg = [round(joints[i] * RAD_TO_MDEG) for i in range(NUM_JOINTS)]

        # Gripper: grip_pos is 0–0.035m, SDK expects ×1e6 units
        gripper_val = abs(round(grip_pos * 1_000_000))

        try:
            # MUST call MotionCtrl_2 every iteration to keep arm in active mode
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
            self.piper.JointCtrl(j_mdeg[0], j_mdeg[1], j_mdeg[2],
                                 j_mdeg[3], j_mdeg[4], j_mdeg[5])
            self.piper.GripperCtrl(gripper_val, 1000, 0x01, 0)
        except Exception as e:
            now = time.time()
            if now - self._last_print > 5.0:
                print(f"[bridge] CAN send error: {e}")

    # ── gRPC (joint control, background threaded) ─────────────────────────
    def send_grpc(self, joints, grip_pos):
        """Queue latest joint values for background send without buildup."""
        if not self.grpc_stub:
            return
        
        # Convert to SDK units (millidegrees)
        j_mdeg = [round(v * RAD_TO_MDEG) for v in joints]
        gripper_val = round(grip_pos * 1_000_000)

        # Update latest data.
        with self._grpc_lock:
            self._last_grpc_joints = j_mdeg
            self._last_grpc_grip = gripper_val
            
            # If a worker is already running, it will pick up these fresh values
            # when it finishes. No need to submit another task.
            if getattr(self, '_grpc_busy', False):
                return
            self._grpc_busy = True

        self.executor.submit(self._grpc_worker)

    def _grpc_worker(self):
        """Thread worker to perform the actual gRPC call."""
        while True:
            with self._grpc_lock:
                if self._last_grpc_joints is None:
                    self._grpc_busy = False
                    return
                joints = self._last_grpc_joints
                grip = self._last_grpc_grip
                self._last_grpc_joints = None # Consume the data

            import robot_teleop_pb2
            try:
                request = robot_teleop_pb2.JointValues(
                    joints=[float(v) for v in joints],
                    gripper=float(grip),
                    enable=True
                )
                # Physical network timeout
                self.grpc_stub.SendJointState(request, timeout=0.05)
            except Exception:
                pass
            
            # After sending, we loop once more to see if fresh data 
            # arrived while we were busy. If not, we exit and clear 'busy'.


# ─── Entry point ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="VR Piper Bridge — Analytical Jacobian IK")
    parser.add_argument("--bind-ip",   default="0.0.0.0")
    parser.add_argument("--udp-port",  type=int, default=5005)
    parser.add_argument("--ros",       action="store_true", help="Enable ROS2 publishing")
    parser.add_argument("--can",       action="store_true",
                        help="Enable direct CAN control of physical arm")
    parser.add_argument("--can-port",  default=None,
                        help="CAN port (COM5 on Windows, can0 on Linux). Auto-detected.")
    parser.add_argument("--grpc",      action="store_true", help="Enable gRPC client")
    parser.add_argument("--grpc-host", default="127.0.0.1")
    parser.add_argument("--grpc-port", type=int, default=50051)
    parser.add_argument("--frame_id",  default="base_link")
    # IK tuning
    parser.add_argument("--ik-step",   type=float, default=0.5,
                        help="IK step length α (default 0.5)")
    parser.add_argument("--ik-iter",   type=int, default=25,
                        help="Max IK iterations per frame (default 25)")
    parser.add_argument("--ik-orientation", action="store_true",
                        help="Include orientation in IK error (6-DOF)")
    parser.add_argument("--auto-grasp", action="store_true",
                        help="Bypass VR input and center arm on the test cube")
    args = parser.parse_args()

    # Auto-detect CAN port if not specified
    if args.can and args.can_port is None:
        import platform as _plat
        args.can_port = "COM5" if _plat.system() == "Windows" else "can0"

    bridge = PiperBridgeNode(args)
    bridge.run()


if __name__ == "__main__":
    main()
