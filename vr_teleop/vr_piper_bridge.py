"""
VR Piper Bridge — Cartesian Control Edition
=============================================
Receives VR controller Cartesian position via UDP, then:
  1. Publishes an EndPose (X,Y,Z,RX,RY,RZ) to gRPC for the physical robot.
  2. Uses a numerical IK (Jacobian-based) built on top of the SDK's FK to compute
     joint angles, and publishes them to /joint_states for RViz visualization.
"""
import argparse
import importlib.util
import json
import math
import os
import socket
import sys
import time
from typing import Dict, List, Optional

# Add parent dir (cartron/) to path so we can import piper_sdk
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARTRON_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, CARTRON_ROOT)

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

# Piper SDK Forward Kinematics — import directly to avoid CAN-bus deps in __init__
try:
    _fk_path = os.path.join(CARTRON_ROOT, "piper_sdk", "kinematics", "piper_fk.py")
    _spec = importlib.util.spec_from_file_location("piper_fk", _fk_path)
    _fk_mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_fk_mod)
    C_PiperForwardKinematics = _fk_mod.C_PiperForwardKinematics
    FK_AVAILABLE = True
except Exception as e:
    FK_AVAILABLE = False
    print(f"[bridge] Warning: could not load piper_fk.py: {e}. FK-based IK disabled.")

# ─── Joint limits from URDF ────────────────────────────────────────────────────
JOINT_LIMITS = [
    (-2.618,  2.618),   # joint1 — revolute
    ( 0.0,    3.14),    # joint2 — revolute
    (-2.967,  0.0),     # joint3 — revolute
    (-1.745,  1.745),   # joint4 — revolute
    (-1.22,   1.22),    # joint5 — revolute
    (-2.0944, 2.0944),  # joint6 — revolute
]
GRIPPER_LIMITS = (0.0, 0.035)  # joint7 — prismatic

# Number of revolute joints (excluding gripper)
NUM_JOINTS = 6

# ─── Workspace mapping ─────────────────────────────────────────────────────────
# The Piper arm's reachable workspace center (mm) — roughly where the
# end-effector sits when J1=0, J2≈1.0, J3≈-1.0  (neutral pose).
ARM_HOME_MM = [56.0, 0.0, 213.0]

# How many mm of arm movement per meter of VR controller movement.
# The arm's reach is ~300mm in any direction from home, and a typical
# VR hand sweep is ~0.5m, so 600 mm/m gives roughly 1:1 feel.
WORKSPACE_SCALE = 600.0  # mm per meter of VR movement

# ─── Numerical IK via Jacobian transpose ────────────────────────────────────────
class NumericalIK:
    """
    Jacobian-transpose IK for the Piper arm's position joints (J1-J3).
    J4-J6 (wrist) are set directly from controller orientation.
    """
    # Only solve for J1, J2, J3 (the position/shoulder/elbow joints)
    NUM_POS_JOINTS = 3

    def __init__(self):
        self.fk = C_PiperForwardKinematics(dh_is_offset=0x01)
        # Start at a neutral pose
        self.current_joints = [0.0] * NUM_JOINTS
        self.current_joints[1] = 1.0   # J2 mid-range [0, 3.14]
        self.current_joints[2] = -1.0  # J3 mid-range [-2.967, 0]

    def get_ee_position(self, joints: List[float]) -> List[float]:
        """Forward kinematics: joints (rad) -> end-effector [x, y, z] in mm."""
        fk_result = self.fk.CalFK(joints)
        ee = fk_result[-1]
        return [ee[0], ee[1], ee[2]]

    def compute_jacobian(self, joints: List[float], delta: float = 0.001) -> List[List[float]]:
        """
        Numerical Jacobian for position joints only (J1-J3).
        Returns a 3×3 matrix (3 Cartesian axes × 3 joints).
        """
        ee0 = self.get_ee_position(joints)
        jac = []
        for axis in range(3):  # x, y, z
            row = []
            for j in range(self.NUM_POS_JOINTS):  # only J1, J2, J3
                perturbed = joints.copy()
                perturbed[j] += delta
                ee1 = self.get_ee_position(perturbed)
                row.append((ee1[axis] - ee0[axis]) / delta)
            jac.append(row)
        return jac

    def solve(self, target_mm: List[float], wrist_angles: List[float],
              max_iter: int = 80, tol: float = 2.0) -> List[float]:
        """
        Iterative IK for position, with wrist angles set externally.
        target_mm: [x, y, z] in millimeters.
        wrist_angles: [J4, J5, J6] in radians (from controller orientation).
        Returns: list of 6 joint angles in radians.
        """
        joints = self.current_joints.copy()
        # Set wrist joints from controller orientation
        joints[3] = max(JOINT_LIMITS[3][0], min(JOINT_LIMITS[3][1], wrist_angles[0]))
        joints[4] = max(JOINT_LIMITS[4][0], min(JOINT_LIMITS[4][1], wrist_angles[1]))
        joints[5] = max(JOINT_LIMITS[5][0], min(JOINT_LIMITS[5][1], wrist_angles[2]))

        step_size = 0.00002  # Jacobian transpose step

        for _ in range(max_iter):
            ee = self.get_ee_position(joints)
            err = [target_mm[i] - ee[i] for i in range(3)]
            err_mag = math.sqrt(sum(e * e for e in err))

            if err_mag < tol:
                break

            jac = self.compute_jacobian(joints)
            for j in range(self.NUM_POS_JOINTS):  # only update J1-J3
                gradient = sum(jac[axis][j] * err[axis] for axis in range(3))
                joints[j] += step_size * gradient
                lo, hi = JOINT_LIMITS[j]
                joints[j] = max(lo, min(hi, joints[j]))

        self.current_joints = joints
        return joints


# ─── Bridge ──────────────────────────────────────────────────────────────────────
class PiperBridgeNode:
    def __init__(self, args):
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.bind_ip, args.udp_port))
        self.sock.settimeout(0.1)

        # Numerical IK (for RViz visualization)
        self.ik = NumericalIK() if FK_AVAILABLE else None

        # VR origin — captured from the first valid controller frame
        # so all movements become relative offsets from start position
        self.vr_origin = None

        # ROS2
        self.ros_node = None
        if args.ros and ROS2_AVAILABLE:
            rclpy.init()
            self.ros_node = Node('vr_piper_bridge')
            self.pose_pub = self.ros_node.create_publisher(PoseStamped, '/piper/pose_goal', 10)
            self.joint_pub = self.ros_node.create_publisher(JointState, '/joint_states', 10)
            print("[bridge] ROS2 publishers initialized.")

        # gRPC
        self.grpc_channel = None
        self.grpc_stub = None
        if args.grpc and GRPC_AVAILABLE:
            import grpc
            self.grpc_channel = grpc.insecure_channel(f"{args.grpc_host}:{args.grpc_port}")
            self.grpc_stub = robot_teleop_pb2_grpc.RobotServiceStub(self.grpc_channel)
            print(f"[bridge] gRPC client initialized to {args.grpc_host}:{args.grpc_port}")

        self._last_print = 0.0

    # ── Coordinate transform ──────────────────────────────────────────────────
    def transform_pose(self, pos, quat):
        """SteamVR (Y-up, right-handed) -> ROS (Z-up, right-handed)."""
        tx = -pos[2]  # SteamVR -Z -> ROS X (forward)
        ty = -pos[0]  # SteamVR -X -> ROS Y (left)
        tz =  pos[1]  # SteamVR  Y -> ROS Z (up)
        return [tx, ty, tz], quat

    def steamvr_quat_to_ros(self, quat):
        """Convert SteamVR quaternion (xyzw) to ROS frame (xyzw)."""
        # Same axis remapping as position
        qx, qy, qz, qw = quat
        return [-qz, -qx, qy, qw]

    # ── Main loop ─────────────────────────────────────────────────────────────
    def run(self):
        print(f"[bridge] Listening for UDP on {self.args.bind_ip}:{self.args.udp_port}...")
        print(f"[bridge] FK-IK: {'enabled' if self.ik else 'disabled (fallback to linear mapping)'}")
        try:
            while True:
                try:
                    data, addr = self.sock.recvfrom(65535)
                    msg = json.loads(data.decode('utf-8'))
                except (socket.timeout, json.JSONDecodeError):
                    continue

                controllers = msg.get("controllers", {})
                ctrl = controllers.get("right") or controllers.get("left")
                if not ctrl:
                    continue

                pos = ctrl.get("pos_m")
                quat = ctrl.get("quat_xyzw")
                buttons = ctrl.get("buttons", {})
                trigger = ctrl.get("trigger", 0.0)

                if pos and quat:
                    t_pos, t_quat = self.transform_pose(pos, quat)
                    r_quat = self.steamvr_quat_to_ros(quat)

                    if self.ros_node:
                        self.publish_ros(t_pos, r_quat, trigger, quat)
                    if self.grpc_stub:
                        self.send_grpc(t_pos, r_quat, trigger)

        except KeyboardInterrupt:
            print("\n[bridge] Interrupted, shutting down.")
        finally:
            if self.ros_node:
                self.ros_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            if self.grpc_channel:
                self.grpc_channel.close()

    # ── ROS2 publishing ───────────────────────────────────────────────────────
    def publish_ros(self, pos, quat, trigger, raw_quat=None):
        header = Header()
        header.stamp = self.ros_node.get_clock().now().to_msg()
        header.frame_id = self.args.frame_id

        # 1. Pose Goal (Cartesian target for debugging / MoveIt)
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

        # 2. JointState via FK-based IK (or fallback)
        js_msg = JointState()
        js_msg.header = header
        js_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                        "joint7", "joint8"]

        # ── Capture VR origin on first frame ──
        if self.vr_origin is None:
            self.vr_origin = [pos[0], pos[1], pos[2]]
            print(f"[bridge] VR origin captured: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})m")

        # ── Compute wrist Euler angles from controller quaternion ──
        roll, pitch, yaw = 0.0, 0.0, 0.0
        if raw_quat:
            qx, qy, qz, qw = raw_quat
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = 2.0 * (qw * qy - qz * qx)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        wrist_angles = [roll, pitch, yaw]

        # ── Map VR position → arm workspace ──
        # Relative offset from VR origin, scaled into arm workspace (mm)
        dx = (pos[0] - self.vr_origin[0]) * WORKSPACE_SCALE
        dy = (pos[1] - self.vr_origin[1]) * WORKSPACE_SCALE
        dz = (pos[2] - self.vr_origin[2]) * WORKSPACE_SCALE
        target_mm = [
            ARM_HOME_MM[0] + dx,
            ARM_HOME_MM[1] + dy,
            ARM_HOME_MM[2] + dz,
        ]

        if self.ik:
            joints = self.ik.solve(target_mm, wrist_angles)
        else:
            # Fallback: simple linear mapping
            scale = self.args.scale
            joints = [
                max(-2.618, min(2.618,   pos[0] * scale)),          # J1: base yaw
                max( 0.0,   min(3.14,   (-pos[2] + 1.0) * scale)), # J2: shoulder (Z inverted)
                max(-2.967, min(0.0,    (pos[1] - 1.0) * scale)),   # J3: elbow
                max(-1.745, min(1.745,  roll)),                     # J4: wrist roll
                max(-1.22,  min(1.22,   pitch)),                    # J5: wrist pitch
                max(-2.0944,min(2.0944, yaw)),                      # J6: wrist yaw
            ]

        grip_pos = max(0.0, min(0.035, float(trigger) * 0.035))
        js_msg.position = joints + [grip_pos, -grip_pos]
        self.joint_pub.publish(js_msg)

        # Log once per second
        now = time.time()
        if now - self._last_print > 1.0:
            self._last_print = now
            j_str = " ".join(f"J{i+1}={joints[i]:.3f}" for i in range(NUM_JOINTS))
            print(f"[bridge] pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})m -> {j_str} Grip={grip_pos:.3f}")

    # ── gRPC (Cartesian EndPoseCtrl) ──────────────────────────────────────────
    def send_grpc(self, pos, quat, trigger):
        """
        Send Cartesian end-pose to the physical robot via gRPC.
        The robot's firmware (via EndPoseCtrl) does the IK internally.
        Units: XYZ in micrometers (μm), RXRYRZ in millidegrees.
        """
        if not self.grpc_stub:
            return

        # Convert meters -> micrometers
        x_um = int(pos[0] * 1_000_000)
        y_um = int(pos[1] * 1_000_000)
        z_um = int(pos[2] * 1_000_000)

        # Convert quaternion to Euler angles (roll, pitch, yaw) in millidegrees
        # Using standard quaternion -> euler conversion
        qx, qy, qz, qw = quat
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convert to millidegrees
        rx_mdeg = int(math.degrees(roll) * 1000)
        ry_mdeg = int(math.degrees(pitch) * 1000)
        rz_mdeg = int(math.degrees(yaw) * 1000)

        gripper_val = int(float(trigger) * 1000)

        try:
            # Use EndPoseCtrl-style fields
            request = robot_teleop_pb2.JointValues(
                joints=[float(x_um), float(y_um), float(z_um),
                        float(rx_mdeg), float(ry_mdeg), float(rz_mdeg)],
                gripper=float(gripper_val),
                enable=True
            )
            self.grpc_stub.SendJointState(request)
        except Exception as e:
            now = time.time()
            if now - self._last_print > 5.0:
                print(f"[bridge] gRPC Send Failure: {e}")


# ─── Entry point ─────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="VR Piper Bridge — Cartesian Control")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--ros", action="store_true", help="Enable ROS2 publishing")
    parser.add_argument("--grpc", action="store_true", help="Enable gRPC client")
    parser.add_argument("--grpc-host", default="127.0.0.1")
    parser.add_argument("--grpc-port", type=int, default=50051)
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Fallback linear scale (only used when FK is unavailable)")
    parser.add_argument("--frame_id", default="base_link")
    args = parser.parse_args()

    bridge = PiperBridgeNode(args)
    bridge.run()


if __name__ == "__main__":
    main()
