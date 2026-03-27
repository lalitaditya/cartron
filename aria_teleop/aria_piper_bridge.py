"""
Aria -> Piper Bridge
====================
Main entry point for controlling the Piper X robotic arm using hand tracking
from Meta Aria Gen 2 glasses.

Pipeline:
  Aria glasses -> hand_pose_mapper.py -> PiperIK -> gRPC/CAN/ROS2

Input modes:
  --udp     Receive hand data over UDP from aria_udp_streamer.py (RECOMMENDED on Windows)
  --sim     Use simulated hand tracker (no Aria glasses needed)
  (default) Use Aria SDK directly (Linux/macOS only)

Usage:
  # RECOMMENDED: Receive from Linux/WSL via UDP (works on Windows):
  python aria_piper_bridge.py --udp --can --can-port COM5

  # Simulation mode (no hardware):
  python aria_piper_bridge.py --sim

  # Direct Aria SDK (Linux/macOS only):
  python aria_piper_bridge.py --grpc
"""
import argparse
import os
import sys
import time
import threading
import math
import socket
import json
import queue
import libusb_package
import usb.core
import usb.backend.libusb1

# ── Fix: Load libusb backend for GS_USB on Windows ──
try:
    import os
    import ctypes
    import libusb_package
    import usb.core
    import usb.backend.libusb1

    libusb_dll_path = str(libusb_package.get_library_path())
    if libusb_dll_path:
        # 1. Load the DLL directly into the process
        ctypes.CDLL(libusb_dll_path)
        # 2. Add to DLL search path for Python 3.8+
        libusb_dir = os.path.dirname(libusb_dll_path)
        if hasattr(os, 'add_dll_directory'):
            os.add_dll_directory(libusb_dir)
        # 3. Add to PATH for legacy lookups
        os.environ['PATH'] = libusb_dir + os.pathsep + os.environ['PATH']
        # 4. Explicitly set up the backend for pyusb
        # Note: the find_library lambda must handle the 'candidate' name argument
        usb.backend.libusb1.get_backend(find_library=lambda x: libusb_dll_path)
        print(f"[bridge] libusb backend loaded from {libusb_dir}")
except Exception as e:
    print(f"[bridge] libusb load warning: {e}")
from typing import Optional, List

import numpy as np

# ─── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARTRON_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, CARTRON_ROOT)
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(CARTRON_ROOT, "vr_teleop"))
sys.path.insert(0, os.path.join(CARTRON_ROOT, "piper_teleop"))

# ─── Import the IK solver from the existing VR bridge ─────────────────────────
from vr_piper_bridge import (
    PiperIK, NUM_JOINTS, RAD_TO_MDEG, JOINT_LIMITS, NEUTRAL_JOINTS,
    quat_to_euler
)

# ─── Import hand tracking components ──────────────────────────────────────────
from aria_hand_tracker import HandData, ARIA_SDK_AVAILABLE
from hand_pose_mapper import HandPoseMapper
from sim_hand_tracker import SimHandTracker
from udp_hand_tracker import UDPHandTracker

# ─── Optional imports ─────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    import grpc
    import robot_teleop_pb2
    import robot_teleop_pb2_grpc
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False

try:
    from piper_sdk import C_PiperInterface_V2
    import platform
    PIPER_SDK_AVAILABLE = True
except ImportError:
    PIPER_SDK_AVAILABLE = False


class AriaPiperBridge:
    """
    Bridges Aria hand tracking data to the Piper X robotic arm.
    
    Subscribes to hand data (from AriaHandTracker or SimHandTracker),
    maps landmarks to robot control targets, runs IK, and sends commands.
    """

    def __init__(self, args):
        self.args = args

        # ── IK Solver ──────────────────────────────────────────────────────
        self.ik = PiperIK()

        # ── Hand Pose Mapper ───────────────────────────────────────────────
        self.mapper = HandPoseMapper(
            workspace_scale=args.workspace_scale,
            smoothing=args.smoothing,
        )

        # ── Latest hand data (thread-safe) ─────────────────────────────────
        self._hand_lock = threading.Lock()
        self._latest_hand: Optional[HandData] = None
        self._hand_updated = False

        # ── Control output state ───────────────────────────────────────────
        self._current_joints = list(NEUTRAL_JOINTS)
        self._current_grip = 0.0

        # ── Stats ──────────────────────────────────────────────────────────
        self._frame_count = 0
        self._last_print = 0.0
        self._start_time = time.time()
        self._can_tx_thread: Optional[threading.Thread] = None
        self._running = False

        # ── Hand Tracker ───────────────────────────────────────────────────────
        if args.sim:
            self.tracker = SimHandTracker(
                on_hand_update=self._on_hand_update,
                mode=args.sim_mode,
                hz=30.0,
            )
            print("[bridge] Using SIMULATION hand tracker")
        elif args.udp:
            self.tracker = UDPHandTracker(
                on_hand_update=self._on_hand_update,
                bind_ip=args.udp_bind,
                bind_port=args.udp_port,
            )
            print(f"[bridge] Using UDP hand tracker (port {args.udp_port})")
            print(f"[bridge] Run aria_udp_streamer.py on Linux/WSL to feed data")
        else:
            if not ARIA_SDK_AVAILABLE:
                raise RuntimeError(
                    "Aria SDK not available on Windows.\n"
                    "Use one of these modes instead:\n"
                    "  --udp   Receive hand data from Linux/WSL via UDP\n"
                    "  --sim   Use simulated hand tracker for testing"
                )
            from aria_hand_tracker import AriaHandTracker
            self.tracker = AriaHandTracker(
                on_hand_update=self._on_hand_update,
                use_usb=not args.wifi,
            )
            print("[bridge] Using ARIA Gen 2 hand tracker (native)")

        # ── ROS2 ──────────────────────────────────────────────────────────
        self.ros_node = None
        if args.ros:
            if not ROS2_AVAILABLE:
                print("[bridge] WARNING: ROS2 not available, --ros disabled")
            else:
                rclpy.init()
                self.ros_node = Node('aria_piper_bridge')
                self.pose_pub = self.ros_node.create_publisher(
                    PoseStamped, '/piper/pose_goal', 10)
                self.joint_pub = self.ros_node.create_publisher(
                    JointState, '/joint_states', 10)
                print("[bridge] ROS2 publishers initialized")

        # ── gRPC ──────────────────────────────────────────────────────────
        self.grpc_stub = None
        self.grpc_channel = None
        self._grpc_lock = threading.Lock()
        self._grpc_busy = False
        self._last_grpc_joints = None
        self._last_grpc_grip = None
        if args.grpc:
            if not GRPC_AVAILABLE:
                print("[bridge] WARNING: gRPC not available, --grpc disabled")
            else:
                options = [('grpc.tcp_nodelay', 1)]
                self.grpc_channel = grpc.insecure_channel(
                    f"{args.grpc_host}:{args.grpc_port}", options=options)
                self.grpc_stub = robot_teleop_pb2_grpc.RobotServiceStub(
                    self.grpc_channel)
                from concurrent import futures
                self.executor = futures.ThreadPoolExecutor(max_workers=1)
                print(f"[bridge] gRPC -> {args.grpc_host}:{args.grpc_port}")

        # ── Direct CAN ────────────────────────────────────────────────────
        self.piper = None
        if args.can:
            if not PIPER_SDK_AVAILABLE:
                print("[bridge] WARNING: piper_sdk not found, --can disabled")
            else:
                self._init_can(args.can_port, args.can_bitrate)

    # ── Hand data callback (called from tracker thread) ────────────────────
    def _on_hand_update(self, hand: HandData):
        """Thread-safe callback: stores the latest hand tracking frame."""
        with self._hand_lock:
            self._latest_hand = hand
            self._hand_updated = True

    # ── CAN initialization ─────────────────────────────────────────────────
    def _init_can(self, port: str, bitrate: int):
        """Connect to physical arm via CAN bus."""
        print(f"[bridge] Selecting CAN interface...")
        # If port is an integer (e.g. '0'), use gs_usb. If 'COMx', use slcan.
        if port.isdigit() or port == "gs_usb":
            real_port = port if port.isdigit() else "0"
            print(f"[bridge] Connecting via GS_USB (Interface {real_port})...")
            self.piper = C_PiperInterface_V2(can_name=real_port, can_interface="gs_usb")
            self.piper.ConnectPort(can_init=True, bitrate=bitrate)
        else:
            print(f"[bridge] Connecting via SLCAN on {port}...")
            self.piper = C_PiperInterface_V2(can_name=port, can_interface="slcan")
            self.piper.ConnectPort(can_init=True, bitrate=bitrate)
        
        # After connect, give it a moment to stabilize
        time.sleep(1.0)

        # 3. Enable PC Control mode (Only do this once!)
        # 0x01: PC control mode, 0x01: Joint control
        self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        print("[bridge] Arm enabled and in PC control mode!")
        
        # 4. Start dedicated TX thread to prevent SLCAN buffer flood
        self._running = True
        self._can_tx_thread = threading.Thread(target=self._can_worker, daemon=True)
        self._can_tx_thread.start()

    def _can_worker(self):
        """Background thread: Sends commands to the bus at a steady pace."""
        print("[bridge] CAN TX thread started.")
        while self._running:
            if not self.piper: break
            
            with self._hand_lock:
                joints = list(self._current_joints)
                grip = self._current_grip
            
            try:
                j_mdeg = [round(joints[i] * RAD_TO_MDEG) for i in range(NUM_JOINTS)]
                val_grip = min(max(abs(round(grip * 1_000_000)), 0), 100000)
                
                # Send Joint Angles
                self.piper.JointCtrl(j_mdeg[0], j_mdeg[1], j_mdeg[2],
                                     j_mdeg[3], j_mdeg[4], j_mdeg[5])
                
                # Micro-delay: CRITICAL for slcan stability on Windows
                time.sleep(0.005) 
                
                # Send Gripper
                self.piper.GripperCtrl(val_grip, 1000, 0x01, 0)
                
            except Exception:
                pass # Suppress floods, main thread will catch some
                
            # Control rate: 50Hz is plenty for slcan
            time.sleep(0.015) 

    # ── Main control loop ──────────────────────────────────────────────────
    def run(self):
        """Start the hand tracker and run the control loop."""
        print("=" * 60)
        print("  Aria -> Piper X Teleoperation Bridge")
        print("=" * 60)
        mode_name = 'Simulation' if self.args.sim else ('UDP' if self.args.udp else 'Aria Gen 2')
        print(f"  Mode:    {mode_name}")
        print(f"  Output:  ", end="")
        outputs = []
        if self.ros_node: outputs.append("ROS2")
        if self.grpc_stub: outputs.append("gRPC")
        if self.piper: outputs.append("CAN")
        print(", ".join(outputs) if outputs else "None (dry run)")
        print(f"  IK:      6-DOF Jacobian pseudoinverse")
        print(f"  Scale:   {self.args.workspace_scale}x")
        print(f"  Smooth:  alpha={self.args.smoothing}")
        print("=" * 60)
        print("  Move your hand to control the arm.")
        print("  Pinch thumb+index to close the gripper.")
        print("  Press Ctrl+C to stop.")
        print("=" * 60)

        # Start the hand tracker
        self.tracker.start()

        try:
            while True:
                loop_start = time.perf_counter()

                # Spin ROS2 if active
                if self.ros_node:
                    rclpy.spin_once(self.ros_node, timeout_sec=0)

                # Get latest hand data
                hand = None
                with self._hand_lock:
                    if self._hand_updated:
                        hand = self._latest_hand
                        self._hand_updated = False

                if hand is None or hand.landmarks is None:
                    # No new data, sleep briefly
                    time.sleep(0.005)
                    continue

                # Skip low-confidence frames
                if hand.confidence < self.args.min_confidence:
                    continue

                # ── Map hand/arm → robot target ────────────────────────────
                if (hand.shoulder_position is not None and 
                    (self.args.mirror or self.args.udp)):
                    # Mirror Mode: Base target on human shoulder origin
                    target_mm, grip, debug = self.mapper.map_arm_to_robot(
                        hand.landmarks, 
                        hand.shoulder_position, 
                        hand.elbow_position, 
                        hand.confidence
                    )
                else:
                    # Legacy Relative Mode: Base target on wrist origin
                    target_mm, grip, debug = self.mapper.map_landmarks(
                        hand.landmarks, hand.confidence
                    )

                # ── Optional: orientation from hand landmarks ──────────────
                target_euler = None
                if self.args.ik_orientation:
                    euler = self.mapper.get_wrist_orientation(hand.landmarks)
                    if euler is not None:
                        target_euler = euler

                # ── IK solve ───────────────────────────────────────────────
                joints = self.ik.solve(
                    target_mm,
                    target_euler_rad=target_euler,
                    step_length=self.args.ik_step,
                    max_iter=self.args.ik_iter,
                    use_orientation=(target_euler is not None),
                )

                self._current_joints = joints
                self._current_grip = grip
                self._frame_count += 1

                # ── Send to outputs ────────────────────────────────────────
                if self.ros_node:
                    self._publish_ros(target_mm, joints, grip)

                if self.piper:
                    self._send_can(joints, grip)

                if self.grpc_stub:
                    self._send_grpc(joints, grip)

                # ── Logging (1Hz) ──────────────────────────────────────────
                now = time.time()
                if now - self._last_print > 1.0:
                    self._last_print = now
                    ee = self.ik.get_ee_pos(joints)
                    fps = self._frame_count / (now - self._start_time) if (now - self._start_time) > 0 else 0
                    pinch = debug.get('pinch_dist_m', 0) * 100  # cm

                    print(
                        f"[bridge] "
                        f"Target=({target_mm[0]:.0f},{target_mm[1]:.0f},{target_mm[2]:.0f})mm "
                        f"EE=({ee[0]:.0f},{ee[1]:.0f},{ee[2]:.0f})mm "
                        f"Grip={grip:.3f} "
                        f"Pinch={pinch:.1f}cm "
                        f"Conf={hand.confidence:.2f} "
                        f"{fps:.0f}fps"
                    )

                # ── Rate limiting (~50Hz max) ──────────────────────────────
                elapsed = time.perf_counter() - loop_start
                sleep_time = 0.02 - elapsed  # 50Hz target
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[bridge] Interrupted, shutting down...")
        finally:
            self._shutdown()

    # ── ROS2 publishing ────────────────────────────────────────────────────
    def _publish_ros(self, target_mm: List[float],
                     joints: List[float], grip: float):
        header = Header()
        header.stamp = self.ros_node.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        # Pose goal (mm → m for ROS)
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position.x = target_mm[0] / 1000.0
        pose_msg.pose.position.y = target_mm[1] / 1000.0
        pose_msg.pose.position.z = target_mm[2] / 1000.0
        pose_msg.pose.orientation.w = 1.0  # identity quaternion
        self.pose_pub.publish(pose_msg)

        # Joint state
        js_msg = JointState()
        js_msg.header = header
        js_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                       "joint7", "joint8"]
        js_msg.position = list(joints) + [grip, -grip]
        self.joint_pub.publish(js_msg)

    # ── CAN control ────────────────────────────────────────────────────────
    def _send_can(self, joints: List[float], grip: float):
        """
        No-op: Motion is now handled by the background _can_worker.
        This function just prints the status/feedback.
        """
        pass

    # ── gRPC control ───────────────────────────────────────────────────────
    def _send_grpc(self, joints: List[float], grip: float):
        if not self.grpc_stub:
            return
        j_mdeg = [round(v * RAD_TO_MDEG) for v in joints]
        gripper_val = round(grip * 1_000_000)

        with self._grpc_lock:
            self._last_grpc_joints = j_mdeg
            self._last_grpc_grip = gripper_val
            if self._grpc_busy:
                return
            self._grpc_busy = True

        self.executor.submit(self._grpc_worker)

    def _grpc_worker(self):
        while True:
            with self._grpc_lock:
                if self._last_grpc_joints is None:
                    self._grpc_busy = False
                    return
                joints = self._last_grpc_joints
                grip = self._last_grpc_grip
                self._last_grpc_joints = None

            try:
                request = robot_teleop_pb2.JointValues(
                    joints=[float(v) for v in joints],
                    gripper=float(grip),
                    enable=True,
                )
                self.grpc_stub.SendJointState(request, timeout=0.05)
            except Exception:
                pass

    # ── Shutdown ───────────────────────────────────────────────────────────
    def _shutdown(self):
        """Clean shutdown of all subsystems."""
        # Stop tracker
        try:
            self.tracker.stop()
        except Exception:
            pass

        # Disable arm
        self._running = False
        if self._can_tx_thread:
            self._can_tx_thread.join(timeout=1.0)

        if self.piper:
            try:
                self.piper.DisableArm(7)
                print("[bridge] Arm disabled (safety).")
            except Exception:
                pass

        # ROS2 cleanup
        if self.ros_node:
            self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        # gRPC cleanup
        if self.grpc_channel:
            self.grpc_channel.close()

        print("[bridge] Shutdown complete.")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Aria -> Piper X Teleoperation Bridge",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # RECOMMENDED for Windows: receive hand data from Linux/WSL:
  python aria_piper_bridge.py --udp --can --can-port COM5

  # Simulation (no hardware needed):
  python aria_piper_bridge.py --sim

  # Aria glasses + gRPC to remote arm (Linux/macOS only):
  python aria_piper_bridge.py --grpc --grpc-host 192.168.1.100
        """,
    )

    # Input source
    input_group = parser.add_argument_group("Input")
    input_group.add_argument("--sim", action="store_true",
                             help="Use simulated hand tracker (no Aria glasses)")
    input_group.add_argument("--sim-mode", default="circle",
                             choices=["circle", "sweep", "static"],
                             help="Simulation pattern (default: circle)")
    input_group.add_argument("--udp", action="store_true",
                             help="Receive hand data via UDP from aria_udp_streamer.py (RECOMMENDED on Windows)")
    input_group.add_argument("--udp-bind", default="0.0.0.0",
                             help="UDP bind address (default: 0.0.0.0)")
    input_group.add_argument("--udp-port", type=int, default=5010,
                             help="UDP listen port (default: 5010)")
    input_group.add_argument("--wifi", action="store_true",
                             help="Connect to Aria via Wi-Fi (default: USB, Linux/macOS only)")
    input_group.add_argument("--mirror", action="store_true",
                             help="Mirror mode: follow human arm from shoulder (recommends scale ~1.0)")

    # Output targets
    output_group = parser.add_argument_group("Output")
    output_group.add_argument("--ros", action="store_true",
                              help="Publish to ROS2 (/joint_states)")
    output_group.add_argument("--can", action="store_true",
                              help="Direct CAN control of physical arm")
    output_group.add_argument("--can-port", default="0", help="COM port (SLCAN) or Device Index (GS_USB)")
    output_group.add_argument("--can-bitrate", type=int, default=1000000,
                              help="CAN bitrate (default: 1000000)")
    output_group.add_argument("--grpc", action="store_true",
                              help="Send joint commands via gRPC")
    output_group.add_argument("--grpc-host", default="127.0.0.1")
    output_group.add_argument("--grpc-port", type=int, default=50051)

    # IK tuning
    ik_group = parser.add_argument_group("IK Parameters")
    ik_group.add_argument("--ik-step", type=float, default=0.5,
                          help="IK step length (default: 0.5)")
    ik_group.add_argument("--ik-iter", type=int, default=25,
                          help="Max IK iterations per frame (default: 25)")
    ik_group.add_argument("--ik-orientation", action="store_true",
                          help="Include wrist orientation in IK (6-DOF)")

    # Mapping tuning
    map_group = parser.add_argument_group("Mapping")
    map_group.add_argument("--workspace-scale", type=float, default=2.0,
                           help="Hand->arm workspace scale (default: 2.0)")
    map_group.add_argument("--smoothing", type=float, default=0.3,
                           help="Exponential smoothing alpha: 0=none, 0.9=heavy (default: 0.3)")
    map_group.add_argument("--min-confidence", type=float, default=0.3,
                           help="Min hand tracking confidence to accept (default: 0.3)")

    args = parser.parse_args()

    # Auto-detect CAN port
    if args.can and args.can_port is None:
        import platform as _plat
        args.can_port = "COM5" if _plat.system() == "Windows" else "can0"

    bridge = AriaPiperBridge(args)
    bridge.run()


if __name__ == "__main__":
    main()
