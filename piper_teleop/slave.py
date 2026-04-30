#!/usr/bin/env python3
import os
import sys
import time
import grpc
from concurrent import futures
import robot_teleop_pb2
import robot_teleop_pb2_grpc

# Add parent dir (cartron/) so we can find piper_sdk
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from piper_sdk import C_PiperInterface_V2

import threading

class RobotService(robot_teleop_pb2_grpc.RobotServiceServicer):
    def __init__(self, piper_interface):
        self.piper = piper_interface
        self.lock = threading.Lock()
        self.latest_joints = None
        self.latest_gripper = None
        self.enabled = False
        self._frame_count = 0

    def SendJointState(self, request, context):
        """gRPC handler: just saves the values to state variables."""
        with self.lock:
            self.latest_joints = [int(val) for val in request.joints]
            self.latest_gripper = int(request.gripper)
            self.enabled = request.enable
        return robot_teleop_pb2.Empty()

def main():
    import argparse
    import platform

    parser = argparse.ArgumentParser(description="Piper Slave Arm gRPC Server")
    parser.add_argument("--can-port", default=None)
    parser.add_argument("--grpc-port", default="50051")
    args = parser.parse_args()

    is_windows = platform.system() == "Windows"
    if args.can_port is None:
        args.can_port = "COM5" if is_windows else "can0"

    print(f"Connecting to Slave Arm on {args.can_port}...")
    if is_windows:
        piper = C_PiperInterface_V2(args.can_port, can_auto_init=False)
        piper.CreateCanBus(args.can_port, bustype="slcan", expected_bitrate=1000000)
    else:
        piper = C_PiperInterface_V2(args.can_port)

    piper.ConnectPort()

    # Start gRPC Server
    service = RobotService(piper)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=5))
    robot_teleop_pb2_grpc.add_RobotServiceServicer_to_server(service, server)
    server.add_insecure_port(f'[::]:{args.grpc_port}')
    server.start()
    print(f"Slave Server listening on port {args.grpc_port}")

    # Enable Motors
    print("Enabling Slave Arm...")
    while not piper.EnablePiper():
        time.sleep(0.1)
    print("Slave Arm enabled!")

    # Fixed 100Hz Control Loop for physical arm
    print("Starting 100Hz State-Based Control Loop...")
    try:
        while True:
            start_time = time.perf_counter()
            
            # Fetch latest state from gRPC variables
            with service.lock:
                joints = service.latest_joints
                gripper = service.latest_gripper
                active = service.enabled

            if joints and active:
                # MUST call MotionCtrl_2 every iteration
                piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
                piper.JointCtrl(joints[0], joints[1], joints[2], 
                               joints[3], joints[4], joints[5])
                piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            else:
                # Keep arm active even if no commands are arriving
                piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)

            # Sleep to maintain ~100Hz
            elapsed = time.perf_counter() - start_time
            sleep_time = 0.01 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping Slave...")
        piper.DisableArm(7)
        server.stop(0)

if __name__ == "__main__":
    main()
