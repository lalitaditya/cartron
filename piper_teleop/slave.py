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

class RobotService(robot_teleop_pb2_grpc.RobotServiceServicer):
    def __init__(self, piper_interface):
        self.piper = piper_interface

    def SendJointState(self, request, context):
        try:
            # 1. Received Joint Values
            # request.joints is a list of floats (0.001 deg units)
            # request.gripper is a float (1000000 units ish)
            
            # 2. Safety Check (optional but recommended)
            # e.g. check if values are within limits
            
            # 3. Command the motors
            # Convert units? No, assuming raw units sent from Master match API expectations.
            # JointCtrl expects: j1, j2, j3, j4, j5, j6
            # Joints are integers in SDK?
            # piper_ctrl_joint.py uses: round(position[idx]*factor) -> int
            # Since master sent raw SDK integers, we can use them directly?
            # Master used: joint_msgs.joint_state.joint_1 (int)
            # Proto: repeated float joints
            # So we cast back to int.
            
            j = [int(val) for val in request.joints]
            gripper = int(request.gripper)

            # MUST call MotionCtrl_2 every frame to keep arm in active mode
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
            self.piper.JointCtrl(j[0], j[1], j[2], j[3], j[4], j[5])
            self.piper.GripperCtrl(abs(gripper), 2000, 0x01, 0)

        except Exception as e:
            print(f"Error processing command: {e}")
            
        return robot_teleop_pb2.Empty()

def main():
    import argparse
    import platform

    parser = argparse.ArgumentParser(description="Piper Slave Arm gRPC Server")
    parser.add_argument("--can-port", default=None,
                        help="CAN port (e.g. COM5 on Windows, can0 on Linux). "
                             "Defaults to COM5 on Windows, can0 on Linux.")
    parser.add_argument("--grpc-port", default="50051", help="gRPC listen port")
    args = parser.parse_args()

    is_windows = platform.system() == "Windows"

    if args.can_port is None:
        args.can_port = "COM5" if is_windows else "can0"

    print(f"Connecting to Slave Arm on {args.can_port}...")

    if is_windows:
        # Windows: use SLCAN (serial CAN) via CreateCanBus
        piper = C_PiperInterface_V2(args.can_port, can_auto_init=False)
        piper.CreateCanBus(args.can_port, bustype="slcan",
                           expected_bitrate=1000000, judge_flag=False)
    else:
        # Linux: use socketcan (default)
        piper = C_PiperInterface_V2(args.can_port)

    piper.ConnectPort()

    # Start gRPC Server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_teleop_pb2_grpc.add_RobotServiceServicer_to_server(RobotService(piper), server)

    server.add_insecure_port(f'[::]:{args.grpc_port}')
    server.start()
    print(f"Slave Server listening on port {args.grpc_port}")

    # Enable Motors
    print("Enabling Slave Arm...")
    while not piper.EnablePiper():
        time.sleep(0.1)
    print("Slave Arm enabled!")

    # Set speed/accel limits
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)

    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        print("\nStopping Slave...")
        piper.DisableArm(7)  # Safety disable

if __name__ == "__main__":
    main()
