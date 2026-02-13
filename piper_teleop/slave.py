#!/usr/bin/env python3
import time
import grpc
from concurrent import futures
import robot_teleop_pb2
import robot_teleop_pb2_grpc
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
            
            # Call SDK Control function
            # MotionCtrl_2 for speed/accel?
            # piper.MotionCtrl_2(0x01, 0x01, 50, 0x00) # Speed 50?
            
            self.piper.JointCtrl(j[0], j[1], j[2], j[3], j[4], j[5])
            
            # Gripper
            # GripperCtrl(val, speed, mode, force)
            # Mode 0x01 = position control
            self.piper.GripperCtrl(abs(gripper), 2000, 0x01, 0) # speed 2000

        except Exception as e:
            print(f"Error processing command: {e}")
            
        return robot_teleop_pb2.Empty()

def main():
    # 1. Connect to Slave Arm (Active Mode)
    can_port = "can1" # Assuming active arm is on can1 (or change to can0 if testing)
    print(f"Connecting to Slave Arm on {can_port}...")
    piper = C_PiperInterface_V2(can_port)
    piper.ConnectPort()
    
    # 2. Start gRPC Server (Start BEFORE enabling to allow connection)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_teleop_pb2_grpc.add_RobotServiceServicer_to_server(RobotService(piper), server)
    
    port = "50051"
    server.add_insecure_port(f'[::]:{port}')
    server.start()
    print(f"Slave Server listening on port {port}")

    # Enable Motors
    print("Enabling Slave Arm...")
    while not piper.EnablePiper():
        time.sleep(0.1)
    
    # Set speed/accel limits if needed
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        print("\nStopping Slave...")
        piper.DisableArm(7) # Safety disable

if __name__ == "__main__":
    main()
