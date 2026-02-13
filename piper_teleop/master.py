#!/usr/bin/env python3
import time
import grpc
import robot_teleop_pb2
import robot_teleop_pb2_grpc
from piper_sdk import C_PiperInterface_V2

def main():
    # 1. Connect to Master Arm (Passive Mode)
    # We DO NOT cal EnablePiper(), so motors stay off and arm is hand-guidable.
    can_port = "can0" 
    print(f"Connecting to Master Arm on {can_port}...")
    piper = C_PiperInterface_V2(can_port)
    piper.ConnectPort()
    
    # 2. Connect to gRPC Server (Slave)
    # Replace 'localhost' with Slave IP if running on different machines
    slave_ip = "localhost" 
    port = "50051"
    channel = grpc.insecure_channel(f'{slave_ip}:{port}')
    stub = robot_teleop_pb2_grpc.RobotServiceStub(channel)
    print(f"Connected to Slave at {slave_ip}:{port}")

    try:
        while True:
            # 3. Read Joint States (0.001 degrees -> radians for calculation, but lets stick to raw or agreed unit)
            # SDK GetArmJointMsgs returns structure.
            # joint_state.joint_1 is in 0.001 degrees. 
            # Example: 1000 = 1 degree.
            
            joint_msgs = piper.GetArmJointMsgs()
            gripper_msgs = piper.GetArmGripperMsgs()

            # Extract joints (order: 1 to 6)
            # We will send them as FLOAT values.
            # It's best to normalize or send as is. Let's send as is (0.001 deg) to match SDK expectations on other end.
            joints = [
                joint_msgs.joint_state.joint_1,
                joint_msgs.joint_state.joint_2,
                joint_msgs.joint_state.joint_3,
                joint_msgs.joint_state.joint_4,
                joint_msgs.joint_state.joint_5,
                joint_msgs.joint_state.joint_6,
            ]
            
            # Gripper: 0-100 ish? SDK usually maps it locally.
            # grippers_angle is in 1000000 units?
            # Let's read: gripper_state.grippers_angle
            gripper_val = gripper_msgs.gripper_state.grippers_angle

            # 4. Create gRPC Message
            request = robot_teleop_pb2.JointValues(
                joints=joints,
                gripper=float(gripper_val),
                enable=True # Keep slave enabled
            )

            # 5. Send
            try:
                stub.SendJointState(request)
                # print(f"Sent: {joints} | G: {gripper_val}")
            except grpc.RpcError as e:
                print(f"gRPC Error: {e.code()}")
            
            time.sleep(0.01) # 100Hz

    except KeyboardInterrupt:
        print("\nStopping Master...")

if __name__ == "__main__":
    main()
