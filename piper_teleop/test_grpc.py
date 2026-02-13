#!/usr/bin/env python3
import grpc
import robot_teleop_pb2
import robot_teleop_pb2_grpc
import time

def run():
    print("Connecting to Slave gRPC Server...")
    channel = grpc.insecure_channel('localhost:50051')
    stub = robot_teleop_pb2_grpc.RobotServiceStub(channel)
    
    # Send a few test messages
    for i in range(5):
        # Fake joint values (0 for now)
        request = robot_teleop_pb2.JointValues(
            joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            gripper=0.0,
            enable=True
        )
        try:
            stub.SendJointState(request)
            print(f"Sent Message {i+1}")
        except grpc.RpcError as e:
            print(f"Failed to send: {e.code()}")
        time.sleep(0.5)

if __name__ == '__main__':
    run()
