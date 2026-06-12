#!/usr/bin/env python3

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class HardcodedObjectPosePublisher(Node):
    def __init__(self, args):
        super().__init__("stage0_hardcoded_object_pose")
        self.args = args
        self.publisher = self.create_publisher(PoseStamped, args.output_topic, 10)
        self.timer = self.create_timer(1.0 / args.rate, self.publish_pose)
        self.get_logger().info(
            f"Publishing hardcoded object pose on {args.output_topic} in frame {args.frame_id}"
        )

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.pose.position.x = self.args.x
        msg.pose.position.y = self.args.y
        msg.pose.position.z = self.args.z
        qx, qy, qz, qw = quaternion_from_euler(
            self.args.roll,
            self.args.pitch,
            self.args.yaw,
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.publisher.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stage 0 perception: publish a known object pose to isolate planning."
    )
    parser.add_argument("--output-topic", default="/perception/object_pose_base")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--x", type=float, default=0.50)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.05)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=math.pi)
    parser.add_argument("--yaw", type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = HardcodedObjectPosePublisher(args)
    try:
        rclpy.spin(node)
    finally:    
        cp = math.cos(pitch * 0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
