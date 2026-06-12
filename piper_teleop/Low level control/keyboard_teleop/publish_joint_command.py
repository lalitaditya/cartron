#!/usr/bin/env python3

import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointCommandPublisher(Node):
    def __init__(self, args):
        super().__init__("piper_joint_command_test")
        self.args = args
        self.publisher = self.create_publisher(JointState, args.topic, 10)
        self.start_time = self.get_clock().now()
        self.done = False
        self.timer = self.create_timer(1.0 / args.rate, self.publish_command)
        self.get_logger().info(
            f"Publishing JointState command to {args.topic} for {args.duration:.2f}s"
        )

    def publish_command(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.args.duration:
            self.done = True
            self.timer.cancel()
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        msg.position = [
            self.args.joint1,
            self.args.joint2,
            self.args.joint3,
            self.args.joint4,
            self.args.joint5,
            self.args.joint6,
        ]
        msg.velocity = [self.args.speed] * 6
        msg.effort = [math.nan] * 6
        self.publisher.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish a direct Piper joint command as sensor_msgs/JointState."
    )
    parser.add_argument("--topic", default="/joint_ctrl_cmd")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--joint1", type=float, required=True)
    parser.add_argument("--joint2", type=float, required=True)
    parser.add_argument("--joint3", type=float, required=True)
    parser.add_argument("--joint4", type=float, required=True)
    parser.add_argument("--joint5", type=float, required=True)
    parser.add_argument("--joint6", type=float, required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = JointCommandPublisher(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
