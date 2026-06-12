#!/usr/bin/env python3

import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class ServoApproachController(Node):
    def __init__(self, args):
        super().__init__("servo_approach_controller")
        self.args = args
        self.publisher = self.create_publisher(TwistStamped, args.twist_topic, 10)

    def run(self):
        start = time.time()
        while rclpy.ok() and time.time() - start < self.args.duration:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.args.frame_id
            msg.twist.linear.x = self.args.linear_x
            msg.twist.linear.y = self.args.linear_y
            msg.twist.linear.z = self.args.linear_z
            msg.twist.angular.x = self.args.angular_x
            msg.twist.angular.y = self.args.angular_y
            msg.twist.angular.z = self.args.angular_z
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0 / self.args.rate)

        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = self.args.frame_id
        self.publisher.publish(stop)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Small MoveIt Servo delta-twist publisher for approach/lift debugging."
    )
    parser.add_argument("--twist-topic", default="/servo_node/delta_twist_cmds")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--linear-z", type=float, default=0.01)
    parser.add_argument("--angular-x", type=float, default=0.0)
    parser.add_argument("--angular-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ServoApproachController(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
