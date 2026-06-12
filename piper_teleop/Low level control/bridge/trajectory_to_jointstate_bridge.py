#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class TrajectoryToJointStateBridge(Node):
    def __init__(self):
        super().__init__("trajectory_to_jointstate_bridge")
        self.declare_parameter("input_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("output_topic", "/joint_ctrl_cmd")
        self.declare_parameter("max_publish_rate", 8.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.min_publish_period = 1.0 / max(0.1, float(self.get_parameter("max_publish_rate").value))
        self.last_publish_time = 0.0

        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.subscription = self.create_subscription(
            JointTrajectory,
            input_topic,
            self.trajectory_callback,
            10,
        )

        self.get_logger().info(
            f"Bridging JointTrajectory '{input_topic}' to JointState '{output_topic}' "
            f"at <= {1.0 / self.min_publish_period:.1f} Hz"
        )

    def trajectory_callback(self, trajectory):
        now = time.time()
        if now - self.last_publish_time < self.min_publish_period:
            return
        if not trajectory.points:
            return

        point = trajectory.points[-1]
        if len(point.positions) < len(trajectory.joint_names):
            self.get_logger().warn("Ignoring trajectory point with missing positions")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(trajectory.joint_names)
        msg.position = list(point.positions[: len(msg.name)])

        if len(point.velocities) >= len(msg.name):
            msg.velocity = list(point.velocities[: len(msg.name)])
        else:
            msg.velocity = [0.0] * len(msg.name)

        msg.effort = [math.nan] * len(msg.name)

        self.publisher.publish(msg)
        self.last_publish_time = now


def main():
    rclpy.init()
    node = TrajectoryToJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
