#!/usr/bin/env python3

import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


class JoyJointTeleop(Node):
    def __init__(self, args):
        super().__init__("piper_joy_joint_teleop")
        self.args = args
        self.command = [0.0] * 7
        self.axes = []
        self.buttons = []
        self.have_feedback = False
        self.deadman_was_pressed = False
        self.publisher = self.create_publisher(JointTrajectory, args.command_topic, 10)
        self.feedback_sub = self.create_subscription(
            JointState,
            args.feedback_topic,
            self.feedback_callback,
            10,
        )
        self.joy_sub = self.create_subscription(Joy, args.joy_topic, self.joy_callback, 10)
        self.timer = self.create_timer(1.0 / args.rate, self.publish_command)
        self.get_logger().info(
            f"Joystick teleop publishing JointTrajectory to {args.command_topic}; hold button {args.deadman_button}"
        )

    def feedback_callback(self, msg):
        positions_by_name = dict(zip(msg.name, msg.position))
        if not self.have_feedback or not self.deadman_pressed():
            for index, name in enumerate(JOINT_NAMES):
                if name in positions_by_name:
                    self.command[index] = positions_by_name[name]
            self.have_feedback = True

    def joy_callback(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

    def axis(self, index):
        if index < 0 or index >= len(self.axes):
            return 0.0
        value = self.axes[index]
        return 0.0 if abs(value) < self.args.deadzone else value

    def button(self, index):
        if index < 0 or index >= len(self.buttons):
            return 0
        return self.buttons[index]

    def deadman_pressed(self):
        return bool(self.button(self.args.deadman_button))

    def publish_command(self):
        if self.args.require_feedback and not self.have_feedback:
            return

        deadman = self.deadman_pressed()
        if not deadman:
            self.deadman_was_pressed = False
            return
        if not self.deadman_was_pressed:
            self.get_logger().info("Deadman pressed; streaming joint commands")
            self.deadman_was_pressed = True

        dt = 1.0 / self.args.rate
        axis_values = [
            self.axis(self.args.joint1_axis),
            self.axis(self.args.joint2_axis),
            self.axis(self.args.joint3_axis),
            self.axis(self.args.joint4_axis),
            self.axis(self.args.joint5_axis),
            self.axis(self.args.joint6_axis),
        ]
        for index, axis_value in enumerate(axis_values):
            self.command[index] += axis_value * self.args.joint_speed * dt
            self.command[index] = min(math.pi, max(-math.pi, self.command[index]))

        gripper_velocity = 0.0
        if self.button(self.args.gripper_open_button):
            gripper_velocity += self.args.gripper_speed
        if self.button(self.args.gripper_close_button):
            gripper_velocity -= self.args.gripper_speed
        self.command[6] = min(
            self.args.gripper_max,
            max(0.0, self.command[6] + gripper_velocity * dt),
        )

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = JOINT_NAMES[:]

        point = JointTrajectoryPoint()
        point.positions = self.command[:]
        point.velocities = [self.args.speed_percent] * 7
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e9 / self.args.rate)
        trajectory.points.append(point)

        self.publisher.publish(trajectory)


def parse_args():
    parser = argparse.ArgumentParser(description="Joystick Piper teleop through JointTrajectory bridge.")
    parser.add_argument("--joy-topic", default="/joy")
    parser.add_argument("--command-topic", default="/arm_controller/joint_trajectory")
    parser.add_argument("--feedback-topic", default="/joint_states_single")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--deadzone", type=float, default=0.08)
    parser.add_argument("--joint-speed", type=float, default=0.25)
    parser.add_argument("--speed-percent", type=float, default=20.0)
    parser.add_argument("--gripper-speed", type=float, default=0.04)
    parser.add_argument("--gripper-max", type=float, default=0.08)
    parser.add_argument("--gripper-effort", type=float, default=1.0)
    parser.add_argument("--deadman-button", type=int, default=0)
    parser.add_argument("--gripper-close-button", type=int, default=4)
    parser.add_argument("--gripper-open-button", type=int, default=5)
    parser.add_argument("--joint1-axis", type=int, default=0)
    parser.add_argument("--joint2-axis", type=int, default=1)
    parser.add_argument("--joint3-axis", type=int, default=3)
    parser.add_argument("--joint4-axis", type=int, default=4)
    parser.add_argument("--joint5-axis", type=int, default=6)
    parser.add_argument("--joint6-axis", type=int, default=7)
    parser.add_argument("--no-require-feedback", dest="require_feedback", action="store_false")
    parser.set_defaults(require_feedback=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = JoyJointTeleop(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
