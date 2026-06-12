#!/usr/bin/env python3

import argparse
import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


class KeyboardJointTeleop(Node):
    def __init__(self, args):
        super().__init__("piper_keyboard_joint_teleop")
        self.args = args
        self.publisher = self.create_publisher(JointTrajectory, args.command_topic, 10)
        self.feedback_sub = self.create_subscription(
            JointState,
            args.feedback_topic,
            self.feedback_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / args.rate, self.publish_command)
        self.command = [0.0] * 7
        self.have_feedback = False
        self.dirty = False
        self.running = True
        self.get_logger().info(
            f"Keyboard teleop publishing JointTrajectory to {args.command_topic}; feedback from {args.feedback_topic}"
        )

    def feedback_callback(self, msg):
        positions_by_name = dict(zip(msg.name, msg.position))
        if not self.have_feedback:
            for index, name in enumerate(JOINT_NAMES):
                if name in positions_by_name:
                    self.command[index] = positions_by_name[name]
            self.have_feedback = True
            self.get_logger().info("Initialized command from arm feedback")

    def apply_key(self, key):
        if key == "\x03":
            self.running = False
            return
        if key == " ":
            self.dirty = True
            return
        if key in ("[", "{"):
            self.args.step = max(0.001, self.args.step * 0.5)
            self.get_logger().info(f"Joint step: {self.args.step:.4f} rad")
            return
        if key in ("]", "}"):
            self.args.step = min(0.25, self.args.step * 2.0)
            self.get_logger().info(f"Joint step: {self.args.step:.4f} rad")
            return

        bindings = {
            "q": (0, 1.0),
            "a": (0, -1.0),
            "w": (1, 1.0),
            "s": (1, -1.0),
            "e": (2, 1.0),
            "d": (2, -1.0),
            "r": (3, 1.0),
            "f": (3, -1.0),
            "t": (4, 1.0),
            "g": (4, -1.0),
            "y": (5, 1.0),
            "h": (5, -1.0),
            "u": (6, 1.0),
            "j": (6, -1.0),
        }
        if key not in bindings:
            return

        joint_index, direction = bindings[key]
        step = self.args.gripper_step if joint_index == 6 else self.args.step
        lower, upper = (0.0, self.args.gripper_max) if joint_index == 6 else (-math.pi, math.pi)
        self.command[joint_index] = min(upper, max(lower, self.command[joint_index] + direction * step))
        self.dirty = True

    def publish_command(self):
        if self.args.require_feedback and not self.have_feedback:
            return
        if not self.dirty and not self.args.repeat:
            return
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = JOINT_NAMES[:]

        point = JointTrajectoryPoint()
        point.positions = self.command[:]
        point.velocities = [self.args.speed] * 7
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e9 / self.args.rate)
        trajectory.points.append(point)

        self.publisher.publish(trajectory)
        self.dirty = False


def parse_args():
    parser = argparse.ArgumentParser(description="Interactive Piper keyboard teleop through JointTrajectory bridge.")
    parser.add_argument("--command-topic", default="/arm_controller/joint_trajectory")
    parser.add_argument("--feedback-topic", default="/joint_states_single")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--step", type=float, default=0.02)
    parser.add_argument("--speed", type=float, default=20.0)
    parser.add_argument("--gripper-step", type=float, default=0.005)
    parser.add_argument("--gripper-max", type=float, default=0.08)
    parser.add_argument("--gripper-effort", type=float, default=1.0)
    parser.add_argument("--repeat", action="store_true")
    parser.add_argument("--no-require-feedback", dest="require_feedback", action="store_false")
    parser.set_defaults(require_feedback=True)
    return parser.parse_args()


def print_help():
    print(
        "\n".join(
            [
                "Piper keyboard joint teleop",
                "  q/a joint1   w/s joint2   e/d joint3",
                "  r/f joint4   t/g joint5   y/h joint6",
                "  u/j gripper open/close",
                "  [ and ] change joint step, space republishes hold command, Ctrl-C exits",
            ]
        )
    )


def main():
    args = parse_args()
    print_help()
    rclpy.init()
    node = KeyboardJointTeleop(args)
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                node.apply_key(sys.stdin.read(1))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
