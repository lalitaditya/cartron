#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
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


def quaternion_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return [
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ]


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class CameraToBasePoseTransformer(Node):
    def __init__(self, args):
        super().__init__("camera_to_base_pose_transformer")
        self.args = args
        self.base_q_camera = quaternion_from_euler(args.roll, args.pitch, args.yaw)
        self.base_r_camera = quaternion_matrix(self.base_q_camera)
        self.last_log = 0.0
        self.publisher = self.create_publisher(PoseStamped, args.output_topic, 10)
        self.create_subscription(PoseStamped, args.input_topic, self.callback, 10)
        self.get_logger().info(
            f"Transforming {args.input_topic} -> {args.output_topic} with static T_base_camera"
        )

    def callback(self, msg):
        point = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        rotated = [
            sum(self.base_r_camera[row][col] * point[col] for col in range(3))
            for row in range(3)
        ]
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.args.base_frame
        out.pose.position.x = self.args.x + rotated[0]
        out.pose.position.y = self.args.y + rotated[1]
        out.pose.position.z = self.args.z + rotated[2]

        object_q_camera = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        qx, qy, qz, qw = quaternion_multiply(self.base_q_camera, object_q_camera)
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.publisher.publish(out)
        self.log_transform_throttled(point, out)

    def log_transform_throttled(self, camera_point, base_msg):
        now = time.time()
        if now - self.last_log < self.args.log_period:
            return
        self.last_log = now
        self.get_logger().info(
            "Pose transform "
            f"camera_xyz=({camera_point[0]:.3f},{camera_point[1]:.3f},{camera_point[2]:.3f}) "
            f"-> base_xyz=({base_msg.pose.position.x:.3f},{base_msg.pose.position.y:.3f},"
            f"{base_msg.pose.position.z:.3f}); "
            f"T_base_camera=({self.args.x:.3f},{self.args.y:.3f},{self.args.z:.3f}, "
            f"rpy={self.args.roll:.3f},{self.args.pitch:.3f},{self.args.yaw:.3f})"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stage 3 placeholder: apply static hand-eye T_base_camera."
    )
    parser.add_argument("--input-topic", default="/perception/object_pose_camera")
    parser.add_argument("--output-topic", default="/perception/object_pose_base")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--log-period", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CameraToBasePoseTransformer(args)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
