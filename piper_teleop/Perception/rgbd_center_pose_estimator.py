#!/usr/bin/env python3

import argparse
import math
from threading import Lock

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


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


class RgbdCenterPoseEstimator(Node):
    def __init__(self, args):
        super().__init__("rgbd_center_pose_estimator")
        self.args = args
        self.bridge = CvBridge()
        self.lock = Lock()
        self.rgb = None
        self.depth = None
        self.camera_info = None
        self.publisher = self.create_publisher(PoseStamped, args.output_topic, 10)
        self.create_subscription(Image, args.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, args.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.camera_info_callback, 10)
        self.timer = self.create_timer(1.0 / args.rate, self.estimate)
        self.get_logger().info(
            f"RGBD estimator publishing {args.output_topic}; mode={args.mode}"
        )

    def rgb_callback(self, msg):
        with self.lock:
            self.rgb = msg

    def depth_callback(self, msg):
        with self.lock:
            self.depth = msg

    def camera_info_callback(self, msg):
        with self.lock:
            self.camera_info = msg

    def estimate(self):
        with self.lock:
            rgb_msg = self.rgb
            depth_msg = self.depth
            info = self.camera_info

        if depth_msg is None or info is None:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert depth image: {exc}")
            return

        if self.args.mode == "pixel":
            u = self.args.pixel_u
            v = self.args.pixel_v
        else:
            if rgb_msg is None:
                return
            try:
                rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            except Exception as exc:
                self.get_logger().warn(f"Failed to convert RGB image: {exc}")
                return
            center = self.find_color_center(rgb)
            if center is None:
                return
            u, v = center

        xyz = self.back_project(depth, info, u, v)
        if xyz is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = depth_msg.header.frame_id or self.args.camera_frame
        msg.pose.position.x = xyz[0]
        msg.pose.position.y = xyz[1]
        msg.pose.position.z = xyz[2]
        qx, qy, qz, qw = quaternion_from_euler(0.0, math.pi, 0.0)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.publisher.publish(msg)

    def find_color_center(self, rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        lower = np.array([self.args.h_min, self.args.s_min, self.args.v_min], dtype=np.uint8)
        upper = np.array([self.args.h_max, self.args.s_max, self.args.v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        moments = cv2.moments(mask)
        if moments["m00"] <= self.args.min_area:
            return None
        u = int(moments["m10"] / moments["m00"])
        v = int(moments["m01"] / moments["m00"])
        return u, v

    def back_project(self, depth, info, u, v):
        if v < 0 or u < 0 or v >= depth.shape[0] or u >= depth.shape[1]:
            self.get_logger().warn(f"Pixel ({u}, {v}) is outside depth image")
            return None

        window = depth[
            max(0, v - self.args.depth_window): min(depth.shape[0], v + self.args.depth_window + 1),
            max(0, u - self.args.depth_window): min(depth.shape[1], u + self.args.depth_window + 1),
        ]
        valid = window[np.isfinite(window)]
        valid = valid[valid > 0]
        if valid.size == 0:
            self.get_logger().warn(f"No valid depth around pixel ({u}, {v})")
            return None

        z = float(np.median(valid))
        if depth.dtype == np.uint16 or z > 20.0:
            z *= 0.001

        fx = info.k[0]
        fy = info.k[4]
        cx = info.k[2]
        cy = info.k[5]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stage 1 perception: back-project a selected RGB pixel using aligned depth."
    )
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--output-topic", default="/perception/object_pose_camera")
    parser.add_argument("--camera-frame", default="camera_color_optical_frame")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--mode", choices=["pixel", "color"], default="pixel")
    parser.add_argument("--pixel-u", type=int, default=320)
    parser.add_argument("--pixel-v", type=int, default=240)
    parser.add_argument("--depth-window", type=int, default=3)
    parser.add_argument("--h-min", type=int, default=0)
    parser.add_argument("--h-max", type=int, default=179)
    parser.add_argument("--s-min", type=int, default=80)
    parser.add_argument("--s-max", type=int, default=255)
    parser.add_argument("--v-min", type=int, default=80)
    parser.add_argument("--v-max", type=int, default=255)
    parser.add_argument("--min-area", type=float, default=100.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RgbdCenterPoseEstimator(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
