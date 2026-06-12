#!/usr/bin/env python3

import argparse
import math
import time

import cv2
import numpy as np
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


def dictionary_from_name(name):
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise ValueError(f"OpenCV aruco dictionary not available: {name}")
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


class DepthAIAprilTagPosePublisher(Node):
    def __init__(self, args):
        super().__init__("depthai_apriltag_pose_publisher")
        self.args = args
        self.publisher = self.create_publisher(PoseStamped, args.output_topic, 10)
        self.dictionary = dictionary_from_name(args.dictionary)
        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            cv2.aruco.DetectorParameters(),
        )
        self.last_publish = 0.0
        self.last_log = 0.0
        self.last_depth_warn = 0.0
        self.get_logger().info(
            f"Publishing AprilTag center poses on {args.output_topic}; dictionary={args.dictionary}"
        )

    def run(self):
        import depthai as dai

        with dai.Pipeline() as pipeline:
            color_queue, depth_queue = self.build_depthai_pipeline(dai, pipeline)
            pipeline.start()
            self.get_logger().info("DepthAI AprilTag pipeline started. Press q or Esc in the image window to quit.")
            while rclpy.ok() and pipeline.isRunning():
                color_msg = color_queue.get()
                frame = color_msg.getCvFrame()
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if self.args.frame_color_mode == "rgb" else frame
                depth = None
                if depth_queue is not None:
                    depth_msg = depth_queue.tryGet()
                    if depth_msg is not None:
                        depth = depth_msg.getFrame()

                tag = self.detect_tag(bgr, depth)
                if tag is not None and tag["camera_xyz"] is not None:
                    self.publish_tag_pose(tag)
                self.log_status(tag)

                vis = bgr.copy()
                self.draw_status(vis, tag)
                cv2.imshow(self.args.window_name, vis)
                key = cv2.waitKey(1) & 0xFF
                rclpy.spin_once(self, timeout_sec=0.001)
                if key in (ord("q"), 27):
                    break
        cv2.destroyAllWindows()

    def build_depthai_pipeline(self, dai, pipeline):
        width = max(16, int(self.args.width) - (int(self.args.width) % 16))
        height = int(self.args.height)
        cam = pipeline.create(dai.node.Camera).build()
        rgb_output = cam.requestOutput(
            (width, height),
            type=dai.ImgFrame.Type.RGB888p,
            fps=self.args.fps,
        )
        color_queue = rgb_output.createOutputQueue(maxSize=2, blocking=False)

        depth_queue = None
        if self.args.depth_source == "stereo":
            left = pipeline.create(dai.node.MonoCamera)
            right = pipeline.create(dai.node.MonoCamera)
            stereo = pipeline.create(dai.node.StereoDepth)
            left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            left.setFps(self.args.fps)
            right.setFps(self.args.fps)
            preset = getattr(dai.node.StereoDepth.PresetMode, self.args.stereo_preset)
            stereo.setDefaultProfilePreset(preset)
            stereo.setLeftRightCheck(True)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputKeepAspectRatio(False)
            stereo.setOutputSize(width, height)
            left.out.link(stereo.left)
            right.out.link(stereo.right)
            depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
        return color_queue, depth_queue

    def detect_tag(self, bgr, depth):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(corners) == 0:
            return None

        candidates = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if self.args.tag_id >= 0 and marker_id != self.args.tag_id:
                continue
            pts = marker_corners.reshape(4, 2)
            area = cv2.contourArea(pts.astype(np.float32))
            center = pts.mean(axis=0)
            xyz = self.pixel_to_camera_xyz(
                int(round(center[0])),
                int(round(center[1])),
                bgr.shape,
                depth,
                pts,
            )
            candidates.append((area, marker_id, pts, center, xyz))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        _area, marker_id, pts, center, xyz = candidates[0]
        return {
            "id": marker_id,
            "corners": pts,
            "center": center,
            "camera_xyz": xyz,
        }

    def pixel_to_camera_xyz(self, u, v, image_shape, depth, tag_corners):
        height, width = image_shape[:2]
        fx = self.args.fx if self.args.fx > 0 else width / (2.0 * math.tan(math.radians(self.args.fov_deg) * 0.5))
        fy = self.args.fy if self.args.fy > 0 else fx
        cx = self.args.cx if self.args.cx >= 0 else width * 0.5
        cy = self.args.cy if self.args.cy >= 0 else height * 0.5
        z = None

        if depth is not None:
            du = int(u * depth.shape[1] / width)
            dv = int(v * depth.shape[0] / height)
            valid = depth[
                max(0, dv - self.args.depth_window): min(depth.shape[0], dv + self.args.depth_window + 1),
                max(0, du - self.args.depth_window): min(depth.shape[1], du + self.args.depth_window + 1),
            ]
            valid = valid[np.isfinite(valid)]
            valid = valid[valid > 0]
            if valid.size >= self.args.min_depth_pixels:
                z_values = valid.astype(np.float32)
                if np.nanmedian(z_values) > 20.0:
                    z_values *= 0.001
                z_values = z_values[(z_values >= self.args.min_depth_m) & (z_values <= self.args.max_depth_m)]
                if z_values.size >= self.args.min_depth_pixels:
                    z = float(np.median(z_values))

        if z is None and self.args.tag_size_m > 0:
            z = self.estimate_depth_from_tag_size(tag_corners, fx, fy)
            if z is not None:
                self.warn_depth_throttled("Using tag-size depth estimate for AprilTag center")

        if z is None:
            return None
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return [x, y, z]

    def estimate_depth_from_tag_size(self, tag_corners, fx, fy):
        pts = np.asarray(tag_corners, dtype=np.float64).reshape(4, 2)
        edges = [
            np.linalg.norm(pts[1] - pts[0]),
            np.linalg.norm(pts[2] - pts[1]),
            np.linalg.norm(pts[3] - pts[2]),
            np.linalg.norm(pts[0] - pts[3]),
        ]
        pixel_size = float(np.mean(edges))
        if pixel_size <= 1e-6:
            return None
        focal = 0.5 * (fx + fy)
        z = self.args.tag_size_m * focal / pixel_size
        if self.args.min_depth_m <= z <= self.args.max_depth_m:
            return z
        return None

    def publish_tag_pose(self, tag):
        now = time.time()
        if now - self.last_publish < 1.0 / self.args.publish_rate:
            return
        self.last_publish = now
        x, y, z = tag["camera_xyz"]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.camera_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(0.0, math.pi, 0.0)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.publisher.publish(msg)

    def log_status(self, tag):
        now = time.time()
        if now - self.last_log < self.args.log_period:
            return
        self.last_log = now
        if tag is None:
            self.get_logger().warn("No AprilTag detected")
            return
        if tag["camera_xyz"] is None:
            self.get_logger().warn(f"Detected AprilTag id={tag['id']}, but no valid 3D point")
            return
        x, y, z = tag["camera_xyz"]
        self.get_logger().info(
            f"Selected AprilTag id={tag['id']} pixel=({tag['center'][0]:.0f},{tag['center'][1]:.0f}) "
            f"camera_xyz=({x:.3f},{y:.3f},{z:.3f})"
        )

    def warn_depth_throttled(self, message):
        now = time.time()
        if now - self.last_depth_warn >= self.args.depth_warn_period:
            self.last_depth_warn = now
            self.get_logger().warn(message)

    def draw_status(self, image, tag):
        if tag is None:
            cv2.putText(image, "No AprilTag", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 180, 255), 2)
            return
        pts = tag["corners"].astype(int)
        cv2.polylines(image, [pts], True, (0, 255, 0), 3)
        center = tuple(int(v) for v in tag["center"])
        cv2.circle(image, center, 7, (255, 0, 0), -1)
        if tag["camera_xyz"] is None:
            label = f"id={tag['id']} no 3D point"
            color = (0, 180, 255)
        else:
            x, y, z = tag["camera_xyz"]
            label = f"id={tag['id']} cam=({x:.3f},{y:.3f},{z:.3f})"
            color = (0, 255, 0)
        cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def parse_args():
    parser = argparse.ArgumentParser(description="DepthAI AprilTag center pose publisher.")
    parser.add_argument("--output-topic", default="/perception/object_pose_camera")
    parser.add_argument("--camera-frame", default="depthai_camera")
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--tag-id", type=int, default=-1)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--frame-color-mode", choices=["bgr", "rgb"], default="bgr")
    parser.add_argument("--window-name", default="DepthAI AprilTag Pose")
    parser.add_argument("--depth-source", choices=["tag_size", "stereo"], default="tag_size")
    parser.add_argument("--stereo-preset", choices=["DEFAULT", "DENSITY", "FAST_DENSITY", "ROBOTICS", "HIGH_DETAIL"], default="FAST_DENSITY")
    parser.add_argument("--tag-size-m", type=float, default=0.08)
    parser.add_argument("--depth-window", type=int, default=12)
    parser.add_argument("--min-depth-pixels", type=int, default=10)
    parser.add_argument("--min-depth-m", type=float, default=0.12)
    parser.add_argument("--max-depth-m", type=float, default=2.0)
    parser.add_argument("--fov-deg", type=float, default=69.0)
    parser.add_argument("--fx", type=float, default=-1.0)
    parser.add_argument("--fy", type=float, default=-1.0)
    parser.add_argument("--cx", type=float, default=-1.0)
    parser.add_argument("--cy", type=float, default=-1.0)
    parser.add_argument("--publish-rate", type=float, default=10.0)
    parser.add_argument("--log-period", type=float, default=1.0)
    parser.add_argument("--depth-warn-period", type=float, default=3.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DepthAIAprilTagPosePublisher(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
