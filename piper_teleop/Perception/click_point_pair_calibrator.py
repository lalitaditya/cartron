#!/usr/bin/env python3

import argparse
import json
import math
import select
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node


def rotation_matrix_to_euler(matrix):
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def solve_rigid_transform(camera_points, base_points):
    camera = np.asarray(camera_points, dtype=np.float64)
    base = np.asarray(base_points, dtype=np.float64)
    if camera.shape != base.shape or camera.ndim != 2 or camera.shape[1] != 3:
        raise ValueError("camera_points and base_points must both be Nx3 arrays")
    if camera.shape[0] < 3:
        raise ValueError("Need at least 3 point pairs; 6 or more is better")

    camera_mean = camera.mean(axis=0)
    base_mean = base.mean(axis=0)
    camera_centered = camera - camera_mean
    base_centered = base - base_mean
    covariance = camera_centered.T @ base_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = base_mean - rotation @ camera_mean
    transformed = (rotation @ camera.T).T + translation
    residuals = np.linalg.norm(transformed - base, axis=1)
    return rotation, translation, residuals


class ClickPointPairCalibrator(Node):
    def __init__(self, args):
        super().__init__("click_point_pair_calibrator")
        self.args = args
        self.latest_end_pose = None
        self.samples = []
        self.clicked_pixel = None
        self.clicked_xyz = None
        self.create_subscription(Pose, args.end_pose_topic, self.end_pose_callback, 10)

    def end_pose_callback(self, msg):
        self.latest_end_pose = msg

    def mouse_callback(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_pixel = (int(x), int(y))
            self.clicked_xyz = None
            print(f"Clicked pixel ({x}, {y}); wait for valid depth, then press Enter to save.")

    def run(self):
        import depthai as dai

        with dai.Pipeline() as pipeline:
            color_queue, depth_queue = self.build_depthai_pipeline(dai, pipeline)
            pipeline.start()
            cv2.namedWindow(self.args.window_name)
            cv2.setMouseCallback(self.args.window_name, self.mouse_callback)
            self.get_logger().info(
                "Click-point calibration running. Click a safe physical point in the camera image, "
                "move the gripper tip to that same point, then press Enter in this terminal."
            )
            print("")
            print("Controls: mouse left-click=select camera point, Enter=save sample, s=solve, q=quit")
            print("")

            while rclpy.ok() and pipeline.isRunning():
                color_msg = color_queue.get()
                frame = color_msg.getCvFrame()
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if self.args.frame_color_mode == "rgb" else frame
                depth = None
                depth_msg = depth_queue.tryGet()
                if depth_msg is not None:
                    depth = depth_msg.getFrame()

                if self.clicked_pixel is not None:
                    self.clicked_xyz = self.pixel_to_camera_xyz(
                        self.clicked_pixel[0],
                        self.clicked_pixel[1],
                        bgr.shape,
                        depth,
                    )

                vis = bgr.copy()
                self.draw_status(vis)
                cv2.imshow(self.args.window_name, vis)
                key = cv2.waitKey(1) & 0xFF
                rclpy.spin_once(self, timeout_sec=0.001)
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    self.solve_and_print()

                if self.stdin_ready_prompt():
                    command = input("Press Enter to save, s to solve, q to quit: ").strip().lower()
                    if command == "q":
                        break
                    if command == "s":
                        self.solve_and_print()
                    else:
                        self.save_sample()
        cv2.destroyAllWindows()

    def stdin_ready_prompt(self):
        if time.time() - getattr(self, "_last_prompt", 0.0) < self.args.prompt_period:
            return False
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        return bool(ready)

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

    def pixel_to_camera_xyz(self, u, v, image_shape, depth):
        if depth is None:
            return None
        height, width = image_shape[:2]
        du = int(u * depth.shape[1] / width)
        dv = int(v * depth.shape[0] / height)
        valid = depth[
            max(0, dv - self.args.depth_window): min(depth.shape[0], dv + self.args.depth_window + 1),
            max(0, du - self.args.depth_window): min(depth.shape[1], du + self.args.depth_window + 1),
        ]
        valid = valid[np.isfinite(valid)]
        valid = valid[valid > 0]
        if valid.size < self.args.min_depth_pixels:
            return None
        z_values = valid.astype(np.float32)
        if np.nanmedian(z_values) > 20.0:
            z_values *= 0.001
        z_values = z_values[
            (z_values >= self.args.min_depth_m) & (z_values <= self.args.max_depth_m)
        ]
        if z_values.size < self.args.min_depth_pixels:
            return None
        z = float(np.median(z_values))
        fx = self.args.fx if self.args.fx > 0 else width / (2.0 * math.tan(math.radians(self.args.fov_deg) * 0.5))
        fy = self.args.fy if self.args.fy > 0 else fx
        cx = self.args.cx if self.args.cx >= 0 else width * 0.5
        cy = self.args.cy if self.args.cy >= 0 else height * 0.5
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return [x, y, z]

    def draw_status(self, image):
        if self.clicked_pixel is None:
            cv2.putText(
                image,
                f"Click a safe calibration point. samples={len(self.samples)}",
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 180, 255),
                2,
            )
            return
        cv2.drawMarker(
            image,
            self.clicked_pixel,
            (255, 0, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=3,
        )
        if self.clicked_xyz is None:
            label = f"clicked={self.clicked_pixel} no valid depth samples={len(self.samples)}"
            color = (0, 180, 255)
        else:
            label = (
                f"cam=({self.clicked_xyz[0]:.3f},{self.clicked_xyz[1]:.3f},"
                f"{self.clicked_xyz[2]:.3f}) samples={len(self.samples)}"
            )
            color = (0, 255, 0)
        cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    def save_sample(self):
        self._last_prompt = time.time()
        if self.clicked_pixel is None or self.clicked_xyz is None:
            print("No clicked point with valid depth yet; not saved.")
            return
        if self.latest_end_pose is None:
            print(f"No {self.args.end_pose_topic} feedback yet; not saved.")
            return
        base_xyz = [
            self.latest_end_pose.position.x,
            self.latest_end_pose.position.y,
            self.latest_end_pose.position.z,
        ]
        sample = {
            "pixel": list(self.clicked_pixel),
            "camera_xyz": self.clicked_xyz,
            "base_xyz": base_xyz,
            "time": time.time(),
        }
        self.samples.append(sample)
        print(
            f"Saved sample {len(self.samples)}: "
            f"pixel={sample['pixel']} camera={sample['camera_xyz']} base={sample['base_xyz']}"
        )
        self.save_samples_file()
        if len(self.samples) >= self.args.solve_after:
            self.solve_and_print()

    def save_samples_file(self):
        if not self.args.output:
            return
        path = Path(self.args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"samples": self.samples}, indent=2), encoding="utf-8")

    def solve_and_print(self):
        if len(self.samples) < 3:
            print(f"Need at least 3 samples to solve; have {len(self.samples)}.")
            return
        rotation, translation, residuals = solve_rigid_transform(
            [sample["camera_xyz"] for sample in self.samples],
            [sample["base_xyz"] for sample in self.samples],
        )
        roll, pitch, yaw = rotation_matrix_to_euler(rotation)
        print("")
        print("Solved T_base_camera:")
        print(f"--camera-x {translation[0]:.6f} \\")
        print(f"--camera-y {translation[1]:.6f} \\")
        print(f"--camera-z {translation[2]:.6f} \\")
        print(f"--camera-roll {roll:.9f} \\")
        print(f"--camera-pitch {pitch:.9f} \\")
        print(f"--camera-yaw {yaw:.9f}")
        print(
            f"residual mean={float(np.mean(residuals)):.4f} m, "
            f"max={float(np.max(residuals)):.4f} m, samples={len(self.samples)}"
        )
        print("")


def parse_args():
    parser = argparse.ArgumentParser(
        description="No-printer camera-to-base calibration from clicked RGBD points and robot tip point pairs."
    )
    parser.add_argument("--end-pose-topic", default="/end_pose")
    parser.add_argument("--output", default="piper_teleop/Perception/click_point_pairs.json")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--frame-color-mode", choices=["bgr", "rgb"], default="bgr")
    parser.add_argument("--window-name", default="Click Point-Pair Calibration")
    parser.add_argument("--stereo-preset", choices=["DEFAULT", "DENSITY", "FAST_DENSITY", "ROBOTICS", "HIGH_DETAIL"], default="FAST_DENSITY")
    parser.add_argument("--depth-window", type=int, default=12)
    parser.add_argument("--min-depth-pixels", type=int, default=10)
    parser.add_argument("--min-depth-m", type=float, default=0.12)
    parser.add_argument("--max-depth-m", type=float, default=2.0)
    parser.add_argument("--fov-deg", type=float, default=69.0)
    parser.add_argument("--fx", type=float, default=-1.0)
    parser.add_argument("--fy", type=float, default=-1.0)
    parser.add_argument("--cx", type=float, default=-1.0)
    parser.add_argument("--cy", type=float, default=-1.0)
    parser.add_argument("--solve-after", type=int, default=6)
    parser.add_argument("--prompt-period", type=float, default=0.2)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ClickPointPairCalibrator(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
