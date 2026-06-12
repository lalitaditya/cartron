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


def euler_to_rotation_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def solve_translation_with_fixed_rotation(camera_points, base_points, roll, pitch, yaw):
    camera = np.asarray(camera_points, dtype=np.float64)
    base = np.asarray(base_points, dtype=np.float64)
    if camera.shape != base.shape or camera.ndim != 2 or camera.shape[1] != 3:
        raise ValueError("camera_points and base_points must both be Nx3 arrays")
    if camera.shape[0] < 1:
        raise ValueError("Need at least 1 point pair")
    rotation = euler_to_rotation_matrix(roll, pitch, yaw)
    translations = base - (rotation @ camera.T).T
    translation = translations.mean(axis=0)
    transformed = (rotation @ camera.T).T + translation
    residuals = np.linalg.norm(transformed - base, axis=1)
    translation_spread = translations.max(axis=0) - translations.min(axis=0)
    return rotation, translation, residuals, translation_spread


def dictionary_from_name(name):
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise ValueError(f"OpenCV aruco dictionary not available: {name}")
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


class AprilTagPointPairCalibrator(Node):
    def __init__(self, args):
        super().__init__("apriltag_point_pair_calibrator")
        self.args = args
        self.latest_end_pose = None
        self.samples = []
        self.last_tag = None
        self.last_valid_tag = None
        self.last_valid_tag_time = 0.0
        self.last_valid_tag_seq = 0
        self.last_saved_tag_seq = -1
        self.last_status_log = 0.0
        self.create_subscription(Pose, args.end_pose_topic, self.end_pose_callback, 10)
        self.dictionary = dictionary_from_name(args.dictionary)
        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            cv2.aruco.DetectorParameters(),
        )

    def end_pose_callback(self, msg):
        self.latest_end_pose = msg

    def run(self):
        import depthai as dai

        with dai.Pipeline() as pipeline:
            color_queue, depth_queue = self.build_depthai_pipeline(dai, pipeline)
            pipeline.start()
            self.get_logger().info(
                "AprilTag calibration running. Save each point pair from the visible tag center."
            )
            print("")
            if self.args.base_source == "manual":
                print("Manual mode: enter measured base XYZ in meters, for example: 0.35 -0.12 0.08")
            elif self.args.base_source == "hover":
                print(
                    "Hover mode: place the gripper tip above the tag center, then press Enter. "
                    f"The script subtracts hover offset ({self.args.hover_offset_x:.3f}, "
                    f"{self.args.hover_offset_y:.3f}, {self.args.hover_offset_z:.3f}) m."
                )
            else:
                print("End-pose mode: move the gripper tip to the visible tag center, then press Enter.")
            print("Controls: Enter=save from robot feedback, '<x> <y> <z>'=save measured base point, s=solve, q=quit")
            print("")
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
                self.last_tag = tag
                if tag is not None and tag["camera_xyz"] is not None:
                    self.last_valid_tag = tag
                    self.last_valid_tag_time = time.time()
                    self.last_valid_tag_seq += 1
                    self.last_valid_tag["seq"] = self.last_valid_tag_seq
                self.log_detection_status(tag)
                vis = bgr.copy()
                self.draw_status(vis, tag)
                cv2.imshow(self.args.window_name, vis)
                key = cv2.waitKey(1) & 0xFF
                rclpy.spin_once(self, timeout_sec=0.001)
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    self.solve_and_print()
                if key in (ord(" "), 10, 13):
                    self.save_sample(tag, None)

                if self.stdin_ready_prompt():
                    command = input("Press Enter to save, s to solve, q to quit: ").strip().lower()
                    if command == "q":
                        break
                    if command == "s":
                        self.solve_and_print()
                    else:
                        base_xyz = self.parse_manual_base_xyz(command)
                        self.save_sample(tag, base_xyz)
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

    def log_detection_status(self, tag):
        now = time.time()
        if now - self.last_status_log < self.args.status_log_period:
            return
        self.last_status_log = now
        if tag is None:
            self.get_logger().warn("No AprilTag detected in the camera image")
        elif tag["camera_xyz"] is None:
            self.get_logger().warn(f"Detected AprilTag id={tag['id']}, but no valid stereo depth at tag center")
        else:
            xyz = tag["camera_xyz"]
            self.get_logger().info(
                f"Detected AprilTag id={tag['id']} camera_xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"
            )

    def detect_tag(self, bgr, depth):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(corners) == 0:
            return None
        candidates = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if self.args.tag_id >= 0 and int(marker_id) != self.args.tag_id:
                continue
            pts = marker_corners.reshape(4, 2)
            area = cv2.contourArea(pts.astype(np.float32))
            center = pts.mean(axis=0)
            xyz = self.pixel_to_camera_xyz(int(round(center[0])), int(round(center[1])), bgr.shape, depth, pts)
            candidates.append((area, int(marker_id), pts, center, xyz))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        _, marker_id, pts, center, xyz = candidates[0]
        return {
            "id": marker_id,
            "corners": pts,
            "center": center,
            "camera_xyz": xyz,
        }

    def pixel_to_camera_xyz(self, u, v, image_shape, depth, tag_corners=None):
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
                z_values = z_values[
                    (z_values >= self.args.min_depth_m) & (z_values <= self.args.max_depth_m)
                ]
                if z_values.size >= self.args.min_depth_pixels:
                    z = float(np.median(z_values))
        if z is None and self.args.tag_size_m > 0 and tag_corners is not None:
            z = self.estimate_depth_from_tag_size(tag_corners, fx, fy)
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

    def draw_status(self, image, tag):
        if tag is None:
            cv2.putText(image, "No AprilTag", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 180, 255), 2)
            return
        pts = tag["corners"].astype(int)
        cv2.polylines(image, [pts], True, (0, 255, 0), 3)
        center = tuple(int(v) for v in tag["center"])
        cv2.circle(image, center, 6, (255, 0, 0), -1)
        xyz = tag["camera_xyz"]
        if xyz is None:
            label = f"id={tag['id']} no valid depth"
            color = (0, 180, 255)
        else:
            label = f"id={tag['id']} cam=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}) samples={len(self.samples)}"
            color = (0, 255, 0)
        cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    def parse_manual_base_xyz(self, command):
        if command == "":
            return None
        parts = command.replace(",", " ").split()
        if len(parts) != 3:
            print("Expected measured base XYZ as three numbers in meters, for example: 0.35 -0.12 0.08")
            return "invalid"
        try:
            return [float(part) for part in parts]
        except ValueError:
            print("Could not parse measured base XYZ. Use meters, for example: 0.35 -0.12 0.08")
            return "invalid"

    def save_sample(self, tag, manual_base_xyz=None):
        self._last_prompt = time.time()
        if manual_base_xyz == "invalid":
            return
        if tag is None or tag["camera_xyz"] is None:
            age = time.time() - self.last_valid_tag_time
            if self.last_valid_tag is not None and (
                self.args.last_tag_timeout <= 0.0 or age <= self.args.last_tag_timeout
            ):
                tag = self.last_valid_tag
                print(f"Using last valid AprilTag detection from {age:.1f}s ago.")
        if tag is None or tag["camera_xyz"] is None:
            print("No valid AprilTag camera XYZ yet; not saved.")
            return
        tag_seq = tag.get("seq", self.last_valid_tag_seq)
        if tag_seq == self.last_saved_tag_seq:
            print("This AprilTag camera detection was already saved. Move to the next position and get a new detection first.")
            return
        if self.args.base_source == "manual":
            if manual_base_xyz is None:
                print("Manual mode needs measured base XYZ. Type: x y z   in meters.")
                return
            base_xyz = manual_base_xyz
        else:
            if self.latest_end_pose is None:
                print(f"No {self.args.end_pose_topic} feedback yet; not saved.")
                return
            base_xyz = [
                self.latest_end_pose.position.x,
                self.latest_end_pose.position.y,
                self.latest_end_pose.position.z,
            ]
            if self.args.base_source == "hover":
                base_xyz = [
                    base_xyz[0] - self.args.hover_offset_x,
                    base_xyz[1] - self.args.hover_offset_y,
                    base_xyz[2] - self.args.hover_offset_z,
                ]
        sample = {
            "tag_id": tag["id"],
            "camera_xyz": tag["camera_xyz"],
            "base_xyz": base_xyz,
            "base_source": self.args.base_source,
            "time": time.time(),
        }
        self.samples.append(sample)
        self.last_saved_tag_seq = tag_seq
        print(
            f"Saved sample {len(self.samples)}: "
            f"camera={sample['camera_xyz']} base={sample['base_xyz']}"
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
        if self.args.solve_mode == "rigid" and len(self.samples) < 3:
            print(f"Need at least 3 samples to solve rigid transform; have {len(self.samples)}.")
            return
        if self.args.solve_mode == "fixed_rpy" and len(self.samples) < 1:
            print("Need at least 1 sample to solve fixed-RPY translation.")
            return
        camera_xyz = [sample["camera_xyz"] for sample in self.samples]
        base_xyz = [sample["base_xyz"] for sample in self.samples]
        fixed_rotation = self.args.solve_mode == "fixed_rpy"
        translation_spread = None
        if fixed_rotation:
            rotation, translation, residuals, translation_spread = solve_translation_with_fixed_rotation(
                camera_xyz,
                base_xyz,
                self.args.fixed_roll,
                self.args.fixed_pitch,
                self.args.fixed_yaw,
            )
            roll, pitch, yaw = self.args.fixed_roll, self.args.fixed_pitch, self.args.fixed_yaw
        else:
            rotation, translation, residuals = solve_rigid_transform(camera_xyz, base_xyz)
            roll, pitch, yaw = rotation_matrix_to_euler(rotation)
        camera_points = np.asarray([sample["camera_xyz"] for sample in self.samples], dtype=np.float64)
        base_points = np.asarray([sample["base_xyz"] for sample in self.samples], dtype=np.float64)
        transformed = (rotation @ camera_points.T).T + translation
        camera_span = camera_points.max(axis=0) - camera_points.min(axis=0)
        base_span = base_points.max(axis=0) - base_points.min(axis=0)
        print("")
        if fixed_rotation:
            print("Solved T_base_camera with fixed camera RPY:")
        else:
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
        print(
            f"camera span=({camera_span[0]:.3f}, {camera_span[1]:.3f}, {camera_span[2]:.3f}) m, "
            f"base span=({base_span[0]:.3f}, {base_span[1]:.3f}, {base_span[2]:.3f}) m"
        )
        if translation_spread is not None and len(self.samples) > 1:
            print(
                f"fixed-RPY translation spread=({translation_spread[0]:.3f}, "
                f"{translation_spread[1]:.3f}, {translation_spread[2]:.3f}) m"
            )
        if float(np.mean(residuals)) > self.args.max_good_residual:
            print(
                f"WARNING: residual is too high for grasping. Target mean is <= "
                f"{self.args.max_good_residual:.3f} m."
            )
        if not fixed_rotation and np.linalg.norm(camera_span) < self.args.min_camera_span:
            print(
                "WARNING: saved camera points are tightly clustered. Move the visible tag to "
                "clearly different places in the camera view before saving each sample."
            )
        if fixed_rotation:
            print(
                "NOTE: fixed_rpy mode only calibrates camera translation. If the camera is tilted "
                "differently from the fixed RPY, the grasp target will still be directionally wrong."
            )
        print("Per-sample residuals:")
        for index, (sample, predicted, residual) in enumerate(zip(self.samples, transformed, residuals), 1):
            print(
                f"  {index}: residual={float(residual):.4f} m, "
                f"base=({sample['base_xyz'][0]:.3f}, {sample['base_xyz'][1]:.3f}, {sample['base_xyz'][2]:.3f}), "
                f"pred=({predicted[0]:.3f}, {predicted[1]:.3f}, {predicted[2]:.3f})"
            )
        print("")


def parse_args():
    parser = argparse.ArgumentParser(
        description="No-printer camera-to-base calibration from AprilTag center point pairs."
    )
    parser.add_argument("--end-pose-topic", default="/end_pose")
    parser.add_argument("--base-source", choices=["manual", "end_pose", "hover"], default="manual")
    parser.add_argument("--hover-offset-x", type=float, default=0.0)
    parser.add_argument("--hover-offset-y", type=float, default=0.0)
    parser.add_argument("--hover-offset-z", type=float, default=0.05)
    parser.add_argument("--output", default="piper_teleop/Perception/apriltag_point_pairs.json")
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--tag-id", type=int, default=-1)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--frame-color-mode", choices=["bgr", "rgb"], default="bgr")
    parser.add_argument("--window-name", default="AprilTag Point-Pair Calibration")
    parser.add_argument("--depth-source", choices=["tag_size", "stereo"], default="tag_size")
    parser.add_argument("--stereo-preset", choices=["DEFAULT", "DENSITY", "FAST_DENSITY", "ROBOTICS", "HIGH_DETAIL"], default="FAST_DENSITY")
    parser.add_argument("--depth-window", type=int, default=12)
    parser.add_argument("--min-depth-pixels", type=int, default=10)
    parser.add_argument("--min-depth-m", type=float, default=0.12)
    parser.add_argument("--max-depth-m", type=float, default=2.0)
    parser.add_argument("--tag-size-m", type=float, default=0.0)
    parser.add_argument("--fov-deg", type=float, default=69.0)
    parser.add_argument("--fx", type=float, default=-1.0)
    parser.add_argument("--fy", type=float, default=-1.0)
    parser.add_argument("--cx", type=float, default=-1.0)
    parser.add_argument("--cy", type=float, default=-1.0)
    parser.add_argument("--solve-after", type=int, default=6)
    parser.add_argument(
        "--solve-mode",
        choices=["rigid", "fixed_rpy"],
        default="rigid",
        help="rigid solves full T_base_camera from 3+ samples; fixed_rpy solves only translation from 1+ samples.",
    )
    parser.add_argument("--fixed-roll", type=float, default=-1.57079632679)
    parser.add_argument("--fixed-pitch", type=float, default=0.0)
    parser.add_argument("--fixed-yaw", type=float, default=-1.57079632679)
    parser.add_argument("--prompt-period", type=float, default=0.2)
    parser.add_argument("--status-log-period", type=float, default=1.0)
    parser.add_argument(
        "--last-tag-timeout",
        type=float,
        default=10.0,
        help="Seconds to allow saving the last valid tag detection. Use 0 for no timeout.",
    )
    parser.add_argument("--max-good-residual", type=float, default=0.05)
    parser.add_argument("--min-camera-span", type=float, default=0.15)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AprilTagPointPairCalibrator(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
