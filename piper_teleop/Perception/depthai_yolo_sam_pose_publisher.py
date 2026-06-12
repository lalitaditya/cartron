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


def normalize_vector(values):
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        return [0.0, 0.0, -1.0]
    return [value / norm for value in values]


class Detection:
    def __init__(self, xyxy, score, class_id, label):
        self.xyxy = xyxy
        self.score = score
        self.class_id = class_id
        self.label = label

    @property
    def center(self):
        x1, y1, x2, y2 = self.xyxy
        return int((x1 + x2) * 0.5), int((y1 + y2) * 0.5)


class UltralyticsYoloDetector:
    def __init__(self, model_path, target_class, confidence):
        try:
            from ultralytics import YOLO
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Ultralytics is not installed. Install it or use --detector opencv_onnx."
            ) from exc
        self.model = YOLO(model_path)
        self.target_class = target_class
        self.confidence = confidence

    def detect(self, bgr):
        results = self.model.predict(bgr, verbose=False, conf=self.confidence)
        detections = []
        if not results:
            return detections
        result = results[0]
        names = result.names
        boxes = result.boxes
        if boxes is None:
            return detections
        for box in boxes:
            score = float(box.conf[0])
            class_id = int(box.cls[0])
            label = str(names.get(class_id, class_id))
            if self.target_class and label != self.target_class:
                continue
            xyxy = tuple(float(v) for v in box.xyxy[0])
            detections.append(Detection(xyxy, score, class_id, label))
        return detections


class OpenCvOnnxYoloDetector:
    def __init__(self, model_path, target_class_id, confidence, input_size):
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.target_class_id = target_class_id
        self.confidence = confidence
        self.input_size = input_size

    def detect(self, bgr):
        height, width = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(
            bgr,
            scalefactor=1.0 / 255.0,
            size=(self.input_size, self.input_size),
            mean=(0, 0, 0),
            swapRB=False,
            crop=False,
        )
        self.net.setInput(blob)
        output = self.net.forward()
        rows = self.normalize_yolo_output(output)
        detections = []
        scale_x = width / self.input_size
        scale_y = height / self.input_size
        for row in rows:
            if len(row) < 6:
                continue
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])
            if score < self.confidence:
                continue
            if self.target_class_id is not None and class_id != self.target_class_id:
                continue
            cx, cy, w, h = row[:4]
            x1 = max(0.0, (cx - w * 0.5) * scale_x)
            y1 = max(0.0, (cy - h * 0.5) * scale_y)
            x2 = min(float(width - 1), (cx + w * 0.5) * scale_x)
            y2 = min(float(height - 1), (cy + h * 0.5) * scale_y)
            detections.append(Detection((x1, y1, x2, y2), score, class_id, str(class_id)))
        return detections

    def normalize_yolo_output(self, output):
        output = np.squeeze(output)
        if output.ndim != 2:
            return []
        if output.shape[0] < output.shape[1] and output.shape[0] <= 256:
            output = output.T
        return output


class CenterFallbackDetector:
    def __init__(self, confidence):
        self.confidence = confidence

    def detect(self, bgr):
        height, width = bgr.shape[:2]
        box_w = width * 0.25
        box_h = height * 0.25
        cx = width * 0.5
        cy = height * 0.5
        return [
            Detection(
                (cx - box_w * 0.5, cy - box_h * 0.5, cx + box_w * 0.5, cy + box_h * 0.5),
                self.confidence,
                -1,
                "center_smoke_test",
            )
        ]


class HsvColorDetector:
    def __init__(self, args):
        self.args = args

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([self.args.h_min, self.args.s_min, self.args.v_min], dtype=np.uint8)
        upper = np.array([self.args.h_max, self.args.s_max, self.args.v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), dtype=np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.args.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            detections.append(Detection((x, y, x + w, y + h), 1.0, -2, "hsv_object"))
        return detections


class DepthAiYoloPosePublisher(Node):
    def __init__(self, args):
        super().__init__("depthai_yolo_pose_publisher")
        self.args = args
        self.publisher = self.create_publisher(PoseStamped, args.output_topic, 10)
        self.detector = self.create_detector(args)
        self.last_publish = 0.0
        self.last_depth_warn = 0.0
        self.last_pose_log = 0.0
        self.arm_approach_camera = self.compute_arm_approach_camera()
        self.get_logger().info(
            f"Publishing detected object poses on {args.output_topic}; detector={args.detector}"
        )

    def compute_arm_approach_camera(self):
        base_motion = normalize_vector(
            [
                self.args.approach_base_x,
                self.args.approach_base_y,
                self.args.approach_base_z,
            ]
        )
        q = quaternion_from_euler(self.args.camera_roll, self.args.camera_pitch, self.args.camera_yaw)
        rotation_base_camera = quaternion_matrix(q)
        return normalize_vector(
            [
                sum(rotation_base_camera[row][col] * base_motion[row] for row in range(3))
                for col in range(3)
            ]
        )

    def create_detector(self, args):
        if args.detector == "ultralytics":
            if not args.yolo_model:
                raise RuntimeError("--yolo-model is required for --detector ultralytics")
            return UltralyticsYoloDetector(args.yolo_model, args.target_class, args.confidence)
        if args.detector == "opencv_onnx":
            if not args.yolo_model:
                raise RuntimeError("--yolo-model must point to a YOLO ONNX file")
            return OpenCvOnnxYoloDetector(
                args.yolo_model,
                args.target_class_id,
                args.confidence,
                args.yolo_input_size,
            )
        if args.detector == "hsv":
            return HsvColorDetector(args)
        self.get_logger().warn(
            "--detector center is a smoke test only. It always targets the image center; "
            "use --detector hsv, ultralytics, or opencv_onnx for actual object detection."
        )
        return CenterFallbackDetector(args.confidence)

    def run(self):
        import depthai as dai

        try:
            with dai.Pipeline() as pipeline:
                bgr_queue, depth_queue = self.build_depthai_pipeline(dai, pipeline)
                pipeline.start()
                self.get_logger().info("DepthAI pipeline started. Press q or Esc in the image window to quit.")

                while rclpy.ok() and pipeline.isRunning():
                    rgb_msg = bgr_queue.get()
                    frame = rgb_msg.getCvFrame()
                    if self.args.frame_color_mode == "rgb":
                        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    else:
                        bgr = frame
                    depth = None
                    if depth_queue is not None:
                        depth_msg = depth_queue.tryGet()
                        if depth_msg is not None:
                            depth = depth_msg.getFrame()

                    try:
                        detections = self.detector.detect(bgr)
                    except KeyboardInterrupt:
                        break
                    selected = self.select_detection(detections)
                    selected_xyz = None
                    if selected is not None:
                        selected_xyz = self.publish_detection_pose(selected, bgr.shape, depth)
                    vis = bgr.copy()
                    self.draw_detections(vis, detections, selected, selected_xyz)

                    cv2.imshow(self.args.window_name, vis)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    try:
                        rclpy.spin_once(self, timeout_sec=0.001)
                    except Exception:
                        break
        except KeyboardInterrupt:
            pass
        except RuntimeError as exc:
            self.get_logger().error(f"DepthAI runtime error: {exc}")
        finally:
            cv2.destroyAllWindows()

    def build_depthai_pipeline(self, dai, pipeline):
        stereo_width = max(16, int(self.args.width) - (int(self.args.width) % 16))
        stereo_height = int(self.args.height)
        cam = pipeline.create(dai.node.Camera).build()
        rgb_output = cam.requestOutput(
            (stereo_width, stereo_height),
            type=dai.ImgFrame.Type.RGB888p,
            fps=self.args.fps,
        )
        rgb_queue = rgb_output.createOutputQueue(maxSize=2, blocking=False)

        depth_queue = None
        if self.args.depth_source == "stereo":
            try:
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
                stereo.setLeftRightCheck(self.args.left_right_check)
                stereo.setSubpixel(self.args.subpixel)
                stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
                stereo.setOutputKeepAspectRatio(False)
                stereo.setOutputSize(stereo_width, stereo_height)
                left.out.link(stereo.left)
                right.out.link(stereo.right)
                depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
            except Exception as exc:
                raise RuntimeError(f"Could not create stereo depth pipeline: {exc}") from exc
        return rgb_queue, depth_queue

    def select_detection(self, detections):
        if not detections:
            return None
        return max(detections, key=lambda det: det.score)

    def draw_detections(self, image, detections, selected, selected_xyz=None):
        if not detections:
            target = self.args.target_class if self.args.target_class else "any class"
            text = f"No {target} detection above conf={self.args.confidence:.2f}"
            cv2.putText(
                image,
                text,
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 180, 255),
                2,
            )
            return

        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det.xyxy]
            color = (0, 255, 0) if det is selected else (180, 180, 180)
            thickness = 4 if det is selected else 2
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
            label = f"{det.label} {det.score:.2f}"
            cv2.putText(image, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cx, cy = det.center
            cv2.circle(image, (cx, cy), 6 if det is selected else 4, color, -1)
            if det is selected and selected_xyz is not None:
                _, _, x, y, z, depth_mode = selected_xyz
                xyz_label = f"xyz=({x:.2f},{y:.2f},{z:.2f}) {depth_mode}"
                cv2.putText(
                    image,
                    xyz_label,
                    (x1, min(image.shape[0] - 12, y2 + 24)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )
                self.draw_arm_approach(image, x, y, z)

    def draw_arm_approach(self, image, x, y, z):
        origin = self.project_camera_xyz(x, y, z, image.shape)
        if origin is None:
            return
        axis_len = self.args.axis_length_m
        dx, dy, dz = self.arm_approach_camera
        start = self.project_camera_xyz(
            x - dx * axis_len,
            y - dy * axis_len,
            z - dz * axis_len,
            image.shape,
        )
        if start is not None:
            cv2.arrowedLine(
                image,
                start,
                origin,
                (255, 0, 0),
                self.args.axis_thickness,
                tipLength=0.18,
            )
            cv2.putText(
                image,
                "arm approach",
                start,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
            )

    def project_camera_xyz(self, x, y, z, image_shape):
        if z <= 1e-6:
            return None
        height, width = image_shape[:2]
        fx = self.args.fx if self.args.fx > 0 else width / (2.0 * math.tan(math.radians(self.args.fov_deg) * 0.5))
        fy = self.args.fy if self.args.fy > 0 else fx
        cx = self.args.cx if self.args.cx >= 0 else width * 0.5
        cy = self.args.cy if self.args.cy >= 0 else height * 0.5
        u = int(round((x * fx / z) + cx))
        v = int(round((y * fy / z) + cy))
        if u < -width or u > 2 * width or v < -height or v > 2 * height:
            return None
        return max(0, min(width - 1, u)), max(0, min(height - 1, v))

    def publish_detection_pose(self, detection, image_shape, depth):
        now = time.time()
        if now - self.last_publish < 1.0 / self.args.publish_rate:
            return
        self.last_publish = now

        xyz = self.detection_to_camera_xyz(detection, image_shape, depth)
        if xyz is None:
            return None
        u, v, x, y, z, depth_mode = xyz

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
        self.log_pose_throttled(detection, u, v, x, y, z, depth_mode)
        return xyz

    def detection_to_camera_xyz(self, detection, image_shape, depth):
        u, v = detection.center
        z = self.depth_at_detection(u, v, image_shape, depth)
        depth_mode = self.args.depth_source
        if z is None:
            if self.args.depth_source == "fixed" or self.args.allow_fixed_depth_fallback:
                z = self.args.fixed_depth_m
                depth_mode = "fixed" if self.args.depth_source == "fixed" else "fixed_fallback"
                self.warn_depth_throttled(
                    f"Using fixed depth fallback z={z:.3f} m for {detection.label}"
                )
            else:
                self.warn_depth_throttled(
                    f"No valid RGBD depth for {detection.label}; not publishing pose this frame"
                )
                return None

        x, y, z = self.pixel_to_camera_xyz(u, v, image_shape, z)
        if not (self.args.min_depth_m <= z <= self.args.max_depth_m):
            self.warn_depth_throttled(
                f"Rejected {detection.label}: depth {z:.3f} m outside "
                f"[{self.args.min_depth_m:.3f}, {self.args.max_depth_m:.3f}]"
            )
            return None
        return u, v, x, y, z, depth_mode

    def warn_depth_throttled(self, message):
        now = time.time()
        if now - self.last_depth_warn >= self.args.depth_warn_period:
            self.last_depth_warn = now
            self.get_logger().warn(message)

    def depth_at_detection(self, u, v, image_shape, depth):
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
        low, high = np.percentile(z_values, [self.args.depth_low_percentile, self.args.depth_high_percentile])
        z_values = z_values[(z_values >= low) & (z_values <= high)]
        if z_values.size == 0:
            return None
        return float(np.median(z_values))

    def pixel_to_camera_xyz(self, u, v, image_shape, z):
        height, width = image_shape[:2]
        fx = self.args.fx if self.args.fx > 0 else width / (2.0 * math.tan(math.radians(self.args.fov_deg) * 0.5))
        fy = self.args.fy if self.args.fy > 0 else fx
        cx = self.args.cx if self.args.cx >= 0 else width * 0.5
        cy = self.args.cy if self.args.cy >= 0 else height * 0.5
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z

    def log_pose_throttled(self, detection, u, v, x, y, z, depth_mode):
        now = time.time()
        if now - self.last_pose_log < self.args.pose_log_period:
            return
        self.last_pose_log = now
        x1, y1, x2, y2 = detection.xyxy
        self.get_logger().info(
            f"Selected {detection.label} score={detection.score:.2f} "
            f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}) pixel=({u},{v}) "
            f"camera_xyz=({x:.3f},{y:.3f},{z:.3f}) depth={depth_mode}"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="DepthAI live YOLO RGBD object detector that publishes object PoseStamped."
    )
    parser.add_argument("--output-topic", default="/perception/object_pose_camera")
    parser.add_argument("--camera-frame", default="camera_color_optical_frame")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--window-name", default="DepthAI YOLO RGBD Perception")
    parser.add_argument(
        "--frame-color-mode",
        choices=["bgr", "rgb"],
        default="bgr",
        help="Use bgr when DepthAI getCvFrame() already returns OpenCV BGR; use rgb to swap RGB into BGR.",
    )
    parser.add_argument("--detector", choices=["ultralytics", "opencv_onnx", "hsv", "center"], default="ultralytics")
    parser.add_argument("--yolo-model", default="")
    parser.add_argument("--target-class", default="")
    parser.add_argument("--target-class-id", type=int, default=None)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--yolo-input-size", type=int, default=640)
    parser.add_argument("--depth-source", choices=["stereo", "fixed"], default="stereo")
    parser.add_argument("--fixed-depth-m", type=float, default=0.45)
    parser.add_argument("--allow-fixed-depth-fallback", action="store_true")
    parser.add_argument("--depth-window", type=int, default=12)
    parser.add_argument("--min-depth-pixels", type=int, default=10)
    parser.add_argument("--min-depth-m", type=float, default=0.12)
    parser.add_argument("--max-depth-m", type=float, default=2.00)
    parser.add_argument("--depth-low-percentile", type=float, default=10.0)
    parser.add_argument("--depth-high-percentile", type=float, default=90.0)
    parser.add_argument("--depth-warn-period", type=float, default=1.0)
    parser.add_argument("--pose-log-period", type=float, default=1.0)
    parser.add_argument("--axis-length-m", type=float, default=0.16)
    parser.add_argument("--axis-thickness", type=int, default=8)
    parser.add_argument("--camera-roll", type=float, default=-1.57079632679)
    parser.add_argument("--camera-pitch", type=float, default=0.0)
    parser.add_argument("--camera-yaw", type=float, default=-1.57079632679)
    parser.add_argument("--approach-base-x", type=float, default=0.0)
    parser.add_argument("--approach-base-y", type=float, default=0.0)
    parser.add_argument("--approach-base-z", type=float, default=-1.0)
    parser.add_argument(
        "--stereo-preset",
        choices=["DEFAULT", "DENSITY", "FAST_DENSITY", "ROBOTICS", "HIGH_DETAIL"],
        default="FAST_DENSITY",
    )
    parser.add_argument("--subpixel", action="store_true")
    parser.add_argument("--left-right-check", action="store_true", default=True)
    parser.add_argument("--no-left-right-check", dest="left_right_check", action="store_false")
    parser.add_argument("--publish-rate", type=float, default=10.0)
    parser.add_argument("--fov-deg", type=float, default=69.0)
    parser.add_argument("--fx", type=float, default=-1.0)
    parser.add_argument("--fy", type=float, default=-1.0)
    parser.add_argument("--cx", type=float, default=-1.0)
    parser.add_argument("--cy", type=float, default=-1.0)
    parser.add_argument("--h-min", type=int, default=0)
    parser.add_argument("--h-max", type=int, default=20)
    parser.add_argument("--s-min", type=int, default=80)
    parser.add_argument("--s-max", type=int, default=255)
    parser.add_argument("--v-min", type=int, default=80)
    parser.add_argument("--v-max", type=int, default=255)
    parser.add_argument("--min-area", type=float, default=300.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DepthAiYoloPosePublisher(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
