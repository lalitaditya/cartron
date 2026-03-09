"""
Wrist Camera Publisher — Live or Synthetic Feed
================================================
Publishes camera data as ROS2 topics for the wrist-mounted camera.

Modes:
  --camera N       Use physical USB camera at index N
  --synthetic      Use synthetic placeholder images (no camera needed)

Topics published:
  /wrist_camera/color/image_raw    (sensor_msgs/Image, RGB8)
  /wrist_camera/depth/image_raw    (sensor_msgs/Image, 16UC1 placeholder)
  /wrist_camera/color/camera_info  (sensor_msgs/CameraInfo)

Usage:
  python wrist_camera_sim.py --synthetic      # No camera needed
  python wrist_camera_sim.py --camera 1       # USB camera
  python wrist_camera_sim.py --camera 0       # Built-in webcam
"""
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import numpy as np

# Camera intrinsics (generic 640x480, ~65° HFOV)
CAM_FX = 500.0
CAM_FY = 500.0

WIDTH  = 640
HEIGHT = 480


class WristCameraPublisher(Node):
    def __init__(self, camera_index=None):
        super().__init__('wrist_camera_sim')

        self._width = WIDTH
        self._height = HEIGHT
        self.cap = None
        self._synthetic = camera_index is None

        if not self._synthetic:
            import cv2
            self.cv2 = cv2
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened():
                self.get_logger().warn(
                    f'Camera {camera_index} not found, falling back to synthetic')
                self._synthetic = True
                self.cap = None
            else:
                ret, frame = self.cap.read()
                if ret:
                    self._height, self._width = frame.shape[:2]
                self.get_logger().info(
                    f'Camera {camera_index}: {self._width}x{self._height}')

        if self._synthetic:
            self._synth_color = self._make_synthetic_image()
            self.get_logger().info(
                f'Synthetic mode: {self._width}x{self._height}')

        # Publishers
        self.color_pub = self.create_publisher(
            Image, '/wrist_camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(
            Image, '/wrist_camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, '/wrist_camera/color/camera_info', 10)

        # Placeholder depth (uniform 500mm)
        self._depth_data = np.full(
            (self._height, self._width), 500, dtype=np.uint16).tobytes()

        # Publish at 10 Hz (synthetic) or 15 Hz (live)
        rate = 10.0 if self._synthetic else 15.0
        self.timer = self.create_timer(1.0 / rate, self.publish_frames)

        mode = 'synthetic' if self._synthetic else 'live'
        self.get_logger().info(f'Publishing ({mode}):')
        self.get_logger().info('  /wrist_camera/color/image_raw')
        self.get_logger().info('  /wrist_camera/depth/image_raw')
        self.get_logger().info('  /wrist_camera/color/camera_info')

    @staticmethod
    def _make_synthetic_image():
        """Black background — lets the RViz Camera overlay show through."""
        img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        return img.tobytes()

    def _make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_optical_frame'
        return header

    def _build_camera_info(self, header):
        info = CameraInfo()
        info.header = header
        info.width = self._width
        info.height = self._height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        cx = self._width / 2.0
        cy = self._height / 2.0
        info.k = [CAM_FX, 0.0, cx, 0.0, CAM_FY, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [CAM_FX, 0.0, cx, 0.0, 0.0, CAM_FY, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def publish_frames(self):
        header = self._make_header()

        if self._synthetic:
            color_data = self._synth_color
        else:
            ret, frame = self.cap.read()
            if not ret:
                return
            rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
            color_data = rgb.tobytes()

        # Color image
        color_msg = Image()
        color_msg.header = header
        color_msg.height = self._height
        color_msg.width = self._width
        color_msg.encoding = 'rgb8'
        color_msg.is_bigendian = False
        color_msg.step = self._width * 3
        color_msg.data = color_data
        self.color_pub.publish(color_msg)

        # Depth image (placeholder)
        depth_msg = Image()
        depth_msg.header = header
        depth_msg.height = self._height
        depth_msg.width = self._width
        depth_msg.encoding = '16UC1'
        depth_msg.is_bigendian = False
        depth_msg.step = self._width * 2
        depth_msg.data = self._depth_data
        self.depth_pub.publish(depth_msg)

        # Camera info
        self.info_pub.publish(self._build_camera_info(header))

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='Wrist camera publisher')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--camera', type=int, default=None,
                       help='Camera index (e.g. 0=webcam, 1=USB)')
    group.add_argument('--synthetic', action='store_true',
                       help='Use synthetic placeholder images (no camera)')
    parsed = parser.parse_args()

    camera_index = parsed.camera
    if parsed.synthetic or camera_index is None:
        camera_index = None  # synthetic mode

    rclpy.init(args=args)
    node = WristCameraPublisher(camera_index=camera_index)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
