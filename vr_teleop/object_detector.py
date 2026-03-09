"""
Object Detector — Color-Based Green Object Detection
=====================================================
Subscribes to the wrist camera feed, detects green objects via HSV color
segmentation, and publishes their estimated 3D position.

Topics:
  Subscribes: /wrist_camera/color/image_raw  (sensor_msgs/Image)
  Publishes:  /detected_object/position       (geometry_msgs/PointStamped)
              /detected_object/debug_image     (sensor_msgs/Image, annotated)

Usage:
  python object_detector.py
  python object_detector.py --z-height 50   # Object surface height in mm
"""
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import numpy as np
import cv2


# Camera intrinsics (must match wrist_camera_sim.py)
CAM_FX = 500.0
CAM_FY = 500.0


# HSV range for green detection (tuned for the RViz green cube)
GREEN_LOWER = np.array([35, 80, 80])
GREEN_UPPER = np.array([85, 255, 255])

# Minimum contour area to count as a detection (pixels)
MIN_CONTOUR_AREA = 200


class ObjectDetector(Node):
    def __init__(self, assumed_z_mm=50.0):
        super().__init__('object_detector')

        self.assumed_z_mm = assumed_z_mm

        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image, '/wrist_camera/color/image_raw',
            self.image_callback, 10)

        # Publish detected object position
        self.pos_pub = self.create_publisher(
            PointStamped, '/detected_object/position', 10)

        # Publish annotated debug image
        self.debug_pub = self.create_publisher(
            Image, '/detected_object/debug_image', 10)

        self._last_log = 0.0
        self.get_logger().info(
            f'Object detector started (assumed Z={assumed_z_mm}mm)')
        self.get_logger().info('  Subscribing to: /wrist_camera/color/image_raw')
        self.get_logger().info('  Publishing to:  /detected_object/position')

    def image_callback(self, msg):
        import time

        # Decode ROS Image → numpy
        h, w = msg.height, msg.width
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        else:
            return

        # HSV color segmentation for green
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        cx_px, cy_px = 0, 0

        if contours:
            # Find the largest green contour
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > MIN_CONTOUR_AREA:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx_px = int(M['m10'] / M['m00'])
                    cy_px = int(M['m01'] / M['m00'])
                    detected = True

                    # Pixel → 3D using pinhole model
                    # Assume object is at known Z height from camera
                    # For now, use a fixed depth estimate
                    # X_mm = (cx_px - w/2) * Z_est / fx
                    # Y_mm = (cy_px - h/2) * Z_est / fy
                    #
                    # Since the camera is on the wrist, the transform
                    # to world frame depends on the arm's FK.
                    # For Phase 1, we publish in camera frame and let
                    # the bridge handle the transform.

                    # Estimated depth from camera to object (mm)
                    # This is a rough estimate — replaced by real depth in Phase 2
                    z_est = 300.0  # mm (rough workspace distance)
                    x_cam = (cx_px - w / 2.0) * z_est / CAM_FX
                    y_cam = (cy_px - h / 2.0) * z_est / CAM_FY

                    # Publish position in camera frame
                    pt = PointStamped()
                    pt.header = Header()
                    pt.header.stamp = self.get_clock().now().to_msg()
                    pt.header.frame_id = 'camera_optical_frame'
                    pt.point.x = x_cam / 1000.0  # mm → meters
                    pt.point.y = y_cam / 1000.0
                    pt.point.z = z_est / 1000.0
                    self.pos_pub.publish(pt)

        # Debug image: draw detection
        debug = bgr.copy()
        if detected:
            cv2.drawContours(debug, [largest], -1, (0, 255, 0), 2)
            cv2.circle(debug, (cx_px, cy_px), 6, (0, 0, 255), -1)
            cv2.putText(debug, f'({x_cam:.0f}, {y_cam:.0f})mm',
                        (cx_px + 10, cy_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            cv2.putText(debug, 'No green object detected',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

        # Publish debug image
        debug_rgb = cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)
        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = h
        debug_msg.width = w
        debug_msg.encoding = 'rgb8'
        debug_msg.is_bigendian = False
        debug_msg.step = w * 3
        debug_msg.data = debug_rgb.tobytes()
        self.debug_pub.publish(debug_msg)

        # Periodic logging
        now = time.time()
        if now - self._last_log > 2.0:
            self._last_log = now
            if detected:
                self.get_logger().info(
                    f'Detected green object at pixel ({cx_px},{cy_px}), '
                    f'cam frame ({x_cam:.0f},{y_cam:.0f},{z_est:.0f})mm')
            else:
                self.get_logger().info('No green object detected')


def main(args=None):
    parser = argparse.ArgumentParser(description='Green object detector')
    parser.add_argument('--z-height', type=float, default=50.0,
                        help='Assumed object surface height in mm')
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = ObjectDetector(assumed_z_mm=parsed.z_height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
