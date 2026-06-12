#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from piper_msgs.msg import PosCmd
from piper_msgs.srv import Enable
from rclpy.node import Node


def euler_from_quaternion(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Stage0GraspExecutor(Node):
    def __init__(self, args):
        super().__init__("stage0_grasp_executor")
        self.args = args
        self.latest_pose = None
        self.command_pub = self.create_publisher(PosCmd, args.pos_cmd_topic, 10)
        self.enable_client = self.create_client(Enable, args.enable_service)
        self.ik_client = self.create_client(GetPositionIK, args.ik_service)
        self.create_subscription(PoseStamped, args.object_pose_topic, self.pose_callback, 10)
        self.latest_end_pose = None
        self.create_subscription(Pose, args.end_pose_topic, self.end_pose_callback, 10)
        self.get_logger().info(
            f"Waiting for object pose on {args.object_pose_topic}; backend=pos_cmd"
        )

    def pose_callback(self, msg):
        self.latest_pose = msg

    def end_pose_callback(self, msg):
        self.latest_end_pose = msg

    def wait_for_pose(self):
        start = time.time()
        while rclpy.ok() and self.latest_pose is None:
            if time.time() - start > self.args.pose_timeout:
                raise TimeoutError(f"No object pose received on {self.args.object_pose_topic}")
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_pose

    def wait_for_end_pose(self):
        start = time.time()
        while rclpy.ok() and self.latest_end_pose is None:
            if time.time() - start > self.args.end_pose_timeout:
                raise TimeoutError(
                    f"No current arm pose received on {self.args.end_pose_topic}. "
                    "Start the Piper driver and verify CAN feedback before executing motion."
                )
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_end_pose

    def make_pose_stamped(self, source, x, y, z, q):
        pose = PoseStamped()
        pose.header = source.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def validate_workspace_pose(self, name, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        radius = math.hypot(x, y)
        errors = []
        if not (self.args.min_radius <= radius <= self.args.max_radius):
            errors.append(
                f"radius {radius:.3f} outside [{self.args.min_radius:.3f}, {self.args.max_radius:.3f}]"
            )
        if not (self.args.min_z <= z <= self.args.max_z):
            errors.append(f"z {z:.3f} outside [{self.args.min_z:.3f}, {self.args.max_z:.3f}]")
        if abs(x) > self.args.max_abs_x:
            errors.append(f"|x| {abs(x):.3f} > {self.args.max_abs_x:.3f}")
        if abs(y) > self.args.max_abs_y:
            errors.append(f"|y| {abs(y):.3f} > {self.args.max_abs_y:.3f}")
        if errors:
            raise ValueError(f"{name} rejected by workspace guard: " + "; ".join(errors))

    def check_ik_pose(self, name, pose):
        if not self.args.check_ik:
            return
        if not self.ik_client.wait_for_service(timeout_sec=self.args.ik_service_timeout):
            raise RuntimeError(
                f"IK service {self.args.ik_service} is unavailable. Start MoveIt move_group or omit --check-ik."
            )

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.args.group_name
        request.ik_request.ik_link_name = self.args.end_effector_link
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = self.args.avoid_collisions
        request.ik_request.timeout.sec = int(self.args.ik_timeout)
        request.ik_request.timeout.nanosec = int((self.args.ik_timeout % 1.0) * 1e9)

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.ik_service_timeout)
        result = future.result()
        if result is None:
            raise RuntimeError(f"IK service returned no result for {name}")
        if result.error_code.val != 1:
            raise ValueError(f"{name} rejected by MoveIt IK; error_code={result.error_code.val}")

    def validate_motion_poses(self, named_poses):
        for name, pose in named_poses:
            self.validate_workspace_pose(name, pose)
            self.check_ik_pose(name, pose)
            self.get_logger().info(f"{name} passed reachability checks")

    def validate_initial_motion(self, first_pose):
        if self.args.allow_large_motion:
            return
        current = self.wait_for_end_pose()
        dx = first_pose.pose.position.x - current.position.x
        dy = first_pose.pose.position.y - current.position.y
        dz = first_pose.pose.position.z - current.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > self.args.max_initial_translation:
            raise ValueError(
                "Initial pregrasp move rejected: "
                f"distance {distance:.3f} m exceeds --max-initial-translation "
                f"{self.args.max_initial_translation:.3f} m. Current end_pose is "
                f"({current.position.x:.3f}, {current.position.y:.3f}, {current.position.z:.3f}); "
                f"pregrasp target is ({first_pose.pose.position.x:.3f}, "
                f"{first_pose.pose.position.y:.3f}, {first_pose.pose.position.z:.3f}). "
                "Move the arm near the test pose first, or rerun with --allow-large-motion only after checking clearance."
            )
        self.get_logger().info(f"Initial move guard passed: pregrasp delta={distance:.3f} m")

    def enable_arm(self):
        if not self.args.enable:
            return
        if not self.enable_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f"Enable service not available: {self.args.enable_service}. "
                "Start the Piper hardware driver, or omit --enable for perception/planning smoke tests."
            )
        request = Enable.Request()
        request.enable_request = True
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.service_timeout)
        if future.result() is None or not future.result().enable_response:
            raise RuntimeError("Arm enable request failed")

    def publish_pose(self, name, x, y, z, roll, pitch, yaw, gripper, hold_sec):
        start_feedback = self.latest_end_pose
        msg = PosCmd()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)
        msg.gripper = float(gripper)
        msg.mode1 = 0
        msg.mode2 = 0

        end = time.time() + hold_sec
        interval = 1.0 / max(self.args.command_rate, 0.1)
        while rclpy.ok() and time.time() < end:
            self.command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(interval)
        self.log_motion_feedback(name, start_feedback)

    def log_motion_feedback(self, name, start_feedback):
        end_feedback = self.latest_end_pose
        if end_feedback is None:
            self.get_logger().warn(f"{name}: no /end_pose feedback received")
            return
        if start_feedback is None:
            self.get_logger().info(
                f"{name}: end_pose now x={end_feedback.position.x:.3f}, "
                f"y={end_feedback.position.y:.3f}, z={end_feedback.position.z:.3f}"
            )
            return
        dx = end_feedback.position.x - start_feedback.position.x
        dy = end_feedback.position.y - start_feedback.position.y
        dz = end_feedback.position.z - start_feedback.position.z
        self.get_logger().info(
            f"{name}: end_pose delta dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}; "
            f"now x={end_feedback.position.x:.3f}, y={end_feedback.position.y:.3f}, z={end_feedback.position.z:.3f}"
        )

    def execute_once(self):
        pose_msg = self.wait_for_pose()
        current_pose = self.wait_for_end_pose() if self.args.require_end_pose else None

        pose = pose_msg.pose
        roll, pitch, yaw = euler_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        if self.args.preserve_current_orientation and current_pose is not None:
            roll, pitch, yaw = current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z
            q = quaternion_from_euler(roll, pitch, yaw)
        elif self.args.force_top_down:
            roll = self.args.roll
            pitch = self.args.pitch
            yaw = self.args.yaw
            q = quaternion_from_euler(roll, pitch, yaw)
        else:
            q = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

        grasp_x = pose.position.x + self.args.grasp_dx
        grasp_y = pose.position.y + self.args.grasp_dy
        grasp_z = pose.position.z + self.args.grasp_dz
        pre_z = grasp_z + self.args.pregrasp_height
        lift_z = grasp_z + self.args.lift_height
        named_poses = [
            ("pregrasp", self.make_pose_stamped(pose_msg, grasp_x, grasp_y, pre_z, q)),
            ("grasp", self.make_pose_stamped(pose_msg, grasp_x, grasp_y, grasp_z, q)),
            ("lift", self.make_pose_stamped(pose_msg, grasp_x, grasp_y, lift_z, q)),
        ]
        self.validate_motion_poses(named_poses)
        self.validate_initial_motion(named_poses[0][1])
        self.enable_arm()

        self.get_logger().info(
            f"Executing grasp at ({grasp_x:.3f}, {grasp_y:.3f}, {grasp_z:.3f}); "
            f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f}), command_rate={self.args.command_rate:.1f} Hz"
        )
        self.publish_pose("pregrasp", grasp_x, grasp_y, pre_z, roll, pitch, yaw, self.args.open_gripper, self.args.step_time)
        self.publish_pose("grasp", grasp_x, grasp_y, grasp_z, roll, pitch, yaw, self.args.open_gripper, self.args.step_time)
        self.publish_pose("close", grasp_x, grasp_y, grasp_z, roll, pitch, yaw, self.args.closed_gripper, self.args.close_time)
        self.publish_pose("lift", grasp_x, grasp_y, lift_z, roll, pitch, yaw, self.args.closed_gripper, self.args.step_time)
        self.get_logger().info("Grasp sequence complete")


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


def parse_args():
    parser = argparse.ArgumentParser(
        description="Layer 1 Stage 0 grasp executor: pose -> pregrasp -> grasp -> close -> lift."
    )
    parser.add_argument("--object-pose-topic", default="/perception/object_pose_base")
    parser.add_argument("--pos-cmd-topic", default="/pos_cmd")
    parser.add_argument("--end-pose-topic", default="/end_pose")
    parser.add_argument("--enable-service", default="/enable_srv")
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--enable", action="store_true")
    parser.add_argument("--check-ik", action="store_true")
    parser.add_argument("--avoid-collisions", action="store_true")
    parser.add_argument("--group-name", default="piper_arm")
    parser.add_argument("--end-effector-link", default="gripper_base")
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--ik-service-timeout", type=float, default=5.0)
    parser.add_argument("--ik-timeout", type=float, default=0.5)
    parser.add_argument("--pose-timeout", type=float, default=10.0)
    parser.add_argument("--end-pose-timeout", type=float, default=5.0)
    parser.add_argument("--command-rate", type=float, default=5.0)
    parser.add_argument("--step-time", type=float, default=2.0)
    parser.add_argument("--close-time", type=float, default=1.0)
    parser.add_argument("--pregrasp-height", type=float, default=0.10)
    parser.add_argument("--lift-height", type=float, default=0.12)
    parser.add_argument("--grasp-dx", type=float, default=0.0)
    parser.add_argument("--grasp-dy", type=float, default=0.0)
    parser.add_argument("--grasp-dz", type=float, default=0.02)
    parser.add_argument("--open-gripper", type=float, default=0.08)
    parser.add_argument("--closed-gripper", type=float, default=0.0)
    parser.add_argument("--require-end-pose", action="store_true", default=True)
    parser.add_argument("--no-require-end-pose", dest="require_end_pose", action="store_false")
    parser.add_argument("--preserve-current-orientation", action="store_true", default=True)
    parser.add_argument("--use-command-orientation", dest="preserve_current_orientation", action="store_false")
    parser.add_argument("--force-top-down", action="store_true", default=True)
    parser.add_argument("--use-pose-orientation", dest="force_top_down", action="store_false")
    parser.add_argument("--roll", type=float, default=math.radians(-179.9))
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=math.radians(-179.9))
    parser.add_argument("--max-initial-translation", type=float, default=0.08)
    parser.add_argument("--allow-large-motion", action="store_true")
    parser.add_argument("--min-radius", type=float, default=0.12)
    parser.add_argument("--max-radius", type=float, default=0.62)
    parser.add_argument("--min-z", type=float, default=0.02)
    parser.add_argument("--max-z", type=float, default=0.55)
    parser.add_argument("--max-abs-x", type=float, default=0.62)
    parser.add_argument("--max-abs-y", type=float, default=0.45)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Stage0GraspExecutor(args)
    try:
        node.execute_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
