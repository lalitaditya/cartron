#!/usr/bin/env python3

import argparse
import math
import time
from collections import deque

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from piper_msgs.msg import PosCmd
from piper_msgs.srv import Enable
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


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


def euler_from_quaternion(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class MoveItGraspExecutor(Node):
    def __init__(self, args):
        super().__init__("moveit_grasp_executor")
        self.args = args
        self.latest_pose = None
        self.latest_end_pose = None
        self.pose_buffer = deque(maxlen=max(1, args.stable_samples))
        self.create_subscription(PoseStamped, args.object_pose_topic, self.pose_callback, 10)
        self.create_subscription(Pose, args.end_pose_topic, self.end_pose_callback, 10)
        self.move_group = ActionClient(self, MoveGroup, args.move_group_action)
        self.gripper_action = ActionClient(
            self,
            FollowJointTrajectory,
            args.gripper_action,
        )
        self.trajectory_pub = self.create_publisher(JointTrajectory, args.trajectory_topic, 10)
        self.pos_cmd_pub = self.create_publisher(PosCmd, args.pos_cmd_topic, 10)
        self.enable_client = self.create_client(Enable, args.enable_service)
        self.get_logger().info(
            f"Waiting for object pose on {args.object_pose_topic}; planner=MoveIt action={args.move_group_action}"
        )

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.pose_buffer.append(msg)

    def end_pose_callback(self, msg):
        self.latest_end_pose = msg

    def wait_for_pose(self):
        start = time.time()
        while rclpy.ok() and self.latest_pose is None:
            if time.time() - start > self.args.pose_timeout:
                raise TimeoutError(f"No object pose received on {self.args.object_pose_topic}")
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_pose

    def wait_for_stable_pose(self):
        start = time.time()
        last_unstable_log = 0.0
        while rclpy.ok():
            if time.time() - start > self.args.pose_timeout:
                raise TimeoutError(
                    f"No stable object pose received on {self.args.object_pose_topic}. "
                    f"Need {self.args.stable_samples} samples within "
                    f"{self.args.stable_position_tolerance:.3f} m."
                )
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.pose_buffer) < self.args.stable_samples:
                continue
            samples = list(self.pose_buffer)
            xs = [msg.pose.position.x for msg in samples]
            ys = [msg.pose.position.y for msg in samples]
            zs = [msg.pose.position.z for msg in samples]
            spread = max(
                math.sqrt(
                    (xs[i] - xs[j]) ** 2
                    + (ys[i] - ys[j]) ** 2
                    + (zs[i] - zs[j]) ** 2
                )
                for i in range(len(samples))
                for j in range(i + 1, len(samples))
            )
            if spread <= self.args.stable_position_tolerance:
                stable = samples[-1]
                self.get_logger().info(
                    f"Stable object pose locked after {len(samples)} samples: "
                    f"base_xyz=({stable.pose.position.x:.3f}, {stable.pose.position.y:.3f}, "
                    f"{stable.pose.position.z:.3f}), spread={spread:.3f} m"
                )
                return stable
            now = time.time()
            if now - last_unstable_log >= self.args.stability_log_period:
                last_unstable_log = now
                newest = samples[-1]
                self.get_logger().warn(
                    f"Waiting for stable object pose: spread={spread:.3f} m > "
                    f"{self.args.stable_position_tolerance:.3f} m; newest base_xyz="
                    f"({newest.pose.position.x:.3f}, {newest.pose.position.y:.3f}, "
                    f"{newest.pose.position.z:.3f})"
                )
        raise RuntimeError("ROS shutdown while waiting for stable object pose")

    def enable_arm(self):
        if not self.args.enable:
            return
        if not self.enable_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise RuntimeError(
                f"Enable service not available: {self.args.enable_service}. "
                "Start the Piper hardware driver, or omit --enable for planning-only tests."
            )
        request = Enable.Request()
        request.enable_request = True
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.service_timeout)
        if future.result() is None or not future.result().enable_response:
            raise RuntimeError("Arm enable request failed")

    def make_pose(self, source, x, y, z, q):
        x, y, z = self.apply_tool_tip_offset(x, y, z, q)
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

    def apply_tool_tip_offset(self, tip_x, tip_y, tip_z, q):
        offset = [
            self.args.tool_tip_offset_x,
            self.args.tool_tip_offset_y,
            self.args.tool_tip_offset_z,
        ]
        if max(abs(v) for v in offset) < 1e-9:
            return tip_x, tip_y, tip_z
        rotation = quaternion_matrix(q)
        rotated_offset = [
            sum(rotation[row][col] * offset[col] for col in range(3))
            for row in range(3)
        ]
        return (
            tip_x - rotated_offset[0],
            tip_y - rotated_offset[1],
            tip_z - rotated_offset[2],
        )

    def tip_from_ee_pose(self, pose, apply_offset=True):
        offset = [
            self.args.tool_tip_offset_x,
            self.args.tool_tip_offset_y,
            self.args.tool_tip_offset_z,
        ]
        if not apply_offset or max(abs(v) for v in offset) < 1e-9:
            return (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        rotation = quaternion_matrix(q)
        rotated_offset = [
            sum(rotation[row][col] * offset[col] for col in range(3))
            for row in range(3)
        ]
        return (
            pose.position.x + rotated_offset[0],
            pose.position.y + rotated_offset[1],
            pose.position.z + rotated_offset[2],
        )

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

    def make_constraints(self, pose_stamped):
        constraints = Constraints()
        constraints.name = "perception_grasp_pose"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.args.position_tolerance]

        region_pose = Pose()
        region_pose.position = pose_stamped.pose.position
        region_pose.orientation.w = 1.0

        position = PositionConstraint()
        position.header = pose_stamped.header
        position.link_name = self.args.end_effector_link
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(region_pose)
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header = pose_stamped.header
        orientation.link_name = self.args.end_effector_link
        orientation.orientation = pose_stamped.pose.orientation
        orientation.absolute_x_axis_tolerance = self.args.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.args.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.args.orientation_tolerance
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def send_moveit_goal(self, name, pose_stamped):
        self.validate_workspace_pose(name, pose_stamped)
        if not self.move_group.wait_for_server(timeout_sec=self.args.server_timeout):
            raise RuntimeError(f"MoveIt action server not available: {self.args.move_group_action}")

        goal = MoveGroup.Goal()
        goal.request.group_name = self.args.group_name
        goal.request.num_planning_attempts = self.args.planning_attempts
        goal.request.allowed_planning_time = self.args.allowed_planning_time
        goal.request.max_velocity_scaling_factor = self.args.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.args.acceleration_scaling
        goal.request.goal_constraints = [self.make_constraints(pose_stamped)]
        moveit_should_execute = (
            not self.args.plan_only and self.args.trajectory_backend == "moveit_controller"
        )
        goal.planning_options.plan_only = not moveit_should_execute
        goal.planning_options.replan = self.args.replan
        goal.planning_options.replan_attempts = self.args.replan_attempts

        self.get_logger().info(
            f"MoveIt {name}: target=({pose_stamped.pose.position.x:.3f}, "
            f"{pose_stamped.pose.position.y:.3f}, {pose_stamped.pose.position.z:.3f}), "
            f"plan_only={self.args.plan_only}"
        )
        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"MoveIt rejected {name} goal")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"MoveIt returned no result for {name}")
        code = wrapped.result.error_code.val
        if code != 1:
            raise RuntimeError(
                f"MoveIt failed {name}; error_code={code}; target="
                f"({pose_stamped.pose.position.x:.3f}, "
                f"{pose_stamped.pose.position.y:.3f}, "
                f"{pose_stamped.pose.position.z:.3f}). "
                "Try lowering --pregrasp-height, increasing --orientation-tolerance, "
                "or moving the object farther from the base if this target is near the arm's singular region."
            )
        if not self.args.plan_only and self.args.trajectory_backend == "topic":
            self.execute_trajectory_topic(wrapped.result.planned_trajectory.joint_trajectory, name)
        self.log_feedback_error(name, pose_stamped.pose)
        self.get_logger().info(f"MoveIt {name} complete")

    def log_feedback_error(self, name, target_ee_pose):
        if self.args.plan_only:
            return None
        deadline = time.time() + self.args.feedback_wait
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_end_pose is None:
            self.get_logger().warn(f"{name}: no /end_pose feedback available for final error check")
            return None
        target_tip = self.tip_from_ee_pose(target_ee_pose)
        actual_tip = self.tip_from_ee_pose(
            self.latest_end_pose,
            apply_offset=not self.args.end_pose_is_tip,
        )
        error = math.sqrt(
            (target_tip[0] - actual_tip[0]) ** 2
            + (target_tip[1] - actual_tip[1]) ** 2
            + (target_tip[2] - actual_tip[2]) ** 2
        )
        self.get_logger().info(
            f"{name} final feedback: target_tip=({target_tip[0]:.3f}, "
            f"{target_tip[1]:.3f}, {target_tip[2]:.3f}), actual_tip="
            f"({actual_tip[0]:.3f}, {actual_tip[1]:.3f}, {actual_tip[2]:.3f}), "
            f"tip_error={error:.3f} m"
        )
        return target_tip, actual_tip, error

    def apply_final_cartesian_correction(self, target_ee_pose):
        if self.args.plan_only or not self.args.final_cartesian_correction:
            return
        if self.latest_end_pose is None:
            self.get_logger().warn("Skipping final correction: no /end_pose feedback")
            return

        target_tip = self.tip_from_ee_pose(target_ee_pose)
        start = time.time()
        while rclpy.ok() and time.time() - start < self.args.final_correction_timeout:
            actual_tip = self.tip_from_ee_pose(
                self.latest_end_pose,
                apply_offset=not self.args.end_pose_is_tip,
            )
            error_vector = [
                target_tip[0] - actual_tip[0],
                target_tip[1] - actual_tip[1],
                target_tip[2] - actual_tip[2],
            ]
            error = math.sqrt(sum(value * value for value in error_vector))
            if error <= self.args.final_correction_tolerance:
                self.get_logger().info(
                    f"Final Cartesian correction complete: target_tip=({target_tip[0]:.3f}, "
                    f"{target_tip[1]:.3f}, {target_tip[2]:.3f}), actual_tip="
                    f"({actual_tip[0]:.3f}, {actual_tip[1]:.3f}, {actual_tip[2]:.3f}), "
                    f"tip_error={error:.3f} m"
                )
                return

            step = min(error, self.args.final_correction_step)
            direction = [value / max(error, 1e-9) for value in error_vector]
            q = [
                self.latest_end_pose.orientation.x,
                self.latest_end_pose.orientation.y,
                self.latest_end_pose.orientation.z,
                self.latest_end_pose.orientation.w,
            ]
            roll, pitch, yaw = euler_from_quaternion(q)

            msg = PosCmd()
            msg.x = self.latest_end_pose.position.x + direction[0] * step
            msg.y = self.latest_end_pose.position.y + direction[1] * step
            msg.z = self.latest_end_pose.position.z + direction[2] * step
            msg.roll = roll
            msg.pitch = pitch
            msg.yaw = yaw
            msg.gripper = self.args.open_gripper
            self.pos_cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(self.args.final_correction_period)

        feedback = self.log_feedback_error("final_correction", target_ee_pose)
        if feedback is not None and feedback[2] > self.args.final_correction_tolerance:
            self.get_logger().warn(
                f"Final Cartesian correction timed out with tip_error={feedback[2]:.3f} m"
            )

    def execute_trajectory_topic(self, trajectory, name):
        if not trajectory.points:
            raise RuntimeError(f"MoveIt planned no trajectory points for {name}")
        self.get_logger().info(
            f"Executing {name} through {self.args.trajectory_topic}: {len(trajectory.points)} points"
        )
        previous_time = 0.0
        max_points = max(1, self.args.max_trajectory_points)
        stride = max(1, math.ceil(len(trajectory.points) / max_points))
        selected_points = trajectory.points[::stride]
        if selected_points[-1] is not trajectory.points[-1]:
            selected_points.append(trajectory.points[-1])

        for point in selected_points:
            previous_time = self.publish_timed_trajectory_point(trajectory, point, previous_time)

        hold_end = time.time() + self.args.final_hold_time
        final_point = selected_points[-1]
        while rclpy.ok() and time.time() < hold_end:
            self.publish_single_trajectory_point(trajectory, final_point)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(min(0.10, self.args.point_duration))

    def publish_timed_trajectory_point(self, trajectory, point, previous_time):
        target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        sleep_time = max(0.0, target_time - previous_time)
        if sleep_time > 0.0:
            time.sleep(min(sleep_time, self.args.max_point_sleep))
        self.publish_single_trajectory_point(trajectory, point)
        rclpy.spin_once(self, timeout_sec=0.01)
        return target_time

    def publish_single_trajectory_point(self, trajectory, point):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(trajectory.joint_names)
        out_point = JointTrajectoryPoint()
        out_point.positions = list(point.positions)
        out_point.velocities = list(point.velocities)
        out_point.accelerations = list(point.accelerations)
        out_point.time_from_start.sec = 0
        out_point.time_from_start.nanosec = int(self.args.point_duration * 1e9)
        msg.points.append(out_point)
        self.trajectory_pub.publish(msg)

    def send_gripper_action(self, position, name):
        if self.args.plan_only or self.args.gripper_backend != "action":
            return
        if not self.gripper_action.wait_for_server(timeout_sec=self.args.server_timeout):
            raise RuntimeError(f"Gripper action server not available: {self.args.gripper_action}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.args.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start.sec = int(self.args.gripper_time)
        point.time_from_start.nanosec = int((self.args.gripper_time % 1.0) * 1e9)
        goal.trajectory.points.append(point)

        self.get_logger().info(f"Sending gripper {name}: {position:.3f} m")
        future = self.gripper_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Gripper action rejected {name}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def publish_gripper_pos_cmd(self, position, name):
        if self.args.plan_only or self.args.gripper_backend != "pos_cmd":
            return
        if self.latest_end_pose is None:
            self.get_logger().warn(f"Cannot send gripper {name} via /pos_cmd: no /end_pose feedback")
            return
        msg = PosCmd()
        msg.x = self.latest_end_pose.position.x
        msg.y = self.latest_end_pose.position.y
        msg.z = self.latest_end_pose.position.z
        roll, pitch, yaw = euler_from_quaternion(
            [
                self.latest_end_pose.orientation.x,
                self.latest_end_pose.orientation.y,
                self.latest_end_pose.orientation.z,
                self.latest_end_pose.orientation.w,
            ]
        )
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        msg.gripper = float(position)
        end = time.time() + self.args.gripper_time
        while rclpy.ok() and time.time() < end:
            self.pos_cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)
        self.get_logger().info(f"Sent gripper {name} via /pos_cmd: {position:.3f} m")

    def command_gripper(self, position, name):
        self.send_gripper_action(position, name)
        self.publish_gripper_pos_cmd(position, name)

    def build_sequence(self, object_pose):
        q = quaternion_from_euler(self.args.roll, self.args.pitch, self.args.yaw)
        x = object_pose.pose.position.x + self.args.grasp_dx
        y = object_pose.pose.position.y + self.args.grasp_dy
        z = object_pose.pose.position.z + self.args.grasp_dz
        pre_z = z + self.args.pregrasp_height
        lift_z = z + self.args.lift_height
        if max(abs(v) for v in [self.args.tool_tip_offset_x, self.args.tool_tip_offset_y, self.args.tool_tip_offset_z]) > 1e-9:
            self.get_logger().info(
                "Using tool-tip offset in end-effector frame: "
                f"({self.args.tool_tip_offset_x:.3f}, {self.args.tool_tip_offset_y:.3f}, "
                f"{self.args.tool_tip_offset_z:.3f}) m"
            )
        return [
            ("pregrasp", self.make_pose(object_pose, x, y, pre_z, q)),
            ("grasp", self.make_pose(object_pose, x, y, z, q)),
            ("lift", self.make_pose(object_pose, x, y, lift_z, q)),
        ]

    def run(self):
        object_pose = self.wait_for_stable_pose()
        sequence = self.build_sequence(object_pose)
        if self.args.pregrasp_only:
            active_sequence = sequence[:1]
        elif self.args.skip_lift:
            active_sequence = sequence[:2]
        else:
            active_sequence = sequence
        for name, pose in active_sequence:
            self.validate_workspace_pose(name, pose)
        self.enable_arm()

        self.command_gripper(self.args.open_gripper, "open")
        self.send_moveit_goal("pregrasp", sequence[0][1])
        if self.args.pregrasp_only:
            self.get_logger().info("MoveIt pregrasp-only sequence complete")
            return
        self.send_moveit_goal("grasp", sequence[1][1])
        self.apply_final_cartesian_correction(sequence[1][1].pose)
        self.command_gripper(self.args.closed_gripper, "close")
        if not self.args.skip_lift:
            self.send_moveit_goal("lift", sequence[2][1])
        self.get_logger().info("MoveIt grasp sequence complete")


def parse_args():
    parser = argparse.ArgumentParser(
        description="MoveIt grasp executor: object pose -> pregrasp -> grasp -> close -> lift."
    )
    parser.add_argument("--object-pose-topic", default="/perception/object_pose_base")
    parser.add_argument("--end-pose-topic", default="/end_pose")
    parser.add_argument("--pos-cmd-topic", default="/pos_cmd")
    parser.add_argument("--enable-service", default="/enable_srv")
    parser.add_argument("--move-group-action", default="/move_action")
    parser.add_argument("--gripper-action", default="/gripper_controller/follow_joint_trajectory")
    parser.add_argument("--trajectory-topic", default="/arm_controller/joint_trajectory")
    parser.add_argument("--trajectory-backend", choices=["topic", "moveit_controller"], default="topic")
    parser.add_argument("--group-name", default="arm")
    parser.add_argument("--end-effector-link", default="link6")
    parser.add_argument("--gripper-joint", default="joint7")
    parser.add_argument("--gripper-backend", choices=["none", "action", "pos_cmd"], default="pos_cmd")
    parser.add_argument("--enable", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--avoid-collisions", action="store_true")
    parser.add_argument("--replan", action="store_true")
    parser.add_argument("--replan-attempts", type=int, default=2)
    parser.add_argument("--pose-timeout", type=float, default=15.0)
    parser.add_argument("--stable-samples", type=int, default=5)
    parser.add_argument("--stable-position-tolerance", type=float, default=0.030)
    parser.add_argument("--stability-log-period", type=float, default=1.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.15)
    parser.add_argument("--acceleration-scaling", type=float, default=0.15)
    parser.add_argument("--position-tolerance", type=float, default=0.015)
    parser.add_argument("--orientation-tolerance", type=float, default=math.radians(20.0))
    parser.add_argument("--pregrasp-height", type=float, default=0.10)
    parser.add_argument("--lift-height", type=float, default=0.12)
    parser.add_argument("--grasp-dx", type=float, default=0.0)
    parser.add_argument("--grasp-dy", type=float, default=0.0)
    parser.add_argument("--grasp-dz", type=float, default=0.02)
    parser.add_argument("--tool-tip-offset-x", type=float, default=0.0)
    parser.add_argument("--tool-tip-offset-y", type=float, default=0.0)
    parser.add_argument("--tool-tip-offset-z", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=math.radians(-179.9))
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=math.radians(-179.9))
    parser.add_argument("--open-gripper", type=float, default=0.08)
    parser.add_argument("--closed-gripper", type=float, default=0.0)
    parser.add_argument("--gripper-time", type=float, default=1.0)
    parser.add_argument("--max-trajectory-points", type=int, default=25)
    parser.add_argument("--max-point-sleep", type=float, default=0.25)
    parser.add_argument("--point-duration", type=float, default=0.20)
    parser.add_argument("--final-hold-time", type=float, default=1.0)
    parser.add_argument("--feedback-wait", type=float, default=0.5)
    parser.add_argument("--final-cartesian-correction", action="store_true")
    parser.add_argument("--end-pose-is-tip", action="store_true", default=True)
    parser.add_argument("--end-pose-is-link", dest="end_pose_is_tip", action="store_false")
    parser.add_argument("--final-correction-timeout", type=float, default=4.0)
    parser.add_argument("--final-correction-step", type=float, default=0.015)
    parser.add_argument("--final-correction-period", type=float, default=0.15)
    parser.add_argument("--final-correction-tolerance", type=float, default=0.015)
    parser.add_argument("--skip-lift", action="store_true")
    parser.add_argument("--pregrasp-only", action="store_true")
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
    node = MoveItGraspExecutor(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
