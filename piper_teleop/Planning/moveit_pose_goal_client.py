#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class MoveItPoseGoalClient(Node):
    def __init__(self, args):
        super().__init__("moveit_pose_goal_client")
        self.args = args
        self.latest_pose = None
        self.action_client = ActionClient(self, MoveGroup, args.move_group_action)
        self.create_subscription(PoseStamped, args.object_pose_topic, self.pose_callback, 10)
        self.get_logger().info(
            f"Waiting for object pose on {args.object_pose_topic}; MoveIt action={args.move_group_action}"
        )

    def pose_callback(self, msg):
        self.latest_pose = msg

    def wait_for_pose(self):
        start = time.time()
        while rclpy.ok() and self.latest_pose is None:
            if time.time() - start > self.args.pose_timeout:
                raise TimeoutError(f"No object pose received on {self.args.object_pose_topic}")
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_pose

    def make_pose(self, source, z_offset):
        pose = PoseStamped()
        pose.header = source.header
        pose.pose.position.x = source.pose.position.x
        pose.pose.position.y = source.pose.position.y
        pose.pose.position.z = source.pose.position.z + z_offset
        pose.pose.orientation = source.pose.orientation
        return pose

    def send_pose_goal(self, pose_stamped, name):
        if not self.action_client.wait_for_server(timeout_sec=self.args.server_timeout):
            raise RuntimeError(f"MoveIt action server not available: {self.args.move_group_action}")

        goal = MoveGroup.Goal()
        goal.request.group_name = self.args.group_name
        goal.request.num_planning_attempts = self.args.planning_attempts
        goal.request.allowed_planning_time = self.args.allowed_planning_time
        goal.request.max_velocity_scaling_factor = self.args.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.args.acceleration_scaling
        goal.request.goal_constraints = [self.make_constraints(pose_stamped)]
        goal.planning_options.plan_only = self.args.plan_only
        goal.planning_options.replan = self.args.replan
        goal.planning_options.replan_attempts = self.args.replan_attempts

        self.get_logger().info(f"Sending MoveIt goal: {name}")
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"MoveIt rejected goal: {name}")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"MoveIt returned no result for goal: {name}")

        code = wrapped.result.error_code.val
        if code != 1:
            raise RuntimeError(f"MoveIt failed goal {name}; error_code={code}")
        self.get_logger().info(f"MoveIt goal complete: {name}")

    def make_constraints(self, pose_stamped):
        constraints = Constraints()
        constraints.name = "piper_pose_goal"

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

    def run(self):
        object_pose = self.wait_for_pose()
        pregrasp = self.make_pose(object_pose, self.args.pregrasp_height)
        grasp = self.make_pose(object_pose, self.args.grasp_z_offset)
        lift = self.make_pose(object_pose, self.args.lift_height)

        self.send_pose_goal(pregrasp, "pregrasp")
        self.send_pose_goal(grasp, "grasp")
        if not self.args.skip_lift:
            self.send_pose_goal(lift, "lift")


def parse_args():
    parser = argparse.ArgumentParser(
        description="MoveIt action client for object-pose grasp approach/lift planning."
    )
    parser.add_argument("--object-pose-topic", default="/perception/object_pose_base")
    parser.add_argument("--move-group-action", default="/move_action")
    parser.add_argument("--group-name", default="piper_arm")
    parser.add_argument("--end-effector-link", default="gripper_base")
    parser.add_argument("--pose-timeout", type=float, default=10.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.2)
    parser.add_argument("--acceleration-scaling", type=float, default=0.2)
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance", type=float, default=math.radians(10.0))
    parser.add_argument("--pregrasp-height", type=float, default=0.10)
    parser.add_argument("--grasp-z-offset", type=float, default=0.02)
    parser.add_argument("--lift-height", type=float, default=0.12)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--replan", action="store_true")
    parser.add_argument("--replan-attempts", type=int, default=2)
    parser.add_argument("--skip-lift", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MoveItPoseGoalClient(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
