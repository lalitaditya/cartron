#!/usr/bin/env python3
"""
Smooth joint trajectory publisher for the Piper arm.

Generates a cubic spline trajectory through a list of waypoints, with:
  - Joint limit checking (from URDF) before execution
  - Time parametrization based on joint velocity limits
  - Correct speed percentage sent to the driver
  - Smooth interpolation (no jerking between waypoints)

Usage: edit WAYPOINTS at the bottom, then run:
  ros2 run piper smooth_joint_trajectory
"""

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState

# Joint limits from piper_description.urdf (radians)
JOINT_LIMITS = [
    (-2.618,  2.618),   # joint1
    ( 0.0,    3.14),    # joint2
    (-2.967,  0.0),     # joint3
    (-1.745,  1.745),   # joint4
    (-1.22,   1.22),    # joint5
    (-2.0944, 2.0944),  # joint6
]

# Max joint velocities from URDF (rad/s) — used for time parametrization
MAX_VEL_RAD_S = [5.0, 5.0, 5.0, 5.0, 5.0, 3.0]


def check_joint_limits(positions: list[float], waypoint_index: int) -> None:
    for i, (pos, (lo, hi)) in enumerate(zip(positions, JOINT_LIMITS)):
        if not (lo <= pos <= hi):
            raise ValueError(
                f"Waypoint {waypoint_index}, joint{i + 1}={pos:.4f} rad "
                f"is outside limits [{lo:.4f}, {hi:.4f}]"
            )


def segment_duration(q0: list[float], q1: list[float], speed_fraction: float) -> float:
    """
    Minimum time to move between two joint positions without exceeding velocity
    limits, scaled by speed_fraction (0.0–1.0).
    """
    effective_vel = [v * speed_fraction for v in MAX_VEL_RAD_S]
    times = [
        abs(a - b) / ev
        for a, b, ev in zip(q0, q1, effective_vel)
        if abs(a - b) > 1e-6
    ]
    return max(times) if times else 0.3


class SmoothTrajectoryPublisher(Node):
    def __init__(self, waypoints: list[list[float]], speed_pct: int = 10, rate_hz: float = 20.0):
        super().__init__("smooth_trajectory_publisher")
        self.publisher = self.create_publisher(JointState, "/joint_ctrl_cmd", 10)

        # Validate all waypoints before doing anything
        for i, wp in enumerate(waypoints):
            if len(wp) != 6:
                raise ValueError(f"Waypoint {i} must have 6 joint values, got {len(wp)}")
            check_joint_limits(wp, i)

        speed_fraction = max(0.01, min(speed_pct / 100.0, 1.0))

        # Build cumulative time stamps for each waypoint
        timestamps = [0.0]
        for i in range(1, len(waypoints)):
            dt = segment_duration(waypoints[i - 1], waypoints[i], speed_fraction)
            timestamps.append(timestamps[-1] + dt)

        positions = np.array(waypoints)  # shape (N, 6)

        # One cubic spline per joint, clamped = zero velocity at start and end
        self.splines = [
            CubicSpline(timestamps, positions[:, j], bc_type="clamped")
            for j in range(6)
        ]
        self.t_end = timestamps[-1]
        self.speed_pct = int(max(1, min(speed_pct, 100)))

        self.t_start = None
        self.done = False
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_step)

        self.get_logger().info(
            f"Trajectory: {len(waypoints)} waypoints, "
            f"duration={self.t_end:.2f}s, speed={self.speed_pct}%"
        )

    def _publish_step(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t_start is None:
            self.t_start = now

        t = now - self.t_start
        if t > self.t_end:
            self.done = True
            self.timer.cancel()
            self.get_logger().info("Trajectory complete.")
            return

        positions = [float(s(t)) for s in self.splines]
        velocities = [float(s(t, 1)) for s in self.splines]  # first derivative = rad/s

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        msg.position = positions
        # The driver reads velocity[6] (7th element) as the speed percentage.
        # Sending exactly 7 values activates that path in piper_ctrl_single_node.py:317-326.
        msg.velocity = velocities + [float(self.speed_pct)]
        self.publisher.publish(msg)


def main() -> None:
    # ---------------------------------------------------------------
    # Edit WAYPOINTS to define the motion. All values are in radians.
    # The arm will move smoothly through each point in order.
    # ---------------------------------------------------------------
    WAYPOINTS = [
        [0.0,  0.5, -0.5,  0.0,  0.0,  0.0],   # start
        [0.5,  1.0, -1.0,  0.3,  0.3,  0.5],   # mid
        [0.0,  0.5, -0.5,  0.0,  0.0,  0.0],   # back to start
    ]

    # Speed as a percentage of max joint velocity (1–100).
    # 10 = smooth and safe; increase once you trust the motion.
    SPEED_PCT = 10

    rclpy.init()
    try:
        node = SmoothTrajectoryPublisher(WAYPOINTS, speed_pct=SPEED_PCT)
    except ValueError as e:
        print(f"[ERROR] {e}")
        rclpy.shutdown()
        return

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
