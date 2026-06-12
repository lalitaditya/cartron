#!/usr/bin/env python3

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUBLISHER = ROOT / "Perception" / "depthai_apriltag_pose_publisher.py"
TRANSFORMER = ROOT / "Perception" / "camera_to_base_pose_transformer.py"
MOVEIT_EXECUTOR = ROOT / "Planning" / "moveit_grasp_executor.py"


def stop_children(children):
    for child in reversed(children):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
    deadline = time.time() + 4.0
    for child in reversed(children):
        if child.poll() is not None:
            continue
        try:
            child.wait(timeout=max(0.1, deadline - time.time()))
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            child.terminate()
    deadline = time.time() + 2.0
    for child in reversed(children):
        if child.poll() is not None:
            continue
        try:
            child.wait(timeout=max(0.1, deadline - time.time()))
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            child.kill()


def main():
    args = parse_args()
    publisher_cmd = [
        sys.executable,
        str(PUBLISHER),
        "--dictionary",
        args.dictionary,
        "--tag-id",
        str(args.tag_id),
        "--frame-color-mode",
        args.frame_color_mode,
        "--depth-source",
        args.depth_source,
        "--tag-size-m",
        str(args.tag_size_m),
        "--fps",
        str(args.fps),
        "--publish-rate",
        str(args.publish_rate),
        "--min-depth-m",
        str(args.min_depth_m),
        "--max-depth-m",
        str(args.max_depth_m),
        "--fx",
        str(args.fx),
        "--fy",
        str(args.fy),
        "--cx",
        str(args.cx),
        "--cy",
        str(args.cy),
    ]
    if args.depth_source == "stereo":
        publisher_cmd.extend(["--stereo-preset", args.stereo_preset])

    transformer_cmd = [
        sys.executable,
        str(TRANSFORMER),
        "--x",
        str(args.camera_x),
        "--y",
        str(args.camera_y),
        "--z",
        str(args.camera_z),
        "--roll",
        str(args.camera_roll),
        "--pitch",
        str(args.camera_pitch),
        "--yaw",
        str(args.camera_yaw),
    ]

    executor_cmd = [
        sys.executable,
        str(MOVEIT_EXECUTOR),
        "--pose-timeout",
        str(args.pose_timeout),
        "--move-group-action",
        args.move_group_action,
        "--group-name",
        args.moveit_group_name,
        "--end-effector-link",
        args.moveit_end_effector_link,
        "--gripper-backend",
        args.gripper_backend,
        "--trajectory-backend",
        args.trajectory_backend,
        "--trajectory-topic",
        args.trajectory_topic,
        "--velocity-scaling",
        str(args.velocity_scaling),
        "--acceleration-scaling",
        str(args.acceleration_scaling),
        "--allowed-planning-time",
        str(args.allowed_planning_time),
        "--service-timeout",
        str(args.service_timeout),
        "--position-tolerance",
        str(args.position_tolerance),
        "--orientation-tolerance",
        str(args.orientation_tolerance),
        "--pregrasp-height",
        str(args.standoff_m),
        "--grasp-dz",
        "0.0",
        "--pregrasp-only",
        "--skip-lift",
        "--tool-tip-offset-x",
        str(args.tool_tip_offset_x),
        "--tool-tip-offset-y",
        str(args.tool_tip_offset_y),
        "--tool-tip-offset-z",
        str(args.tool_tip_offset_z),
        "--max-trajectory-points",
        str(args.max_trajectory_points),
        "--max-point-sleep",
        str(args.max_point_sleep),
        "--point-duration",
        str(args.point_duration),
        "--stable-samples",
        str(args.stable_samples),
        "--stable-position-tolerance",
        str(args.stable_position_tolerance),
        "--min-radius",
        str(args.min_radius),
        "--max-radius",
        str(args.max_radius),
        "--min-z",
        str(args.min_z),
        "--max-z",
        str(args.max_z),
    ]
    if args.enable:
        executor_cmd.append("--enable")
    if args.plan_only:
        executor_cmd.append("--plan-only")

    children = []
    try:
        children.append(subprocess.Popen(publisher_cmd))
        time.sleep(args.startup_delay)
        children.append(subprocess.Popen(transformer_cmd))
        print(
            "\nAPRILTAG STANDOFF MODE: the gripper tip will move to "
            f"{args.standoff_m:.3f} m above the detected AprilTag center and stop. "
            "Confirm camera calibration and workspace clearance before continuing.\n"
        )
        time.sleep(args.startup_delay)
        return subprocess.call(executor_cmd)
    finally:
        stop_children(children)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run AprilTag perception and move the gripper tip to a standoff above the tag center."
    )
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--tag-id", type=int, default=-1)
    parser.add_argument("--frame-color-mode", choices=["bgr", "rgb"], default="bgr")
    parser.add_argument("--depth-source", choices=["tag_size", "stereo"], default="tag_size")
    parser.add_argument("--tag-size-m", type=float, default=0.08)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--publish-rate", type=float, default=10.0)
    parser.add_argument("--min-depth-m", type=float, default=0.12)
    parser.add_argument("--max-depth-m", type=float, default=2.0)
    parser.add_argument("--fx", type=float, default=-1.0)
    parser.add_argument("--fy", type=float, default=-1.0)
    parser.add_argument("--cx", type=float, default=-1.0)
    parser.add_argument("--cy", type=float, default=-1.0)
    parser.add_argument(
        "--stereo-preset",
        choices=["DEFAULT", "DENSITY", "FAST_DENSITY", "ROBOTICS", "HIGH_DETAIL"],
        default="FAST_DENSITY",
    )
    parser.add_argument("--startup-delay", type=float, default=2.0)
    parser.add_argument("--pose-timeout", type=float, default=30.0)
    parser.add_argument("--enable", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--move-group-action", default="/move_action")
    parser.add_argument("--moveit-group-name", default="arm")
    parser.add_argument("--moveit-end-effector-link", default="link6")
    parser.add_argument("--gripper-backend", choices=["none", "action", "pos_cmd"], default="none")
    parser.add_argument("--trajectory-backend", choices=["topic", "moveit_controller"], default="topic")
    parser.add_argument("--trajectory-topic", default="/arm_controller/joint_trajectory")
    parser.add_argument("--velocity-scaling", type=float, default=0.03)
    parser.add_argument("--acceleration-scaling", type=float, default=0.03)
    parser.add_argument("--allowed-planning-time", type=float, default=8.0)
    parser.add_argument("--service-timeout", type=float, default=25.0)
    parser.add_argument("--position-tolerance", type=float, default=0.008)
    parser.add_argument("--orientation-tolerance", type=float, default=0.349066)
    parser.add_argument("--standoff-m", type=float, default=0.10)
    parser.add_argument("--tool-tip-offset-x", type=float, default=0.0)
    parser.add_argument("--tool-tip-offset-y", type=float, default=0.0)
    parser.add_argument("--tool-tip-offset-z", type=float, default=0.0)
    parser.add_argument("--max-trajectory-points", type=int, default=120)
    parser.add_argument("--max-point-sleep", type=float, default=0.05)
    parser.add_argument("--point-duration", type=float, default=0.25)
    parser.add_argument("--stable-samples", type=int, default=5)
    parser.add_argument("--stable-position-tolerance", type=float, default=0.015)
    parser.add_argument("--camera-x", type=float, default=0.0)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.13)
    parser.add_argument("--camera-roll", type=float, default=-1.57079632679)
    parser.add_argument("--camera-pitch", type=float, default=0.0)
    parser.add_argument("--camera-yaw", type=float, default=-1.57079632679)
    parser.add_argument("--min-radius", type=float, default=0.12)
    parser.add_argument("--max-radius", type=float, default=0.62)
    parser.add_argument("--min-z", type=float, default=0.02)
    parser.add_argument("--max-z", type=float, default=0.55)
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(main())
