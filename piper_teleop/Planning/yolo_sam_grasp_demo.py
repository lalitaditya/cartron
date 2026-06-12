#!/usr/bin/env python3

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PERCEPTION = ROOT / "Perception" / "depthai_yolo_sam_pose_publisher.py"
TRANSFORMER = ROOT / "Perception" / "camera_to_base_pose_transformer.py"
EXECUTOR = ROOT / "Planning" / "stage0_grasp_executor.py"
MOVEIT_EXECUTOR = ROOT / "Planning" / "moveit_grasp_executor.py"
DEFAULT_YOLO_MODEL = ROOT / "models" / "yolo11n.pt"


def append_if_value(cmd, flag, value):
    if value is not None and value != "":
        cmd.extend([flag, str(value)])


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
    for child in reversed(children):
        try:
            if child.poll() is None:
                child.wait(timeout=0.5)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            pass


def main():
    args = parse_args()
    yolo_model = args.yolo_model
    if args.detector == "ultralytics" and not yolo_model:
        yolo_model = str(DEFAULT_YOLO_MODEL)
    if args.detector == "ultralytics" and not Path(yolo_model).exists():
        raise FileNotFoundError(
            f"YOLO model not found: {yolo_model}. "
            "Download yolo11n.pt into piper_teleop/models/ or pass --yolo-model /path/to/model.pt."
        )
    perception_cmd = [
        sys.executable,
        str(PERCEPTION),
        "--detector",
        args.detector,
        "--depth-source",
        args.depth_source,
        "--fixed-depth-m",
        str(args.fixed_depth_m),
        "--confidence",
        str(args.confidence),
        "--frame-color-mode",
        args.frame_color_mode,
        "--target-class",
        args.target_class,
        "--publish-rate",
        str(args.publish_rate),
        "--fps",
        str(args.fps),
        "--depth-window",
        str(args.depth_window),
        "--min-depth-pixels",
        str(args.min_depth_pixels),
        "--min-depth-m",
        str(args.min_depth_m),
        "--max-depth-m",
        str(args.max_depth_m),
        "--stereo-preset",
        args.stereo_preset,
        "--camera-roll",
        str(args.camera_roll),
        "--camera-pitch",
        str(args.camera_pitch),
        "--camera-yaw",
        str(args.camera_yaw),
        "--axis-length-m",
        str(args.axis_length_m),
        "--axis-thickness",
        str(args.axis_thickness),
    ]
    append_if_value(perception_cmd, "--yolo-model", yolo_model)
    if args.allow_fixed_depth_fallback:
        perception_cmd.append("--allow-fixed-depth-fallback")
    if args.subpixel:
        perception_cmd.append("--subpixel")
    if not args.left_right_check:
        perception_cmd.append("--no-left-right-check")

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

    executor_script = MOVEIT_EXECUTOR if args.planner_backend == "moveit" else EXECUTOR
    executor_cmd = [
        sys.executable,
        str(executor_script),
        "--pose-timeout",
        str(args.pose_timeout),
        "--min-radius",
        str(args.min_radius),
        "--max-radius",
        str(args.max_radius),
        "--min-z",
        str(args.min_z),
        "--max-z",
        str(args.max_z),
    ]
    if args.planner_backend == "pos_cmd":
        executor_cmd.extend(
            [
                "--command-rate",
                str(args.command_rate),
                "--max-initial-translation",
                str(args.max_initial_translation),
            ]
        )
    else:
        executor_cmd.extend(
            [
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
                str(args.pregrasp_height),
                "--grasp-dx",
                str(args.grasp_dx),
                "--grasp-dy",
                str(args.grasp_dy),
                "--grasp-dz",
                str(args.grasp_dz),
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
                "--final-hold-time",
                str(args.final_hold_time),
                "--feedback-wait",
                str(args.feedback_wait),
                "--final-correction-timeout",
                str(args.final_correction_timeout),
                "--final-correction-step",
                str(args.final_correction_step),
                "--final-correction-period",
                str(args.final_correction_period),
                "--final-correction-tolerance",
                str(args.final_correction_tolerance),
                "--stable-samples",
                str(args.stable_samples),
                "--stable-position-tolerance",
                str(args.stable_position_tolerance),
            ]
        )
    if args.enable:
        executor_cmd.append("--enable")
    if args.plan_only and args.planner_backend == "moveit":
        executor_cmd.append("--plan-only")
    if args.skip_lift and args.planner_backend == "moveit":
        executor_cmd.append("--skip-lift")
    if args.pregrasp_only and args.planner_backend == "moveit":
        executor_cmd.append("--pregrasp-only")
    if args.final_cartesian_correction and args.planner_backend == "moveit":
        executor_cmd.append("--final-cartesian-correction")
    if not args.end_pose_is_tip and args.planner_backend == "moveit":
        executor_cmd.append("--end-pose-is-link")
    if args.check_ik and args.planner_backend == "pos_cmd":
        executor_cmd.append("--check-ik")
    if args.avoid_collisions:
        executor_cmd.append("--avoid-collisions")
    if args.allow_large_motion and args.planner_backend == "pos_cmd":
        executor_cmd.append("--allow-large-motion")
    if args.use_command_orientation and args.planner_backend == "pos_cmd":
        executor_cmd.append("--use-command-orientation")

    children = []
    try:
        children.append(subprocess.Popen(perception_cmd))
        time.sleep(args.startup_delay)
        children.append(subprocess.Popen(transformer_cmd))
        if not args.execute:
            print(
                "\nPerception-only mode is running. The camera window should show the live stream "
                "and detections; no arm motion will be commanded. Press q/Esc in the camera window "
                "or Ctrl-C here to stop.\n"
            )
            try:
                return children[0].wait()
            except KeyboardInterrupt:
                print("\nStopping perception-only run.")
                return 130
        print(
            f"\nEXECUTION MODE: planner_backend={args.planner_backend}; arm motion commands may be sent. Confirm the camera is rigidly "
            "mounted, the USB cable has slack/strain relief, and the first target is near the current pose.\n"
        )
        if not args.enable:
            print(
                "Note: --execute was provided without --enable. The executor will publish commands, "
                "but the Piper driver may ignore motion until the arm is enabled.\n"
            )
        time.sleep(args.startup_delay)
        try:
            result = subprocess.call(executor_cmd)
        except KeyboardInterrupt:
            print("\nStopping grasp demo.")
            return 130
        if result != 0:
            print(
                "\nGrasp executor failed. If the error says /enable_srv is unavailable, "
                "start the Piper hardware driver first. If the initial move guard rejected the target, "
                "move the arm closer to the target or increase --max-initial-translation only after checking clearance.",
                file=sys.stderr,
            )
        return result
    finally:
        stop_children(children)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run live DepthAI YOLO RGBD perception, with explicit opt-in for arm motion."
    )
    parser.add_argument("--detector", choices=["ultralytics", "opencv_onnx", "hsv", "center"], default="ultralytics")
    parser.add_argument("--yolo-model", default="")
    parser.add_argument("--target-class", default="")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--frame-color-mode", choices=["bgr", "rgb"], default="bgr")
    parser.add_argument("--depth-source", choices=["stereo", "fixed"], default="stereo")
    parser.add_argument("--fixed-depth-m", type=float, default=0.25)
    parser.add_argument("--allow-fixed-depth-fallback", action="store_true")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--depth-window", type=int, default=12)
    parser.add_argument("--min-depth-pixels", type=int, default=10)
    parser.add_argument("--min-depth-m", type=float, default=0.12)
    parser.add_argument("--max-depth-m", type=float, default=2.00)
    parser.add_argument(
        "--stereo-preset",
        choices=["DEFAULT", "DENSITY", "FAST_DENSITY", "ROBOTICS", "HIGH_DETAIL"],
        default="FAST_DENSITY",
    )
    parser.add_argument("--subpixel", action="store_true")
    parser.add_argument("--left-right-check", action="store_true", default=True)
    parser.add_argument("--no-left-right-check", dest="left_right_check", action="store_false")
    parser.add_argument("--publish-rate", type=float, default=10.0)
    parser.add_argument("--startup-delay", type=float, default=2.0)
    parser.add_argument("--pose-timeout", type=float, default=30.0)
    parser.add_argument("--planner-backend", choices=["moveit", "pos_cmd"], default="moveit")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--enable", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--check-ik", action="store_true")
    parser.add_argument("--avoid-collisions", action="store_true")
    parser.add_argument("--command-rate", type=float, default=5.0)
    parser.add_argument("--max-initial-translation", type=float, default=0.08)
    parser.add_argument("--allow-large-motion", action="store_true")
    parser.add_argument("--use-command-orientation", action="store_true")
    parser.add_argument("--move-group-action", default="/move_action")
    parser.add_argument("--moveit-group-name", default="arm")
    parser.add_argument("--moveit-end-effector-link", default="link6")
    parser.add_argument("--gripper-backend", choices=["none", "action", "pos_cmd"], default="pos_cmd")
    parser.add_argument("--trajectory-backend", choices=["topic", "moveit_controller"], default="topic")
    parser.add_argument("--trajectory-topic", default="/arm_controller/joint_trajectory")
    parser.add_argument("--velocity-scaling", type=float, default=0.15)
    parser.add_argument("--acceleration-scaling", type=float, default=0.15)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--service-timeout", type=float, default=25.0)
    parser.add_argument("--position-tolerance", type=float, default=0.015)
    parser.add_argument("--orientation-tolerance", type=float, default=0.349066)
    parser.add_argument("--pregrasp-height", type=float, default=0.10)
    parser.add_argument("--grasp-dx", type=float, default=0.0)
    parser.add_argument("--grasp-dy", type=float, default=0.0)
    parser.add_argument("--grasp-dz", type=float, default=0.02)
    parser.add_argument("--tool-tip-offset-x", type=float, default=0.0)
    parser.add_argument("--tool-tip-offset-y", type=float, default=0.0)
    parser.add_argument("--tool-tip-offset-z", type=float, default=0.0)
    parser.add_argument("--skip-lift", action="store_true")
    parser.add_argument("--pregrasp-only", action="store_true")
    parser.add_argument("--max-trajectory-points", type=int, default=25)
    parser.add_argument("--max-point-sleep", type=float, default=0.25)
    parser.add_argument("--point-duration", type=float, default=0.20)
    parser.add_argument("--final-hold-time", type=float, default=1.0)
    parser.add_argument("--feedback-wait", type=float, default=0.5)
    parser.add_argument("--final-cartesian-correction", action="store_true")
    parser.add_argument("--end-pose-is-tip", action="store_true", default=True)
    parser.add_argument("--end-pose-is-link", dest="end_pose_is_tip", action="store_false")
    parser.add_argument("--final-correction-timeout", type=float, default=8.0)
    parser.add_argument("--final-correction-step", type=float, default=0.02)
    parser.add_argument("--final-correction-period", type=float, default=0.12)
    parser.add_argument("--final-correction-tolerance", type=float, default=0.015)
    parser.add_argument("--stable-samples", type=int, default=5)
    parser.add_argument("--stable-position-tolerance", type=float, default=0.030)
    parser.add_argument("--camera-x", type=float, default=0.0)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.13)
    parser.add_argument("--camera-roll", type=float, default=-1.57079632679)
    parser.add_argument("--camera-pitch", type=float, default=0.0)
    parser.add_argument("--camera-yaw", type=float, default=-1.57079632679)
    parser.add_argument("--axis-length-m", type=float, default=0.16)
    parser.add_argument("--axis-thickness", type=int, default=8)
    parser.add_argument("--min-radius", type=float, default=0.12)
    parser.add_argument("--max-radius", type=float, default=0.62)
    parser.add_argument("--min-z", type=float, default=0.02)
    parser.add_argument("--max-z", type=float, default=0.55)
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(main())
