# Stationary Gaze Test Protocol

## Goal

Validate case 1:

- The Aria glasses wearer is stationary.
- The robot or vehicle camera is stationary.
- The scene is dynamic.
- When the wearer fixates on a scene target, the robot-view overlay should indicate the same scene region.

This is not yet a full moving-platform validation. It is the simpler shared-scene consistency test.

## Core Idea

We compare:

- `Aria eye gaze -> Aria RGB projection`
- `Robot or vehicle camera -> mapped Aria gaze in ROS image space`

If the mapping is reasonable, then when the wearer looks at a stable scene target, the mapped ROS dot should also stabilize at the corresponding target region in the robot view.

## Inputs

You need:

- `gaze_stream.csv`
- `rgb_overlay.avi`
- `rgb_overlay_frames.csv` — one row per written RGB video frame, produced by
  the updated `aria_eye_viewer.py`. Used to map ROS bag frames to exact Aria
  video frame indices. Older recordings without this file fall back to a
  best-effort manifest synthesized from `gaze_stream.csv` (accuracy degraded;
  re-record for the real thing).
- ROS bag directory containing `metadata.yaml` and `.db3`

## Recording Workflows

### Split Two-Computer Setup

If the Aria glasses are connected to this laptop and the robot camera is on a different computer, use the split workflow:

On the robot computer:

```bash
./record_case1_robot_bag.sh
```

On the Aria laptop:

```bash
./record_case1_aria_only.sh -- --gaze-source open-model --gaze-model-device cpu
```

Start them as close together as practical, then begin the fixation routine a couple of seconds later.

### Single-Machine Setup

To record a new stationary case-1 run:

```bash
./record_case1_test.sh -- --gaze-source open-model --gaze-model-device cpu
```

This will:

- start `ros2 bag record` for the camera topics
- launch the Aria viewer with recording enabled
- stop the ROS bag when the Aria viewer exits
- print the exact stationary-review command for the captured run

## One-Command Review

Run:

```bash
python3 aria_teleop/align_aria_and_rosbag.py \
  --stationary-review \
  --aria-csv recordings/aria_eye_20260416_154251/gaze_stream.csv \
  --aria-video recordings/aria_eye_20260416_154251/rgb_overlay.avi \
  --rosbag-dir /path/to/rosbag_dir \
  --transform-mode anchor
```

This generates:

- mapped CSV in ROS image space
- ROS bag overlay video
- side-by-side Aria vs ROS comparison video
- stable-gaze segment CSV
- summary JSON

### Frame-Accurate Pairing

The review now drives the output from the ROS bag frame stream. For each ROS
frame it picks the nearest Aria RGB video frame via `rgb_overlay_frames.csv`
(wall-clock elapsed time), decodes that exact frame from `rgb_overlay.avi`,
and renders both the ROS overlay and the side-by-side tile together in a
single pass. ROS frames outside the shared Aria↔ROS window (default
tolerance 0.5 s, configurable via `--sync-tolerance-s`) are dropped instead
of extending the Aria side by freezing on its last frame. Expect the
side-by-side output duration to match the shared window, not the longer of
the two streams.

## What To Look For

### Qualitative

- When gaze moves left, the ROS overlay should move toward the matching left-side scene region.
- When gaze moves right, the ROS overlay should move right.
- During stable fixation, the ROS overlay should stop drifting and remain localized.
- Head-forward and gaze dots should not jump erratically between frames.

### Stable-Gaze Segments

The generated `stable_eye_segments.csv` lists intervals where Aria gaze is relatively steady.

For each segment, inspect:

- `duration_s`
- `aria_std_x`, `aria_std_y`
- `ros_std_x`, `ros_std_y`

Smaller ROS standard deviation during a stable Aria segment is better.

## Suggested Acceptance Criteria For Case 1

- Direction agreement:
  Left/right and up/down gaze changes produce the same directional change in ROS overlay.

- Stability:
  During a stable fixation segment, the ROS dot remains localized instead of drifting continuously.

- Scene agreement:
  In the side-by-side video, the ROS dot appears on the same scene object or region that the wearer is fixating on in the Aria view.

## Limitations

- A visually plausible overlay is not the same as calibrated geometric accuracy.
- For quantitative pixel-accuracy claims, you still need explicit scene correspondences or target annotations.
- This protocol is intended for the first stationary validation before moving to the vehicle-driving case.
