from pathlib import Path
import os
import sys
import tempfile

import numpy as np
import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from align_aria_and_rosbag import ImageCoordinateTransform, TimestampAligner


def _write_csv(tmp_path: Path, rows):
    csv_path = tmp_path / "gaze_stream.csv"
    pd.DataFrame(rows).to_csv(csv_path, index=False)
    return csv_path


def test_anchor_mapping_tracks_current_aria_position():
    with tempfile.TemporaryDirectory() as tmp_dir:
        csv_path = _write_csv(
            Path(tmp_dir),
            [
                {"model_dot_x": 100.0, "model_dot_y": 50.0},
                {"model_dot_x": 120.0, "model_dot_y": 65.0},
                {"model_dot_x": 90.0, "model_dot_y": 40.0},
            ],
        )
        mapped = TimestampAligner.map_aria_motion_to_ros_camera(
            aria_csv=csv_path,
            ros_width=640,
            ros_height=480,
            aria_width=1408,
            aria_height=1408,
            dots=("eye",),
            eye_ros_start=(320.0, 240.0),
            transform_mode="anchor",
        )

    scale_x = 640.0 / 1408.0
    scale_y = 480.0 / 1408.0
    expected_x = [320.0, 320.0 + 20.0 * scale_x, 320.0 - 10.0 * scale_x]
    expected_y = [240.0, 240.0 + 15.0 * scale_y, 240.0 - 10.0 * scale_y]

    assert list(mapped["eye_aria_anchor_x"]) == [100.0, 100.0, 100.0]
    assert list(mapped["eye_aria_anchor_y"]) == [50.0, 50.0, 50.0]
    assert np.allclose(mapped["eye_ros_x_unclipped"], expected_x)
    assert np.allclose(mapped["eye_ros_y_unclipped"], expected_y)
    assert list(mapped["eye_ros_x"]) == [320.0, round(expected_x[1]), round(expected_x[2])]
    assert list(mapped["eye_ros_y"]) == [240.0, round(expected_y[1]), round(expected_y[2])]


def test_affine_fit_maps_points_into_ros_image_space():
    point_pairs = [
        ((0.0, 0.0), (10.0, 20.0)),
        ((1.0, 0.0), (12.0, 19.0)),
        ((0.0, 1.0), (13.0, 24.0)),
        ((2.0, 1.0), (17.0, 22.0)),
    ]
    transform = ImageCoordinateTransform.fit_affine(point_pairs)

    mapped = transform.apply((3.0, 2.0))
    assert mapped is not None
    assert np.allclose(mapped, (22.0, 25.0))


def test_affine_mode_uses_fitted_transform_for_csv_mapping():
    with tempfile.TemporaryDirectory() as tmp_dir:
        csv_path = _write_csv(
            Path(tmp_dir),
            [
                {"model_dot_x": 0.0, "model_dot_y": 0.0},
                {"model_dot_x": 2.0, "model_dot_y": 1.0},
                {"model_dot_x": 3.0, "model_dot_y": 2.0},
            ],
        )
        mapped = TimestampAligner.map_aria_motion_to_ros_camera(
            aria_csv=csv_path,
            ros_width=200,
            ros_height=200,
            aria_width=1408,
            aria_height=1408,
            dots=("eye",),
            transform_mode="affine",
            affine_pairs=[
                [0.0, 0.0, 10.0, 20.0],
                [1.0, 0.0, 12.0, 19.0],
                [0.0, 1.0, 13.0, 24.0],
                [2.0, 1.0, 17.0, 22.0],
            ],
        )

    assert np.allclose(mapped["eye_ros_x_unclipped"], [10.0, 17.0, 22.0])
    assert np.allclose(mapped["eye_ros_y_unclipped"], [20.0, 22.0, 25.0])
    assert list(mapped["eye_ros_x"]) == [10.0, 17.0, 22.0]
    assert list(mapped["eye_ros_y"]) == [20.0, 22.0, 25.0]


def test_build_affine_mapping_command_includes_pairs_and_sizes():
    command = TimestampAligner.build_affine_mapping_command(
        point_pairs=[
            ((1.0, 2.0), (10.0, 20.0)),
            ((3.0, 4.0), (30.0, 40.0)),
        ],
        aria_csv=Path("/tmp/gaze_stream.csv"),
        output_csv=Path("/tmp/mapped.csv"),
        ros_width=640,
        ros_height=480,
        dots=["eye"],
    )

    assert "--transform-mode affine" in command
    assert "--aria-csv /tmp/gaze_stream.csv" in command
    assert "--output-csv /tmp/mapped.csv" in command
    assert "--ros-width 640" in command
    assert "--ros-height 480" in command
    assert "--dots eye" in command
    assert "--affine-pair 1.000 2.000 10.000 20.000" in command
    assert "--affine-pair 3.000 4.000 30.000 40.000" in command


def test_choose_rosbag_image_topic_prefers_nonempty_image_topic():
    with tempfile.TemporaryDirectory() as tmp_dir:
        metadata_path = Path(tmp_dir) / "metadata.yaml"
        metadata_path.write_text(
            """
rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata:
        name: /cam_1/image_raw
        type: sensor_msgs/msg/Image
      message_count: 0
    - topic_metadata:
        name: /cam_0/image_raw
        type: sensor_msgs/msg/Image
      message_count: 169
    - topic_metadata:
        name: /cam_0/camera_info
        type: sensor_msgs/msg/CameraInfo
      message_count: 169
""".strip(),
            encoding="utf-8",
        )

        topic = TimestampAligner.choose_rosbag_image_topic(metadata_path)
        assert topic == "/cam_0/image_raw"


def test_compute_stable_gaze_segments_finds_hold_interval():
    df = pd.DataFrame(
        {
            "wall_time_s": [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2],
            "model_dot_x": [100.0, 103.0, 105.0, 106.0, 180.0, 182.0, 183.0],
            "model_dot_y": [200.0, 201.0, 202.0, 201.0, 260.0, 262.0, 261.0],
            "eye_ros_x": [50.0, 51.0, 52.0, 52.0, 90.0, 91.0, 92.0],
            "eye_ros_y": [70.0, 70.0, 71.0, 70.0, 120.0, 121.0, 120.0],
        }
    )

    segments = TimestampAligner.compute_stable_gaze_segments(
        df,
        dot_name="eye",
        max_step_px=20.0,
        min_duration_s=0.5,
        min_samples=3,
    )

    assert len(segments) == 1
    segment = segments.iloc[0]
    assert abs(segment["start_time_s"] - 0.0) < 1e-6
    assert abs(segment["end_time_s"] - 0.6) < 1e-6
    assert segment["sample_count"] == 4
    assert segment["ros_std_x"] < 1.0


# ---------------------------------------------------------------------------
# RGB frame manifest + ROS<->Aria pairing
# ---------------------------------------------------------------------------


def test_load_rgb_overlay_manifest_prefers_explicit_file():
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        # Write a gaze_stream.csv that would otherwise be used as fallback.
        _write_csv(
            tmp,
            [
                {"wall_time_s": 1000.0, "rgb_timestamp_ns": 1, "model_dot_x": 1.0, "model_dot_y": 1.0},
                {"wall_time_s": 1000.1, "rgb_timestamp_ns": 2, "model_dot_x": 2.0, "model_dot_y": 2.0},
            ],
        )
        # Write the manifest with different content so we can tell which was loaded.
        manifest_path = tmp / "rgb_overlay_frames.csv"
        pd.DataFrame(
            {
                "video_frame_index": [0, 1, 2],
                "wall_time_s": [5000.0, 5000.033, 5000.066],
                "rgb_timestamp_ns": [100, 200, 300],
                "rgb_frames": [10, 11, 12],
                "model_dot_x": [0.0, 1.0, 2.0],
                "model_dot_y": [0.0, 1.0, 2.0],
                "head_dot_x": ["", "", ""],
                "head_dot_y": ["", "", ""],
                "yaw_rad": ["", "", ""],
                "pitch_rad": ["", "", ""],
            }
        ).to_csv(manifest_path, index=False)

        manifest = TimestampAligner.load_rgb_overlay_manifest(aria_csv=tmp / "gaze_stream.csv")

    assert list(manifest["video_frame_index"]) == [0, 1, 2]
    assert list(manifest["rgb_timestamp_ns"]) == [100, 200, 300]
    assert np.allclose(manifest["aria_rel_s"].to_numpy(), [0.0, 0.033, 0.066])


def test_load_rgb_overlay_manifest_synthesizes_from_gaze_csv_when_missing():
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        # Four rows, but rgb_timestamp_ns=10 repeats (eye-only updates between
        # RGB arrivals shouldn't create extra video frames).
        _write_csv(
            tmp,
            [
                {"wall_time_s": 1000.000, "rgb_timestamp_ns": 10, "model_dot_x": 1.0, "model_dot_y": 2.0},
                {"wall_time_s": 1000.010, "rgb_timestamp_ns": 10, "model_dot_x": 1.1, "model_dot_y": 2.1},
                {"wall_time_s": 1000.033, "rgb_timestamp_ns": 20, "model_dot_x": 3.0, "model_dot_y": 4.0},
                {"wall_time_s": 1000.066, "rgb_timestamp_ns": 30, "model_dot_x": 5.0, "model_dot_y": 6.0},
            ],
        )
        manifest = TimestampAligner.load_rgb_overlay_manifest(aria_csv=tmp / "gaze_stream.csv")

    # One row per unique rgb_timestamp_ns, preserving the first-seen order.
    assert list(manifest["rgb_timestamp_ns"]) == [10, 20, 30]
    assert list(manifest["video_frame_index"]) == [0, 1, 2]
    assert np.allclose(manifest["aria_rel_s"].to_numpy(), [0.0, 0.033, 0.066])


def test_pair_ros_frames_to_aria_frames_picks_nearest_neighbor():
    # Use well-separated targets so float rounding doesn't flip which neighbor wins.
    aria_rel_s = np.array([0.0, 1.0, 2.0, 3.0], dtype=np.float64)
    ros_rel_s = np.array([0.0, 1.4, 2.6, 3.0], dtype=np.float64)

    idx, valid = TimestampAligner.pair_ros_frames_to_aria_frames(aria_rel_s, ros_rel_s, max_tol_s=0.5)

    assert list(idx) == [0, 1, 3, 3]
    assert list(valid) == [True, True, True, True]


def test_pair_ros_frames_to_aria_frames_marks_end_overflow_invalid():
    # Aria ends at 12.0 s; ROS keeps going to 13.0 s.
    aria_rel_s = np.arange(0.0, 12.0 + 1e-9, 0.1)
    ros_rel_s = np.array([0.0, 6.0, 11.9, 12.1, 12.8, 13.0])

    idx, valid = TimestampAligner.pair_ros_frames_to_aria_frames(aria_rel_s, ros_rel_s, max_tol_s=0.5)

    # 12.1 is within tolerance of aria's last point 12.0; 12.8 and 13.0 are not.
    assert list(valid) == [True, True, True, True, False, False]
    # The "valid" later ones still pair to the last aria index.
    assert idx[-1] == len(aria_rel_s) - 1


def test_pair_ros_frames_to_aria_frames_marks_start_underflow_invalid():
    # Aria starts at 1.0 s (i.e. ROS started well before Aria).
    aria_rel_s = np.array([1.0, 1.033, 1.066, 1.1], dtype=np.float64)
    ros_rel_s = np.array([0.1, 0.45, 0.6, 1.05], dtype=np.float64)

    idx, valid = TimestampAligner.pair_ros_frames_to_aria_frames(aria_rel_s, ros_rel_s, max_tol_s=0.5)

    # 0.1 -> |0.1 - 1.0| = 0.9 > 0.5 -> invalid
    # 0.45 -> |0.45 - 1.0| = 0.55 > 0.5 -> invalid
    # 0.6 -> 0.4 <= 0.5 -> valid
    # 1.05 -> close to 1.066 -> valid
    assert list(valid) == [False, False, True, True]


def test_pair_ros_frames_to_aria_frames_handles_empty_inputs():
    idx, valid = TimestampAligner.pair_ros_frames_to_aria_frames(
        np.array([], dtype=np.float64), np.array([1.0, 2.0], dtype=np.float64)
    )
    assert list(idx) == [0, 0]
    assert list(valid) == [False, False]

    idx, valid = TimestampAligner.pair_ros_frames_to_aria_frames(
        np.array([0.0, 1.0], dtype=np.float64), np.array([], dtype=np.float64)
    )
    assert len(idx) == 0
    assert len(valid) == 0


def test_side_by_side_builder_chooses_correct_aria_indices_for_ros_stream():
    """End-to-end logic check without any video IO.

    Simulates the pairing choice made by render_synced_side_by_side for a
    stream where ROS runs 2x longer than Aria so the latter half of ROS
    frames should be dropped instead of freezing on the last Aria frame.
    """
    # Aria: 30 fps for 0.5 s (15 frames, elapsed 0..0.4666...)
    aria_rel_s = np.arange(0, 15) / 30.0
    # ROS: 10 fps for 1.0 s (10 frames, elapsed 0..0.9)
    ros_rel_s = np.arange(0, 10) / 10.0

    idx, valid = TimestampAligner.pair_ros_frames_to_aria_frames(aria_rel_s, ros_rel_s, max_tol_s=0.1)

    # First 5 ROS frames (0.0, 0.1, 0.2, 0.3, 0.4) fall within Aria's window.
    # Frame at 0.5 s is 0.033 beyond aria's last sample 0.4666, within tol 0.1.
    # Frames at 0.6..0.9 exceed tolerance and must be dropped.
    assert bool(valid[0]) and bool(valid[5])
    assert not bool(valid[6])
    assert not bool(valid[9])

    # Indices picked for the valid ROS frames should be monotonic and land on
    # plausible Aria frame indices (round(ros_rel_s * 30)).
    paired_aria_frames = [int(idx[k]) for k in range(len(ros_rel_s)) if valid[k]]
    assert paired_aria_frames == sorted(paired_aria_frames)
    assert paired_aria_frames[0] == 0
    assert paired_aria_frames[-1] == len(aria_rel_s) - 1

