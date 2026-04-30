# We need to align the coordinates of the Aria data with the ROS bag data, so that we can compare the two datasets and analyze the performance of the Aria system. This script will help us to achieve that by finding the offset between the two sets of timestamps and saving the aligned timestamps to a new file.

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import numpy as np
import pandas as pd
import yaml

try:
    from vendor.projectaria_eyetracking.projectaria_eyetracking.inference.data import data
except ImportError:
    data = None


@dataclass
class ImageCoordinateTransform:
    """Affine image-space transform from Aria RGB pixels to ROS image pixels."""

    matrix: np.ndarray
    description: str
    rmse_px: float | None = None

    def __post_init__(self):
        self.matrix = np.array(self.matrix, dtype=np.float64).reshape(3, 2)

    def apply(self, point: tuple[float, float] | np.ndarray | None):
        if point is None:
            return None
        xy = np.asarray(point, dtype=np.float64).reshape(-1)
        if xy.size < 2 or not np.all(np.isfinite(xy[:2])):
            return None
        mapped = np.array([xy[0], xy[1], 1.0], dtype=np.float64) @ self.matrix
        if not np.all(np.isfinite(mapped[:2])):
            return None
        return float(mapped[0]), float(mapped[1])

    def to_json(self):
        return {
            "matrix": self.matrix.tolist(),
            "description": self.description,
            "rmse_px": self.rmse_px,
        }

    @classmethod
    def from_anchor_scale(
        cls,
        aria_anchor: tuple[float, float],
        ros_anchor: tuple[float, float],
        aria_size: tuple[float, float],
        ros_size: tuple[float, float],
        gain: tuple[float, float],
    ):
        aria_w, aria_h = aria_size
        ros_w, ros_h = ros_size
        sx = ros_w / aria_w * gain[0]
        sy = ros_h / aria_h * gain[1]
        ax, ay = aria_anchor
        rx, ry = ros_anchor
        matrix = np.array(
            [
                [sx, 0.0],
                [0.0, sy],
                [rx - sx * ax, ry - sy * ay],
            ],
            dtype=np.float64,
        )
        return cls(
            matrix=matrix,
            description=(
                f"anchor-scale aria_anchor=({ax:.1f}, {ay:.1f}) "
                f"ros_anchor=({rx:.1f}, {ry:.1f}) scale=({sx:.3f}, {sy:.3f})"
            ),
            rmse_px=0.0,
        )

    @classmethod
    def fit_affine(cls, point_pairs):
        valid_pairs = []
        for aria_point, ros_point in point_pairs:
            aria_xy = np.asarray(aria_point, dtype=np.float64).reshape(-1)
            ros_xy = np.asarray(ros_point, dtype=np.float64).reshape(-1)
            if aria_xy.size < 2 or ros_xy.size < 2:
                continue
            if not (np.all(np.isfinite(aria_xy[:2])) and np.all(np.isfinite(ros_xy[:2]))):
                continue
            valid_pairs.append(((float(aria_xy[0]), float(aria_xy[1])), (float(ros_xy[0]), float(ros_xy[1]))))

        if len(valid_pairs) < 3:
            raise ValueError("Affine mapping requires at least 3 valid Aria/ROS point pairs.")

        a = np.array([[aria_x, aria_y, 1.0] for (aria_x, aria_y), _ in valid_pairs], dtype=np.float64)
        b = np.array([[ros_x, ros_y] for _, (ros_x, ros_y) in valid_pairs], dtype=np.float64)
        matrix, _residuals, rank, _singular_values = np.linalg.lstsq(a, b, rcond=None)
        if rank < 3:
            raise ValueError("Affine point pairs are degenerate; could not fit a full-rank transform.")

        residual = a @ matrix - b
        rmse_px = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        return cls(
            matrix=matrix,
            description=f"affine-fit pairs={len(valid_pairs)} rmse={rmse_px:.2f}px",
            rmse_px=rmse_px,
        )

    @classmethod
    def fit_weighted_affine(cls, point_pairs, weights=None, description_prefix: str = "affine-weighted"):
        """Weighted least-squares affine fit: min_A sum_i w_i ||A p_i - q_i||^2."""
        valid_pairs = []
        valid_weights = []
        for idx, pair in enumerate(point_pairs):
            aria_point, ros_point = pair
            aria_xy = np.asarray(aria_point, dtype=np.float64).reshape(-1)
            ros_xy = np.asarray(ros_point, dtype=np.float64).reshape(-1)
            if aria_xy.size < 2 or ros_xy.size < 2:
                continue
            if not (np.all(np.isfinite(aria_xy[:2])) and np.all(np.isfinite(ros_xy[:2]))):
                continue
            w = 1.0
            if weights is not None and idx < len(weights):
                try:
                    w = float(weights[idx])
                except (TypeError, ValueError):
                    w = 1.0
            if not np.isfinite(w) or w <= 0.0:
                continue
            valid_pairs.append(((float(aria_xy[0]), float(aria_xy[1])), (float(ros_xy[0]), float(ros_xy[1]))))
            valid_weights.append(w)

        if len(valid_pairs) < 3:
            raise ValueError("Weighted affine mapping requires at least 3 valid point pairs.")

        a = np.array([[aria_x, aria_y, 1.0] for (aria_x, aria_y), _ in valid_pairs], dtype=np.float64)
        b = np.array([[ros_x, ros_y] for _, (ros_x, ros_y) in valid_pairs], dtype=np.float64)
        w = np.asarray(valid_weights, dtype=np.float64)
        sqrt_w = np.sqrt(w)[:, None]
        a_w = a * sqrt_w
        b_w = b * sqrt_w

        matrix, _residuals, rank, _singular_values = np.linalg.lstsq(a_w, b_w, rcond=None)
        if rank < 3:
            raise ValueError("Weighted affine point pairs are degenerate; could not fit a full-rank transform.")

        residual = a @ matrix - b
        weighted_rmse = float(np.sqrt(np.average(np.sum(residual * residual, axis=1), weights=w)))
        return cls(
            matrix=matrix,
            description=f"{description_prefix} pairs={len(valid_pairs)} rmse={weighted_rmse:.2f}px",
            rmse_px=weighted_rmse,
        )


class TimestampAligner:
    DOT_COLUMNS = {
        "eye": ("model_dot_x", "model_dot_y"),
        "head": ("head_dot_x", "head_dot_y"),
    }

    def __init__(self):
        pass

    @staticmethod
    def align_timestamps(aria_file: Path, rosbag_file: Path, output_file: Path):
        # Load Aria timestamps
        with open(aria_file, "r") as f:
            aria_data = json.load(f)
        aria_timestamps = [entry["timestamp"] for entry in aria_data]

        # Load ROS bag timestamps
        rosbag_timestamps = []
        with open(rosbag_file, "r") as f:
            for line in f:
                timestamp_str = line.strip()
                if timestamp_str:
                    rosbag_timestamps.append(float(timestamp_str))

        # Find the offset between the two sets of timestamps
        if not aria_timestamps or not rosbag_timestamps:
            print("Error: One of the timestamp files is empty.")
            return

        offset = rosbag_timestamps[0] - aria_timestamps[0]
        print(f"Calculated offset: {offset} seconds")

        # Align Aria timestamps with ROS bag timestamps
        aligned_aria_data = []
        for entry in aria_data:
            aligned_entry = entry.copy()
            aligned_entry["timestamp"] += offset
            aligned_aria_data.append(aligned_entry)

        # Save the aligned timestamps to a new file
        with open(output_file, "w") as f:
            json.dump(aligned_aria_data, f, indent=4)
        print(f"Aligned timestamps saved to {output_file}")

    @staticmethod
    def transform_coordinates(aria_data, transformation_matrix):
        """
        aria_data: A list of dictionaries containing the Aria data, including coordinates. It has eye gaze data, head pose data, and other relevant information that we want to transform.
        transformation_matrix: A 4x4 matrix that defines the transformation from the Aria coordinate system to the ROS bag coordinate system.
        Purpose: This function would apply a transformation to the coordinates in the Aria data
        using the provided transformation matrix. This is necessary to ensure that the coordinates from the Aria system are in the same reference frame as the ROS bag data, allowing for accurate comparison and analysis.
        """
        transformed_data = []

        # Use pandas vectorization to loop through the Aria data and apply the transformation to the coordinates. This would involve multiplying the coordinate vectors by the transformation matrix to get the new coordinates in the ROS bag reference frame.

        # The only 2 things in the csv file is the timestamp and the coordinates. They are named in the csv file as 'rgb_timestamp_ns', 'eye_timestamp_ns', 'model_dot_x', 'model_dot_y', 'head_dot_x', and 'head_dot_y'. The coordinates are in the form of a list of 3 values (x, y, z) representing the position in the Aria coordinate system. The transformation matrix is a 4x4 matrix that includes rotation and translation components to convert the coordinates from the Aria system to the ROS bag system.

        pandas_data = pd.DataFrame(aria_data)
        for _index, row in pandas_data.iterrows():
            # Extract the coordinates from the row
            eye_coordinates = [row["model_dot_x"], row["model_dot_y"], 0]  # Assuming z=0 for 2D coordinates
            head_coordinates = [row["head_dot_x"], row["head_dot_y"], 0]  # Assuming z=0 for 2D coordinates

            # Convert to homogeneous coordinates (x, y, z, 1)
            homogeneous_coordinates_eye = eye_coordinates + [1]
            homogeneous_coordinates_head = head_coordinates + [1]

            # Apply the transformation. This is T(Aria coordinates -> ROS bag coordinates) = T(ROS bag) * T(Aria) * coordinates. The transformation matrix should be defined such that it transforms the Aria coordinates to the ROS bag coordinates. The multiplication will yield the new coordinates in the ROS bag reference frame. The resulting transformed_coordinates will be in homogeneous form, so we will need to convert it back to Cartesian coordinates by dividing by the homogeneous coordinate (the last element). This will give us the x, y, and z coordinates in the ROS bag reference frame, which we can then store or use for further analysis.
            transformed_coordinates_eye = transformation_matrix @ homogeneous_coordinates_eye
            transformed_coordinates_head = transformation_matrix @ homogeneous_coordinates_head

            # Convert back to Cartesian coordinates (x, y, z)
            transformed_coordinates_cartesian_eye = transformed_coordinates_eye[:3] / transformed_coordinates_eye[3]
            transformed_coordinates_cartesian_head = transformed_coordinates_head[:3] / transformed_coordinates_head[3]

            # Create a new entry with the transformed coordinates and the timestamp. We will store the transformed coordinates in a new dictionary that includes the original timestamp and any other relevant information from the Aria data. This way, we can keep track of the transformed coordinates along with their corresponding timestamps for further analysis and comparison with the ROS bag data.
            transformed_entry_eye = row.to_dict()
            transformed_entry_eye["timestamp"] = row["eye_timestamp_ns"]  # Use the eye timestamp for the entry
            transformed_entry_eye["transformed_x"] = transformed_coordinates_cartesian_eye[0]
            transformed_entry_eye["transformed_y"] = transformed_coordinates_cartesian_eye[1]
            transformed_entry_eye["transformed_z"] = transformed_coordinates_cartesian_eye[2]

            transformed_entry_head = row.to_dict()
            transformed_entry_head["timestamp"] = row["eye_timestamp_ns"]  # There is no separate head timestamp in gaze_stream.csv.
            transformed_entry_head["transformed_x"] = transformed_coordinates_cartesian_head[0]
            transformed_entry_head["transformed_y"] = transformed_coordinates_cartesian_head[1]
            transformed_entry_head["transformed_z"] = transformed_coordinates_cartesian_head[2]

            transformed_data.append(transformed_entry_eye)
            transformed_data.append(transformed_entry_head)

        return transformed_data

    @staticmethod
    def build_transformation_matrix(rotation_angles, translation_vector):
        """
        rotation_angles: A list of three angles (roll, pitch, yaw) that define the rotation from the Aria coordinate system to the ROS bag coordinate system.
        translation_vector: A list of three values (x, y, z) that define the translation from the Aria coordinate system to the ROS bag coordinate system.
        Purpose: This function would construct a 4x4 transformation matrix based on the provided rotation angles and translation vector. The transformation matrix would be used to convert coordinates from the Aria system to the ROS bag system.
        """
        # Convert rotation angles from degrees to radians
        roll = np.radians(rotation_angles[0])
        pitch = np.radians(rotation_angles[1])
        yaw = np.radians(rotation_angles[2])

        # Compute rotation matrices for roll, pitch, and yaw
        R_x = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )

        R_y = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        R_z = np.array(
            [
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1],
            ]
        )

        # Combined rotation matrix
        R = R_z @ R_y @ R_x

        # Construct the transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = translation_vector

        return transformation_matrix

    @staticmethod
    def save_transformed_data(transformed_data, output_file):
        """
        transformed_data: A list of dictionaries containing the transformed Aria data with the new coordinates in the ROS bag reference frame.
        output_file: The path to the file where the transformed data should be saved.
        Purpose: This function would save the transformed data to a new file, allowing us to have a record of the coordinates in the ROS bag reference frame for further analysis and comparison with the ROS bag data.
        """
        with open(output_file, "w") as f:
            json.dump(transformed_data, f, indent=4)
        print(f"Transformed data saved to {output_file}")

    @staticmethod
    def load_transformed_data(input_file):
        """
        input_file: The path to the file where the transformed data is saved.
        Purpose: This function would load the transformed data from the specified file, allowing us to access the coordinates in the ROS bag reference frame for analysis and comparison with the ROS bag data.
        """
        with open(input_file, "r") as f:
            transformed_data = json.load(f)
        return transformed_data

    @staticmethod
    def stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def is_finite_dot(row: pd.Series, x_col: str, y_col: str) -> bool:
        return pd.notna(row.get(x_col)) and pd.notna(row.get(y_col))

    @staticmethod
    def resolve_start_position(values, width: float, height: float) -> tuple[float, float]:
        if values is None:
            return (width - 1.0) * 0.5, (height - 1.0) * 0.5
        if len(values) != 2:
            raise ValueError("Start position must contain exactly two values: x y")
        return float(values[0]), float(values[1])

    @staticmethod
    def resolve_aria_anchor(
        df: pd.DataFrame,
        dot_name: str,
        values=None,
    ) -> tuple[float, float]:
        x_col, y_col = TimestampAligner.DOT_COLUMNS[dot_name]
        if values is not None:
            if len(values) != 2:
                raise ValueError("Aria anchor must contain exactly two values: x y")
            return float(values[0]), float(values[1])

        valid = df[[x_col, y_col]].dropna()
        if valid.empty:
            raise ValueError(f"Could not infer {dot_name} Aria anchor: no valid coordinates in CSV.")
        first = valid.iloc[0]
        return float(first[x_col]), float(first[y_col])

    @staticmethod
    def parse_affine_pairs(raw_pairs):
        if not raw_pairs:
            return []
        pairs = []
        for values in raw_pairs:
            if len(values) != 4:
                raise ValueError("Each --affine-pair must contain 4 numbers: aria_x aria_y ros_x ros_y")
            pairs.append(((float(values[0]), float(values[1])), (float(values[2]), float(values[3]))))
        return pairs

    @staticmethod
    def format_affine_cli_pairs(point_pairs):
        parts = []
        for aria_point, ros_point in point_pairs:
            parts.extend(
                [
                    "--affine-pair",
                    f"{float(aria_point[0]):.3f}",
                    f"{float(aria_point[1]):.3f}",
                    f"{float(ros_point[0]):.3f}",
                    f"{float(ros_point[1]):.3f}",
                ]
            )
        return parts

    @staticmethod
    def build_affine_mapping_command(
        point_pairs,
        aria_csv: Path | None = None,
        output_csv: Path | None = None,
        ros_width: int | None = None,
        ros_height: int | None = None,
        dots=None,
    ):
        command = [
            "python3",
            "aria_teleop/align_aria_and_rosbag.py",
            "--map-2d-motion",
            "--transform-mode",
            "affine",
        ]
        if aria_csv is not None:
            command.extend(["--aria-csv", str(aria_csv)])
        if output_csv is not None:
            command.extend(["--output-csv", str(output_csv)])
        if ros_width is not None:
            command.extend(["--ros-width", str(int(ros_width))])
        if ros_height is not None:
            command.extend(["--ros-height", str(int(ros_height))])
        if dots:
            command.extend(["--dots", *[str(dot) for dot in dots]])
        command.extend(TimestampAligner.format_affine_cli_pairs(point_pairs))
        return " ".join(shlex.quote(part) for part in command)

    @staticmethod
    def load_media_frame(media_path: Path, frame_index: int = 0):
        import cv2

        media_path = Path(media_path)
        if not media_path.is_file():
            raise FileNotFoundError(f"Media file not found: {media_path}")

        image_exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}
        suffix = media_path.suffix.lower()
        if suffix in image_exts:
            image = cv2.imread(str(media_path), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"Could not read image: {media_path}")
            return {
                "image": image,
                "frame_index": 0,
                "frame_count": 1,
                "is_video": False,
                "path": media_path,
            }

        capture = cv2.VideoCapture(str(media_path))
        if not capture.isOpened():
            raise RuntimeError(f"Could not open video: {media_path}")
        try:
            total_frames = int(max(capture.get(cv2.CAP_PROP_FRAME_COUNT), 0))
            if total_frames > 0:
                frame_index = int(np.clip(frame_index, 0, total_frames - 1))
            else:
                frame_index = max(0, int(frame_index))
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, image = capture.read()
            if not ok or image is None:
                raise RuntimeError(f"Could not read frame {frame_index} from {media_path}")
            return {
                "image": image,
                "frame_index": frame_index,
                "frame_count": total_frames,
                "is_video": True,
                "path": media_path,
            }
        finally:
            capture.release()

    @staticmethod
    def choose_rosbag_image_topic(metadata_path: Path, requested_topic: str | None = None) -> str:
        metadata_path = Path(metadata_path)
        if not metadata_path.is_file():
            raise FileNotFoundError(f"ROS bag metadata not found: {metadata_path}")

        with open(metadata_path, "r", encoding="utf-8") as file:
            metadata = yaml.safe_load(file) or {}

        bag_info = metadata.get("rosbag2_bagfile_information") or {}
        topics = bag_info.get("topics_with_message_count") or []
        image_topics = []
        for entry in topics:
            topic_metadata = entry.get("topic_metadata") or {}
            name = topic_metadata.get("name")
            msg_type = topic_metadata.get("type")
            message_count = int(entry.get("message_count", 0) or 0)
            if name and msg_type == "sensor_msgs/msg/Image":
                image_topics.append((name, message_count))

        if not image_topics:
            raise ValueError(f"No sensor_msgs/msg/Image topics found in {metadata_path}")

        if requested_topic is not None:
            for topic_name, message_count in image_topics:
                if topic_name == requested_topic:
                    if message_count <= 0:
                        raise ValueError(f"Requested image topic {requested_topic} exists but has no messages.")
                    return topic_name
            available = ", ".join(name for name, _count in image_topics)
            raise ValueError(f"Requested image topic {requested_topic} not found. Available image topics: {available}")

        nonempty = [name for name, message_count in image_topics if message_count > 0]
        if len(nonempty) == 1:
            return nonempty[0]
        if len(nonempty) > 1:
            available = ", ".join(nonempty)
            raise ValueError(
                "Multiple ROS image topics have messages. Please pass --ros-image-topic explicitly. "
                f"Candidates: {available}"
            )

        available = ", ".join(name for name, _count in image_topics)
        raise ValueError(f"Image topics were found but none have messages. Candidates: {available}")

    @staticmethod
    def image_msg_to_bgr8(msg):
        import cv2

        dtype = np.uint8
        bayer_conversions = {
            "bayer_rggb8": cv2.COLOR_BayerRG2BGR,
            "bayer_bggr8": cv2.COLOR_BayerBG2BGR,
            "bayer_gbrg8": cv2.COLOR_BayerGB2BGR,
            "bayer_grbg8": cv2.COLOR_BayerGR2BGR,
        }
        channels_by_encoding = {
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
            "mono8": 1,
            "8UC1": 1,
            "8UC3": 3,
            "8UC4": 4,
        }
        encoding = msg.encoding.lower()
        channels = channels_by_encoding.get(msg.encoding) or channels_by_encoding.get(encoding)
        if encoding in bayer_conversions:
            channels = 1
        if channels is None:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        row_width = int(msg.width) * channels
        raw = np.frombuffer(msg.data, dtype=dtype)
        expected_min = int(msg.step) * int(msg.height)
        if raw.size < expected_min:
            raise ValueError(
                f"Image data is shorter than expected: got {raw.size}, expected at least {expected_min}"
            )

        padded = raw[:expected_min].reshape((int(msg.height), int(msg.step)))
        image = padded[:, :row_width]
        if channels > 1:
            image = image.reshape((int(msg.height), int(msg.width), channels))
        else:
            image = image.reshape((int(msg.height), int(msg.width)))

        if msg.encoding in ("rgb8",):
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if msg.encoding in ("rgba8",):
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if msg.encoding in ("bgra8",):
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if msg.encoding in ("mono8", "8UC1"):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if encoding in bayer_conversions:
            return cv2.cvtColor(image, bayer_conversions[encoding])
        return image.copy()

    @staticmethod
    def iterate_rosbag_image_frames(rosbag_dir: Path, image_topic: str | None = None):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        rosbag_dir = Path(rosbag_dir)
        metadata_path = rosbag_dir / "metadata.yaml"
        selected_topic = TimestampAligner.choose_rosbag_image_topic(metadata_path, requested_topic=image_topic)

        storage = rosbag2_py.StorageOptions(uri=str(rosbag_dir), storage_id="sqlite3")
        converter = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, converter)

        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        message_type_name = topic_types.get(selected_topic)
        if message_type_name is None:
            raise ValueError(f"Could not resolve message type for topic {selected_topic}")
        message_type = get_message(message_type_name)

        frame_index = -1
        while reader.has_next():
            topic_name, serialized_data, timestamp_ns = reader.read_next()
            if topic_name != selected_topic:
                continue

            frame_index += 1
            message = deserialize_message(serialized_data, message_type)
            image = TimestampAligner.image_msg_to_bgr8(message)
            stamp_s = TimestampAligner.stamp_to_seconds(message.header.stamp)
            if stamp_s <= 0.0:
                stamp_s = float(timestamp_ns) * 1e-9
            yield {
                "image": image,
                "frame_index": frame_index,
                "timestamp_ns": int(timestamp_ns),
                "stamp_s": float(stamp_s),
                "topic": selected_topic,
                "header": message.header,
            }

    @staticmethod
    def load_rosbag_image_frame(rosbag_dir: Path, image_topic: str | None = None, frame_index: int = 0):
        target_index = max(0, int(frame_index))
        for frame in TimestampAligner.iterate_rosbag_image_frames(rosbag_dir, image_topic=image_topic):
            if frame["frame_index"] != target_index:
                continue
            return {
                "image": frame["image"],
                "frame_index": frame["frame_index"],
                "frame_count": None,
                "is_video": False,
                "path": Path(rosbag_dir),
                "topic": frame["topic"],
                "timestamp_ns": frame["timestamp_ns"],
            }

        selected_topic = TimestampAligner.choose_rosbag_image_topic(Path(rosbag_dir) / "metadata.yaml", requested_topic=image_topic)
        raise ValueError(f"Could not read frame {target_index} from topic {selected_topic} in {rosbag_dir}")

    @staticmethod
    def _compose_collection_canvas(aria_image, ros_image, point_pairs, pending_aria=None, pending_ros=None):
        import cv2

        padding = 24
        top_pad = 120
        max_display_height = 720

        aria_h, aria_w = aria_image.shape[:2]
        ros_h, ros_w = ros_image.shape[:2]
        aria_scale = min(1.0, max_display_height / max(aria_h, 1))
        ros_scale = min(1.0, max_display_height / max(ros_h, 1))

        aria_disp = cv2.resize(
            aria_image,
            (max(1, int(round(aria_w * aria_scale))), max(1, int(round(aria_h * aria_scale)))),
            interpolation=cv2.INTER_AREA if aria_scale < 1.0 else cv2.INTER_LINEAR,
        )
        ros_disp = cv2.resize(
            ros_image,
            (max(1, int(round(ros_w * ros_scale))), max(1, int(round(ros_h * ros_scale)))),
            interpolation=cv2.INTER_AREA if ros_scale < 1.0 else cv2.INTER_LINEAR,
        )

        canvas_h = top_pad + max(aria_disp.shape[0], ros_disp.shape[0]) + padding
        canvas_w = aria_disp.shape[1] + ros_disp.shape[1] + padding * 3
        canvas = np.full((canvas_h, canvas_w, 3), 18, dtype=np.uint8)

        aria_x0 = padding
        aria_y0 = top_pad
        ros_x0 = aria_x0 + aria_disp.shape[1] + padding
        ros_y0 = top_pad
        canvas[aria_y0 : aria_y0 + aria_disp.shape[0], aria_x0 : aria_x0 + aria_disp.shape[1]] = aria_disp
        canvas[ros_y0 : ros_y0 + ros_disp.shape[0], ros_x0 : ros_x0 + ros_disp.shape[1]] = ros_disp

        cv2.putText(canvas, "Aria RGB", (aria_x0, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "ROS camera", (ros_x0, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            "Left click corresponding points on both panes. s=save  u=undo  c=clear pending  r=reset  q=quit",
            (padding, 66),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (210, 210, 210),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"Pairs: {len(point_pairs)}  Need at least 3. Pending Aria: {pending_aria is not None}  Pending ROS: {pending_ros is not None}",
            (padding, 92),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (120, 255, 120),
            1,
            cv2.LINE_AA,
        )

        def draw_marker(side_x0, side_y0, scale, point, color, label):
            px = int(round(side_x0 + point[0] * scale))
            py = int(round(side_y0 + point[1] * scale))
            cv2.circle(canvas, (px, py), 8, color, 2, cv2.LINE_AA)
            cv2.drawMarker(canvas, (px, py), color, cv2.MARKER_CROSS, 16, 2)
            cv2.putText(canvas, label, (px + 8, max(18, py - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        for index, (aria_point, ros_point) in enumerate(point_pairs, start=1):
            color = (40 + (index * 43) % 200, 100 + (index * 67) % 150, 255 - (index * 29) % 180)
            draw_marker(aria_x0, aria_y0, aria_scale, aria_point, color, str(index))
            draw_marker(ros_x0, ros_y0, ros_scale, ros_point, color, str(index))

        if pending_aria is not None:
            draw_marker(aria_x0, aria_y0, aria_scale, pending_aria, (0, 255, 255), "pending")
        if pending_ros is not None:
            draw_marker(ros_x0, ros_y0, ros_scale, pending_ros, (0, 255, 255), "pending")

        return canvas, {
            "aria": {
                "x0": aria_x0,
                "y0": aria_y0,
                "width": aria_disp.shape[1],
                "height": aria_disp.shape[0],
                "scale": aria_scale,
            },
            "ros": {
                "x0": ros_x0,
                "y0": ros_y0,
                "width": ros_disp.shape[1],
                "height": ros_disp.shape[0],
                "scale": ros_scale,
            },
        }

    @staticmethod
    def collect_affine_pairs_interactive(aria_image, ros_image):
        import cv2

        state = {
            "pairs": [],
            "pending_aria": None,
            "pending_ros": None,
            "layout": None,
            "window": "Affine Point Collector",
        }

        def hit_test_point(side_name, x, y):
            rect = state["layout"][side_name]
            if not (rect["x0"] <= x < rect["x0"] + rect["width"] and rect["y0"] <= y < rect["y0"] + rect["height"]):
                return None
            orig_x = (x - rect["x0"]) / max(rect["scale"], 1e-9)
            orig_y = (y - rect["y0"]) / max(rect["scale"], 1e-9)
            return float(orig_x), float(orig_y)

        def on_mouse(event, x, y, _flags, _userdata):
            if event != cv2.EVENT_LBUTTONDOWN or state["layout"] is None:
                return
            aria_point = hit_test_point("aria", x, y)
            ros_point = hit_test_point("ros", x, y)
            if aria_point is not None:
                state["pending_aria"] = aria_point
            elif ros_point is not None:
                state["pending_ros"] = ros_point
            else:
                return

            if state["pending_aria"] is not None and state["pending_ros"] is not None:
                state["pairs"].append((state["pending_aria"], state["pending_ros"]))
                state["pending_aria"] = None
                state["pending_ros"] = None

        cv2.namedWindow(state["window"], cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(state["window"], on_mouse)

        try:
            while True:
                canvas, layout = TimestampAligner._compose_collection_canvas(
                    aria_image,
                    ros_image,
                    state["pairs"],
                    pending_aria=state["pending_aria"],
                    pending_ros=state["pending_ros"],
                )
                state["layout"] = layout
                cv2.imshow(state["window"], canvas)
                key = cv2.waitKey(30) & 0xFF

                if key in (27, ord("q")):
                    return None
                if key == ord("u"):
                    if state["pairs"]:
                        state["pairs"].pop()
                elif key == ord("c"):
                    state["pending_aria"] = None
                    state["pending_ros"] = None
                elif key == ord("r"):
                    state["pairs"].clear()
                    state["pending_aria"] = None
                    state["pending_ros"] = None
                elif key == ord("s"):
                    if len(state["pairs"]) >= 3:
                        return list(state["pairs"])
                    print("[collector] Need at least 3 point pairs before saving.")
        finally:
            cv2.destroyWindow(state["window"])

    @staticmethod
    def save_affine_collection(path: Path, payload: dict):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as file:
            json.dump(payload, file, indent=2)
        print(f"[collector] Saved affine calibration summary to {path}")

    @staticmethod
    def compute_stable_gaze_segments(
        df: pd.DataFrame,
        dot_name: str = "eye",
        time_column: str = "wall_time_s",
        max_step_px: float = 35.0,
        min_duration_s: float = 0.6,
        min_samples: int = 5,
    ) -> pd.DataFrame:
        x_col, y_col = TimestampAligner.DOT_COLUMNS[dot_name]
        if x_col not in df.columns or y_col not in df.columns:
            raise ValueError(f"Missing required columns for stable-gaze analysis: {x_col}, {y_col}")
        if time_column not in df.columns:
            raise ValueError(f"Missing required time column for stable-gaze analysis: {time_column}")

        valid = df[[time_column, x_col, y_col]].copy()
        valid = valid.dropna(subset=[time_column, x_col, y_col]).reset_index()
        if valid.empty:
            return pd.DataFrame()

        valid[time_column] = valid[time_column].astype(float)
        valid[x_col] = valid[x_col].astype(float)
        valid[y_col] = valid[y_col].astype(float)

        points = valid[[x_col, y_col]].to_numpy(dtype=np.float64)
        step_px = np.zeros(len(valid), dtype=np.float64)
        if len(valid) > 1:
            step_px[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1)
        valid["step_px"] = step_px
        valid["new_segment"] = (valid["step_px"] > float(max_step_px)).astype(int)
        if len(valid):
            valid.loc[0, "new_segment"] = 1
        valid["segment_id"] = valid["new_segment"].cumsum()

        segments = []
        for segment_id, group in valid.groupby("segment_id", sort=True):
            start_t = float(group[time_column].iloc[0])
            end_t = float(group[time_column].iloc[-1])
            duration_s = end_t - start_t
            if duration_s < float(min_duration_s):
                continue
            if len(group) < int(min_samples):
                continue

            segment_rows = df.loc[group["index"]]
            row = {
                "segment_id": int(segment_id),
                "sample_count": int(len(group)),
                "start_time_s": start_t,
                "end_time_s": end_t,
                "duration_s": duration_s,
                "aria_mean_x": float(group[x_col].mean()),
                "aria_mean_y": float(group[y_col].mean()),
                "aria_std_x": float(group[x_col].std(ddof=0)),
                "aria_std_y": float(group[y_col].std(ddof=0)),
                "aria_mean_step_px": float(group["step_px"].mean()),
                "source_column_x": x_col,
                "source_column_y": y_col,
            }

            ros_x_col = f"{dot_name}_ros_x"
            ros_y_col = f"{dot_name}_ros_y"
            if ros_x_col in segment_rows.columns and ros_y_col in segment_rows.columns:
                ros_valid = segment_rows[[ros_x_col, ros_y_col]].dropna()
                if not ros_valid.empty:
                    row.update(
                        {
                            "ros_mean_x": float(ros_valid[ros_x_col].astype(float).mean()),
                            "ros_mean_y": float(ros_valid[ros_y_col].astype(float).mean()),
                            "ros_std_x": float(ros_valid[ros_x_col].astype(float).std(ddof=0)),
                            "ros_std_y": float(ros_valid[ros_y_col].astype(float).std(ddof=0)),
                        }
                    )

            segments.append(row)

        return pd.DataFrame(segments)

    @staticmethod
    def save_dataframe(df: pd.DataFrame, path: Path, label: str):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(path, index=False)
        print(f"[review] Saved {label} to {path}")

    @staticmethod
    def build_side_by_side_video(
        aria_video: Path,
        ros_video: Path,
        output_video: Path,
        aria_height: int = 768,
        fps: float = 10.0,
        trim_duration_s: float | None = None,
    ):
        output_video = Path(output_video)
        output_video.parent.mkdir(parents=True, exist_ok=True)

        cmd = ["ffmpeg", "-y"]
        if trim_duration_s is not None and trim_duration_s > 0:
            cmd.extend(["-t", f"{float(trim_duration_s):.3f}"])
        cmd.extend(
            [
                "-i",
                str(aria_video),
                "-i",
                str(ros_video),
                "-filter_complex",
                (
                    f"[0:v]setpts=PTS-STARTPTS,fps={float(fps):.3f},scale=-2:{int(aria_height)}[left];"
                    f"[1:v]setpts=PTS-STARTPTS,scale=-2:{int(aria_height)}[right];"
                    "[left][right]hstack=inputs=2[v]"
                ),
                "-map",
                "[v]",
                "-shortest",
                str(output_video),
            ]
        )
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[review] Saved side-by-side video to {output_video}")

    @staticmethod
    def add_delta_follow_columns(
        df: pd.DataFrame,
        dot_name: str,
        aria_size: tuple[float, float],
        ros_size: tuple[float, float],
        ros_start: tuple[float, float],
        gain: tuple[float, float],
    ):
        """
        Legacy stateful mapping that accumulates frame-to-frame Aria deltas.

        This mode is kept as a fallback for backwards compatibility. The new
        direct mapping path is preferred because it computes each ROS dot from
        the current Aria coordinates instead of integrating motion over time.
        """
        x_col, y_col = TimestampAligner.DOT_COLUMNS[dot_name]
        if x_col not in df.columns or y_col not in df.columns:
            raise ValueError(f"Missing required columns for {dot_name}: {x_col}, {y_col}")

        aria_w, aria_h = aria_size
        ros_w, ros_h = ros_size
        scale = np.array([ros_w / aria_w * gain[0], ros_h / aria_h * gain[1]], dtype=np.float64)
        ros_position = np.array(ros_start, dtype=np.float64)
        previous_aria = None

        ros_x = []
        ros_y = []
        ros_dx = []
        ros_dy = []
        aria_delta_x = []
        aria_delta_y = []

        for _index, row in df.iterrows():
            if not TimestampAligner.is_finite_dot(row, x_col, y_col):
                ros_x.append(np.nan)
                ros_y.append(np.nan)
                ros_dx.append(np.nan)
                ros_dy.append(np.nan)
                aria_delta_x.append(np.nan)
                aria_delta_y.append(np.nan)
                continue

            current_aria = np.array([float(row[x_col]), float(row[y_col])], dtype=np.float64)
            if previous_aria is None:
                delta_aria = np.array([0.0, 0.0], dtype=np.float64)
            else:
                delta_aria = current_aria - previous_aria
                ros_position = ros_position + delta_aria * scale

            previous_aria = current_aria
            clipped = np.array(
                [np.clip(ros_position[0], 0, ros_w - 1), np.clip(ros_position[1], 0, ros_h - 1)],
                dtype=np.float64,
            )
            ros_position = clipped
            ros_x.append(round(float(clipped[0])))
            ros_y.append(round(float(clipped[1])))
            ros_dx.append(round(float(clipped[0] - ros_start[0]), 3))
            ros_dy.append(round(float(clipped[1] - ros_start[1]), 3))
            aria_delta_x.append(float(delta_aria[0]))
            aria_delta_y.append(float(delta_aria[1]))

        df[f"{dot_name}_ros_start_x"] = ros_start[0]
        df[f"{dot_name}_ros_start_y"] = ros_start[1]
        df[f"{dot_name}_aria_delta_x"] = aria_delta_x
        df[f"{dot_name}_aria_delta_y"] = aria_delta_y
        df[f"{dot_name}_ros_x"] = ros_x
        df[f"{dot_name}_ros_y"] = ros_y
        df[f"{dot_name}_ros_dx"] = ros_dx
        df[f"{dot_name}_ros_dy"] = ros_dy

    @staticmethod
    def add_direct_map_columns(
        df: pd.DataFrame,
        dot_name: str,
        ros_size: tuple[float, float],
        aria_anchor: tuple[float, float],
        ros_anchor: tuple[float, float],
        transform: ImageCoordinateTransform,
    ):
        """
        Adds ROS-image-space dot columns using a direct Aria->ROS image transform.

        Each ROS dot is computed from the current Aria dot for that frame, which
        keeps the overlay tied to the observed Aria coordinates even if frames are
        dropped, bag playback restarts, or the stream is paused and resumed.
        """
        x_col, y_col = TimestampAligner.DOT_COLUMNS[dot_name]
        if x_col not in df.columns or y_col not in df.columns:
            raise ValueError(f"Missing required columns for {dot_name}: {x_col}, {y_col}")

        ros_w, ros_h = ros_size

        ros_x = []
        ros_y = []
        ros_dx = []
        ros_dy = []
        ros_x_unclipped = []
        ros_y_unclipped = []

        for _index, row in df.iterrows():
            if not TimestampAligner.is_finite_dot(row, x_col, y_col):
                ros_x.append(np.nan)
                ros_y.append(np.nan)
                ros_dx.append(np.nan)
                ros_dy.append(np.nan)
                ros_x_unclipped.append(np.nan)
                ros_y_unclipped.append(np.nan)
                continue

            current_aria = np.array([float(row[x_col]), float(row[y_col])], dtype=np.float64)
            mapped = transform.apply(current_aria)
            if mapped is None:
                ros_x.append(np.nan)
                ros_y.append(np.nan)
                ros_dx.append(np.nan)
                ros_dy.append(np.nan)
                ros_x_unclipped.append(np.nan)
                ros_y_unclipped.append(np.nan)
                continue

            mapped_vec = np.array([float(mapped[0]), float(mapped[1])], dtype=np.float64)
            ros_x_unclipped.append(float(mapped_vec[0]))
            ros_y_unclipped.append(float(mapped_vec[1]))
            clipped = np.array(
                [np.clip(mapped_vec[0], 0, ros_w - 1), np.clip(mapped_vec[1], 0, ros_h - 1)],
                dtype=np.float64,
            )
            ros_x.append(round(float(clipped[0])))
            ros_y.append(round(float(clipped[1])))
            ros_dx.append(round(float(clipped[0] - ros_anchor[0]), 3))
            ros_dy.append(round(float(clipped[1] - ros_anchor[1]), 3))

        df[f"{dot_name}_aria_anchor_x"] = aria_anchor[0]
        df[f"{dot_name}_aria_anchor_y"] = aria_anchor[1]
        df[f"{dot_name}_ros_anchor_x"] = ros_anchor[0]
        df[f"{dot_name}_ros_anchor_y"] = ros_anchor[1]
        df[f"{dot_name}_ros_start_x"] = ros_anchor[0]
        df[f"{dot_name}_ros_start_y"] = ros_anchor[1]
        df[f"{dot_name}_ros_x_unclipped"] = ros_x_unclipped
        df[f"{dot_name}_ros_y_unclipped"] = ros_y_unclipped
        df[f"{dot_name}_ros_x"] = ros_x
        df[f"{dot_name}_ros_y"] = ros_y
        df[f"{dot_name}_ros_dx"] = ros_dx
        df[f"{dot_name}_ros_dy"] = ros_dy
        # In anchor mode, these are the exact x/y scale factors. In affine mode
        # they are still useful "sx-like/sy-like" coefficients for debugging.
        df[f"{dot_name}_sx"] = float(transform.matrix[0, 0])
        df[f"{dot_name}_sy"] = float(transform.matrix[1, 1])

    @staticmethod
    def map_aria_motion_to_ros_camera(
        aria_csv: Path,
        ros_width: float,
        ros_height: float,
        aria_width: float = 1408.0,
        aria_height: float = 1408.0,
        dots=("eye", "head"),
        eye_ros_start=None,
        head_ros_start=None,
        eye_aria_anchor=None,
        head_aria_anchor=None,
        gain_x: float = 1.0,
        gain_y: float = 1.0,
        transform_mode: str = "anchor",
        affine_pairs=None,
        affine_transform_override: ImageCoordinateTransform | None = None,
    ):
        df = pd.read_csv(aria_csv)
        aria_size = (float(aria_width), float(aria_height))
        ros_size = (float(ros_width), float(ros_height))
        gain = (float(gain_x), float(gain_y))
        transform_mode = str(transform_mode).lower()
        affine_pairs = TimestampAligner.parse_affine_pairs(affine_pairs)

        if affine_pairs and transform_mode == "anchor":
            transform_mode = "affine"

        if transform_mode not in {"anchor", "delta", "affine"}:
            raise ValueError(f"Unsupported transform mode: {transform_mode}")

        affine_transform = None
        if transform_mode == "affine":
            if affine_transform_override is not None:
                affine_transform = affine_transform_override
            else:
                affine_transform = ImageCoordinateTransform.fit_affine(affine_pairs)
            print(f"[mapping] Using {affine_transform.description}")

        for dot_name in dots:
            start_values = eye_ros_start if dot_name == "eye" else head_ros_start
            aria_anchor_values = eye_aria_anchor if dot_name == "eye" else head_aria_anchor

            if transform_mode == "delta":
                ros_start = TimestampAligner.resolve_start_position(start_values, ros_size[0], ros_size[1])
                TimestampAligner.add_delta_follow_columns(
                    df,
                    dot_name,
                    aria_size,
                    ros_size,
                    ros_start,
                    gain,
                )
                print(
                    f"[{dot_name}] delta-follow start=({ros_start[0]:.1f}, {ros_start[1]:.1f}), "
                    f"scale=({ros_size[0] / aria_size[0]:.3f}, {ros_size[1] / aria_size[1]:.3f}), "
                    f"gain=({gain[0]:.3f}, {gain[1]:.3f})"
                )
                continue

            aria_anchor = TimestampAligner.resolve_aria_anchor(df, dot_name, values=aria_anchor_values)
            if transform_mode == "affine":
                transform = affine_transform
                ros_anchor = transform.apply(aria_anchor)
                if ros_anchor is None:
                    raise ValueError(f"Could not map {dot_name} anchor {aria_anchor} through affine transform.")
                if start_values is not None:
                    print(f"[{dot_name}] NOTE: ignoring --{dot_name}-ros-start in affine mode.")
            else:
                ros_anchor = TimestampAligner.resolve_start_position(start_values, ros_size[0], ros_size[1])
                transform = ImageCoordinateTransform.from_anchor_scale(
                    aria_anchor=aria_anchor,
                    ros_anchor=ros_anchor,
                    aria_size=aria_size,
                    ros_size=ros_size,
                    gain=gain,
                )

            TimestampAligner.add_direct_map_columns(
                df,
                dot_name,
                ros_size,
                aria_anchor,
                ros_anchor,
                transform,
            )
            print(f"[{dot_name}] direct-map {transform.description}")

        return df

    @staticmethod
    def draw_dot(image, dot, color, label):
        if dot is None:
            return
        import cv2

        x, y = int(round(dot[0])), int(round(dot[1]))
        cv2.circle(image, (x, y), 16, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.circle(image, (x, y), 11, color, 3, cv2.LINE_AA)
        cv2.circle(image, (x, y), 4, color, -1, cv2.LINE_AA)
        cv2.putText(
            image,
            label,
            (max(5, x + 14), max(18, y - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def draw_trail(image, points, color):
        if len(points) < 2:
            return
        import cv2

        pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(image, [pts], False, color, 2, cv2.LINE_AA)

    @staticmethod
    def draw_status_panel(image, lines):
        import cv2

        line_height = 22
        width = min(image.shape[1] - 12, 760)
        height = 12 + line_height * len(lines)
        overlay = image.copy()
        cv2.rectangle(overlay, (6, 6), (6 + width, 6 + height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, image, 0.45, 0, image)
        for idx, line in enumerate(lines):
            cv2.putText(
                image,
                line,
                (14, 28 + idx * line_height),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    @staticmethod
    def parse_2d_motion_args(argv=None):
        parser = argparse.ArgumentParser(
            description="Map Aria 2D eye/head dot movement into a ROS camera stream or CSV."
        )
        parser.add_argument("--map-2d-motion", action="store_true", help="Use the 2D dot-motion mapping path.")
        parser.add_argument("--aria-csv", type=Path, required=True, help="Path to gaze_stream.csv.")
        parser.add_argument("--output-csv", type=Path, default=None, help="Optional mapped CSV output path.")
        parser.add_argument("--aria-width", type=float, default=1408.0)
        parser.add_argument("--aria-height", type=float, default=1408.0)
        parser.add_argument("--ros-width", type=float, default=None, help="ROS image width for CSV-only mapping.")
        parser.add_argument("--ros-height", type=float, default=None, help="ROS image height for CSV-only mapping.")
        parser.add_argument("--dots", nargs="+", choices=sorted(TimestampAligner.DOT_COLUMNS), default=["eye", "head"])
        parser.add_argument(
            "--transform-mode",
            choices=["anchor", "delta", "affine"],
            default="anchor",
            help="anchor maps each frame directly from Aria pixels, delta keeps the legacy accumulated-delta behavior, affine fits a full 2D image transform from paired points.",
        )
        parser.add_argument("--eye-ros-start", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument("--head-ros-start", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument("--eye-aria-anchor", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument("--head-aria-anchor", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument(
            "--affine-pair",
            nargs=4,
            type=float,
            action="append",
            default=None,
            metavar=("ARIA_X", "ARIA_Y", "ROS_X", "ROS_Y"),
            help="Paired Aria/ROS pixels for affine mode. Repeat at least 3 times.",
        )
        parser.add_argument("--gain-x", type=float, default=1.0)
        parser.add_argument("--gain-y", type=float, default=1.0)
        parser.add_argument("--overlay-ros", action="store_true", help="Subscribe to ROS images and publish annotated images.")
        parser.add_argument("--overlay-rosbag", action="store_true", help="Read frames directly from a ROS bag and render the mapped overlay offline.")
        parser.add_argument("--rosbag-dir", type=Path, default=None, help="ROS bag directory for offline overlay mode.")
        parser.add_argument("--ros-image-topic", default=None, help="ROS image topic for bag overlay mode, for example /cam_0/image_raw.")
        parser.add_argument("--overlay-video", type=Path, default=None, help="Optional output video path for offline ROS bag overlay.")
        parser.add_argument("--image-topic", default="/cam_0/image_raw")
        parser.add_argument("--overlay-topic", default="/cam_0/aria_dot_overlay")
        parser.add_argument("--time-column", default="wall_time_s")
        parser.add_argument("--loop-motion", action="store_true", help="Loop Aria CSV motion when bag playback loops.")
        parser.add_argument("--display-window", action="store_true", help="Also show the overlay in an OpenCV window.")
        parser.add_argument("--max-frames", type=int, default=0, help="Maximum number of ROS frames to process in offline overlay mode. 0 means all frames.")
        parser.add_argument("--debug-overlay", action="store_true", help="Draw trails and row/time status text.")
        parser.add_argument("--trail-length", type=int, default=40, help="Number of recent dot positions to draw in debug overlay.")
        parser.add_argument("--debug-print-every", type=int, default=30, help="Print mapping status every N frames; 0 disables.")
        return parser.parse_args(argv)

    @staticmethod
    def parse_affine_collection_args(argv=None):
        parser = argparse.ArgumentParser(
            description="Collect matching Aria/ROS image points and fit an affine image-to-image mapping."
        )
        parser.add_argument("--collect-affine-points", action="store_true", help="Open the interactive affine point collector.")
        parser.add_argument("--aria-media", type=Path, required=True, help="Aria RGB image or video, for example rgb_overlay.avi.")
        parser.add_argument("--ros-media", type=Path, default=None, help="ROS camera image or video frame export.")
        parser.add_argument("--rosbag-dir", type=Path, default=None, help="ROS bag directory containing metadata.yaml and .db3 files.")
        parser.add_argument("--ros-image-topic", default=None, help="ROS image topic to extract from the bag, for example /cam_0/image_raw.")
        parser.add_argument("--aria-frame-index", type=int, default=0, help="Frame index to load from Aria video sources.")
        parser.add_argument("--ros-frame-index", type=int, default=0, help="Frame index to load from ROS video sources.")
        parser.add_argument("--pairs-json", type=Path, default=None, help="Optional output path for the collected pairs and fitted affine transform.")
        parser.add_argument("--aria-csv", type=Path, default=None, help="Optional gaze_stream.csv path for printing or running the follow-up affine mapping command.")
        parser.add_argument("--output-csv", type=Path, default=None, help="Optional mapped CSV path to generate immediately after collecting points.")
        parser.add_argument("--dots", nargs="+", choices=sorted(TimestampAligner.DOT_COLUMNS), default=["eye", "head"])
        parser.add_argument("--run-after-collect", action="store_true", help="Immediately generate the mapped CSV after the affine points are collected. Requires --aria-csv and --output-csv.")
        return parser.parse_args(argv)

    @staticmethod
    def parse_stationary_review_args(argv=None):
        parser = argparse.ArgumentParser(
            description="Generate a stationary-test review package: mapped CSV, ROS bag overlay, side-by-side video, and stable-gaze summaries."
        )
        parser.add_argument("--stationary-review", action="store_true", help="Run the stationary case-1 review workflow.")
        parser.add_argument("--aria-csv", type=Path, required=True, help="Path to gaze_stream.csv.")
        parser.add_argument("--aria-video", type=Path, required=True, help="Path to Aria RGB overlay video, for example rgb_overlay.avi.")
        parser.add_argument("--rosbag-dir", type=Path, required=True, help="ROS bag directory containing metadata.yaml and .db3 files.")
        parser.add_argument("--ros-image-topic", default=None, help="Optional ROS image topic, for example /cam_0/image_raw.")
        parser.add_argument("--output-dir", type=Path, default=None, help="Directory for review outputs. Defaults beside the Aria CSV.")
        parser.add_argument("--dots", nargs="+", choices=sorted(TimestampAligner.DOT_COLUMNS), default=["eye", "head"])
        parser.add_argument("--transform-mode", choices=["anchor", "delta", "affine"], default="anchor")
        parser.add_argument("--affine-pair", nargs=4, type=float, action="append", default=None, metavar=("ARIA_X", "ARIA_Y", "ROS_X", "ROS_Y"))
        parser.add_argument("--eye-ros-start", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument("--head-ros-start", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument("--eye-aria-anchor", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument("--head-aria-anchor", nargs=2, type=float, default=None, metavar=("X", "Y"))
        parser.add_argument("--aria-width", type=float, default=1408.0)
        parser.add_argument("--aria-height", type=float, default=1408.0)
        parser.add_argument("--gain-x", type=float, default=1.0)
        parser.add_argument("--gain-y", type=float, default=1.0)
        parser.add_argument("--time-column", default="wall_time_s")
        parser.add_argument(
            "--auto-affine-gaze-fit",
            action="store_true",
            help=(
                "Auto-fit affine mapping (no clicking): temporal alignment first, then gaze-neighborhood "
                "feature matching + RANSAC + weighted affine fit + holdout validation."
            ),
        )
        parser.add_argument("--auto-affine-offset-range-s", type=float, default=0.12)
        parser.add_argument("--auto-affine-offset-step-s", type=float, default=0.015)
        parser.add_argument("--auto-affine-temporal-sample-step", type=int, default=8)
        parser.add_argument("--auto-affine-fit-sample-step", type=int, default=4)
        parser.add_argument("--auto-affine-patch-radius-px", type=float, default=180.0)
        parser.add_argument("--auto-affine-min-matches", type=int, default=14)
        parser.add_argument("--auto-affine-min-inliers", type=int, default=8)
        parser.add_argument("--auto-affine-max-reproj-px", type=float, default=4.0)
        parser.add_argument("--auto-affine-holdout-mod", type=int, default=5)
        parser.add_argument(
            "--mapped-time-offset-s",
            type=float,
            default=0.0,
            help=(
                "Extra time shift (seconds) for selecting the mapped CSV row after Aria-frame pairing. "
                "Positive values pick slightly later gaze rows; negative values pick earlier rows."
            ),
        )
        parser.add_argument(
            "--debug-eye-x",
            action="store_true",
            help="Overlay and print eye x debug values: aria_x, ros_x, sx, and anchor-relative deltas.",
        )
        parser.add_argument("--debug-print-every", type=int, default=50)
        parser.add_argument("--max-frames", type=int, default=0)
        parser.add_argument("--stable-dot", choices=sorted(TimestampAligner.DOT_COLUMNS), default="eye")
        parser.add_argument("--stable-max-step-px", type=float, default=35.0)
        parser.add_argument("--stable-min-duration-s", type=float, default=0.6)
        parser.add_argument("--stable-min-samples", type=int, default=5)
        parser.add_argument(
            "--sync-tolerance-s",
            type=float,
            default=0.5,
            help=(
                "Max seconds between a ROS frame and its paired Aria frame before "
                "the pair is dropped from the review output. Lower for stricter clipping."
            ),
        )
        parser.add_argument(
            "--sync-basis",
            choices=["relative", "epoch"],
            default="relative",
            help=(
                "How to pair ROS frames to Aria frames: 'relative' aligns by each stream's "
                "local elapsed time; 'epoch' aligns by absolute wall-time stamps."
            ),
        )
        return parser.parse_args(argv)

    @staticmethod
    def run_affine_collection_mode(argv=None):
        args = TimestampAligner.parse_affine_collection_args(argv)
        aria_media = TimestampAligner.load_media_frame(args.aria_media, frame_index=args.aria_frame_index)
        if args.ros_media is not None and args.rosbag_dir is not None:
            raise SystemExit("Use either --ros-media or --rosbag-dir, not both.")
        if args.ros_media is None and args.rosbag_dir is None:
            raise SystemExit("Provide either --ros-media or --rosbag-dir.")

        if args.rosbag_dir is not None:
            ros_media = TimestampAligner.load_rosbag_image_frame(
                args.rosbag_dir,
                image_topic=args.ros_image_topic,
                frame_index=args.ros_frame_index,
            )
        else:
            ros_media = TimestampAligner.load_media_frame(args.ros_media, frame_index=args.ros_frame_index)

        print(
            "[collector] Loaded Aria media "
            f"{aria_media['path']} frame={aria_media['frame_index']} size={aria_media['image'].shape[1]}x{aria_media['image'].shape[0]}"
        )
        ros_source_label = f"{ros_media['path']} frame={ros_media['frame_index']}"
        if ros_media.get("topic"):
            ros_source_label += f" topic={ros_media['topic']}"
        print(f"[collector] Loaded ROS media {ros_source_label} size={ros_media['image'].shape[1]}x{ros_media['image'].shape[0]}")

        point_pairs = TimestampAligner.collect_affine_pairs_interactive(
            aria_media["image"],
            ros_media["image"],
        )
        if not point_pairs:
            print("[collector] Cancelled without saving any affine point pairs.")
            return

        transform = ImageCoordinateTransform.fit_affine(point_pairs)
        pair_flags = TimestampAligner.format_affine_cli_pairs(point_pairs)
        command = TimestampAligner.build_affine_mapping_command(
            point_pairs,
            aria_csv=args.aria_csv,
            output_csv=args.output_csv,
            ros_width=ros_media["image"].shape[1],
            ros_height=ros_media["image"].shape[0],
            dots=args.dots,
        )
        payload = {
            "version": 1,
            "aria_media": str(aria_media["path"]),
            "aria_frame_index": int(aria_media["frame_index"]),
            "aria_size": [int(aria_media["image"].shape[1]), int(aria_media["image"].shape[0])],
            "ros_media": str(ros_media["path"]),
            "ros_frame_index": int(ros_media["frame_index"]),
            "ros_size": [int(ros_media["image"].shape[1]), int(ros_media["image"].shape[0])],
            "ros_image_topic": ros_media.get("topic"),
            "ros_timestamp_ns": ros_media.get("timestamp_ns"),
            "pair_count": len(point_pairs),
            "point_pairs": [
                {
                    "aria": [float(aria_point[0]), float(aria_point[1])],
                    "ros": [float(ros_point[0]), float(ros_point[1])],
                }
                for aria_point, ros_point in point_pairs
            ],
            "transform": transform.to_json(),
            "affine_cli_args": pair_flags,
            "mapping_command": command,
        }
        if args.pairs_json is not None:
            TimestampAligner.save_affine_collection(args.pairs_json, payload)

        print(f"[collector] Fitted affine transform: {transform.description}")
        print("[collector] Affine CLI args:")
        print(" ".join(shlex.quote(part) for part in pair_flags))
        print("[collector] Suggested mapping command:")
        print(command)

        if args.run_after_collect:
            if args.aria_csv is None or args.output_csv is None:
                raise SystemExit("--run-after-collect requires both --aria-csv and --output-csv")
            raw_affine_pairs = [
                [float(aria_point[0]), float(aria_point[1]), float(ros_point[0]), float(ros_point[1])]
                for aria_point, ros_point in point_pairs
            ]
            mapped = TimestampAligner.map_aria_motion_to_ros_camera(
                aria_csv=args.aria_csv,
                ros_width=ros_media["image"].shape[1],
                ros_height=ros_media["image"].shape[0],
                dots=args.dots,
                transform_mode="affine",
                affine_pairs=raw_affine_pairs,
            )
            args.output_csv.parent.mkdir(parents=True, exist_ok=True)
            mapped.to_csv(args.output_csv, index=False)
            print(f"[collector] Saved affine-mapped CSV to {args.output_csv}")

    @staticmethod
    def load_rgb_overlay_manifest(
        aria_csv: Path,
        aria_video: Path | None = None,
        manifest_path: Path | None = None,
    ) -> pd.DataFrame:
        """Load the per-written-RGB-frame manifest produced by the recorder.

        Search order:
          1. ``manifest_path`` if provided.
          2. ``rgb_overlay_frames.csv`` next to ``aria_csv``.
          3. ``rgb_overlay_frames.csv`` next to ``aria_video``.
          4. Fallback: synthesize a best-effort manifest from ``aria_csv`` by
             taking the first row per unique ``rgb_timestamp_ns``.

        Returned DataFrame has at least these columns:
          - ``video_frame_index`` (int)
          - ``wall_time_s`` (float)
          - ``rgb_timestamp_ns`` (int)
          - ``aria_rel_s`` (float, ``wall_time_s`` minus first row's ``wall_time_s``)
        """
        aria_csv = Path(aria_csv)
        search_paths: list[Path] = []
        if manifest_path is not None:
            search_paths.append(Path(manifest_path))
        search_paths.append(aria_csv.parent / "rgb_overlay_frames.csv")
        if aria_video is not None:
            search_paths.append(Path(aria_video).parent / "rgb_overlay_frames.csv")

        chosen = None
        for candidate in search_paths:
            if candidate.is_file():
                chosen = candidate
                break

        if chosen is not None:
            df = pd.read_csv(chosen)
            if "video_frame_index" not in df.columns:
                df = df.reset_index(drop=True)
                df["video_frame_index"] = np.arange(len(df), dtype=np.int64)
            df = df.sort_values("video_frame_index").reset_index(drop=True)
            if "wall_time_s" not in df.columns:
                raise ValueError(
                    f"Manifest {chosen} is missing required column 'wall_time_s'."
                )
            first_wall = float(df["wall_time_s"].iloc[0])
            df["aria_rel_s"] = df["wall_time_s"].astype(float) - first_wall
            print(f"[manifest] Using RGB frame manifest: {chosen} ({len(df)} frames)")
            return df

        # Fallback: synthesize from gaze_stream.csv. Each unique rgb_timestamp_ns
        # marked a new RGB frame in the legacy recorder, so take the first row
        # per unique value. This is best-effort and may under- or over-count
        # slightly if RGB timestamps repeat. Prefer the explicit manifest for
        # new recordings.
        print(
            f"[manifest] WARNING: rgb_overlay_frames.csv not found near {aria_csv}; "
            "synthesizing a best-effort manifest from gaze_stream.csv. "
            "Re-record with the updated aria_eye_viewer.py for frame-accurate pairing."
        )
        gaze = pd.read_csv(aria_csv)
        if "rgb_timestamp_ns" not in gaze.columns or "wall_time_s" not in gaze.columns:
            raise ValueError(
                f"Cannot synthesize manifest: {aria_csv} lacks rgb_timestamp_ns or wall_time_s."
            )
        gaze = gaze.dropna(subset=["rgb_timestamp_ns"]).reset_index(drop=True)
        first_rows = gaze.drop_duplicates(subset=["rgb_timestamp_ns"], keep="first").reset_index(drop=True)
        manifest = pd.DataFrame(
            {
                "video_frame_index": np.arange(len(first_rows), dtype=np.int64),
                "wall_time_s": first_rows["wall_time_s"].astype(float),
                "rgb_timestamp_ns": first_rows["rgb_timestamp_ns"].astype("int64", errors="ignore"),
            }
        )
        if len(manifest) == 0:
            raise ValueError(f"No usable RGB rows in {aria_csv}")
        for col in ("model_dot_x", "model_dot_y", "head_dot_x", "head_dot_y"):
            if col in first_rows.columns:
                manifest[col] = first_rows[col].values
        first_wall = float(manifest["wall_time_s"].iloc[0])
        manifest["aria_rel_s"] = manifest["wall_time_s"].astype(float) - first_wall
        return manifest

    @staticmethod
    def pair_ros_frames_to_aria_frames(
        aria_rel_s,
        ros_rel_s,
        max_tol_s: float = 0.5,
    ):
        """For each ROS elapsed time, pick the nearest Aria frame elapsed time.

        Parameters
        ----------
        aria_rel_s : array-like of float, non-decreasing
            Aria video frames' elapsed times relative to the first Aria frame.
        ros_rel_s : array-like of float
            ROS frames' elapsed times relative to the first ROS frame.
        max_tol_s : float
            Maximum absolute difference between paired times before the pair
            is declared invalid (out of the shared window).

        Returns
        -------
        (aria_idx, valid) : tuple of numpy arrays, each length == len(ros_rel_s)
            ``aria_idx[k]`` is the index into ``aria_rel_s`` chosen for ROS
            frame ``k``. ``valid[k]`` is False when the nearest-neighbor delta
            exceeds ``max_tol_s`` (typical at start/end when one stream is
            longer than the other).
        """
        aria_rel_s = np.asarray(aria_rel_s, dtype=np.float64)
        ros_rel_s = np.asarray(ros_rel_s, dtype=np.float64)
        n_aria = aria_rel_s.size
        n_ros = ros_rel_s.size

        if n_aria == 0 or n_ros == 0:
            return np.zeros(n_ros, dtype=np.int64), np.zeros(n_ros, dtype=bool)

        # Insertion points. For each ROS elapsed, 'right' picks the first aria
        # index strictly greater; we then compare to the one just before.
        right_idx = np.searchsorted(aria_rel_s, ros_rel_s, side="left")
        right_idx = np.clip(right_idx, 0, n_aria - 1)
        left_idx = np.clip(right_idx - 1, 0, n_aria - 1)

        d_right = np.abs(aria_rel_s[right_idx] - ros_rel_s)
        d_left = np.abs(aria_rel_s[left_idx] - ros_rel_s)
        pick_left = d_left <= d_right
        aria_idx = np.where(pick_left, left_idx, right_idx)
        delta = np.where(pick_left, d_left, d_right)
        valid = delta <= float(max_tol_s)
        return aria_idx.astype(np.int64), valid

    @staticmethod
    def _pair_rows_for_times(mapped_rel_s: np.ndarray, target_rel_s: float) -> int:
        """Nearest-row lookup into mapped_df by relative time."""
        if mapped_rel_s.size == 0:
            return 0
        idx = int(np.searchsorted(mapped_rel_s, target_rel_s, side="left"))
        if idx <= 0:
            return 0
        if idx >= mapped_rel_s.size:
            return int(mapped_rel_s.size - 1)
        prev_dt = abs(target_rel_s - mapped_rel_s[idx - 1])
        next_dt = abs(mapped_rel_s[idx] - target_rel_s)
        return int(idx - 1 if prev_dt <= next_dt else idx)

    @staticmethod
    def _decode_aria_frame(cap, current_frame_idx: int, cached_frame, target_idx: int):
        """Sequentially decode aria video to target frame index, with rare backward seek."""
        import cv2

        if target_idx == current_frame_idx and cached_frame is not None:
            return current_frame_idx, cached_frame
        if target_idx < current_frame_idx:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(target_idx))
            current_frame_idx = int(target_idx) - 1
            cached_frame = None
        while current_frame_idx < target_idx:
            ok, frame = cap.read()
            if not ok or frame is None:
                return current_frame_idx, cached_frame
            current_frame_idx += 1
            cached_frame = frame
        return current_frame_idx, cached_frame

    @staticmethod
    def _match_local_features_near_gaze(
        aria_bgr: np.ndarray,
        ros_bgr: np.ndarray,
        aria_center: tuple[float, float],
        ros_center: tuple[float, float],
        patch_radius_px: float = 180.0,
        min_matches: int = 14,
        min_inliers: int = 8,
        max_reproj_px: float = 4.0,
    ):
        """Local ORB matching around gaze neighborhoods."""
        import cv2

        if aria_bgr is None or ros_bgr is None:
            return None
        if not (np.all(np.isfinite(aria_center)) and np.all(np.isfinite(ros_center))):
            return None

        aria_h, aria_w = aria_bgr.shape[:2]
        ros_h, ros_w = ros_bgr.shape[:2]
        ax, ay = float(aria_center[0]), float(aria_center[1])
        rx, ry = float(ros_center[0]), float(ros_center[1])
        if not (0 <= ax < aria_w and 0 <= ay < aria_h and 0 <= rx < ros_w and 0 <= ry < ros_h):
            return None

        patch_radius_px = float(max(30.0, patch_radius_px))
        gray_a = cv2.cvtColor(aria_bgr, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(ros_bgr, cv2.COLOR_BGR2GRAY)

        def _run_match(mask_a, mask_r, needed_matches: int, needed_inliers: int, reproj_limit: float, mode: str):
            orb = cv2.ORB_create(nfeatures=1400, fastThreshold=10)
            kp_a, des_a = orb.detectAndCompute(gray_a, mask_a)
            kp_r, des_r = orb.detectAndCompute(gray_r, mask_r)
            if des_a is None or des_r is None or len(kp_a) < 8 or len(kp_r) < 8:
                return None

            matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
            knn = matcher.knnMatch(des_a, des_r, k=2)
            good = []
            for pair in knn:
                if len(pair) < 2:
                    continue
                m, n = pair[0], pair[1]
                if m.distance < 0.75 * n.distance:
                    good.append(m)
            if len(good) < int(needed_matches):
                return None

            pts_a = np.float32([kp_a[m.queryIdx].pt for m in good])
            pts_r = np.float32([kp_r[m.trainIdx].pt for m in good])

            affine, inliers = cv2.estimateAffine2D(
                pts_a,
                pts_r,
                method=cv2.RANSAC,
                ransacReprojThreshold=float(reproj_limit),
                maxIters=5000,
                confidence=0.995,
                refineIters=30,
            )
            if affine is None or inliers is None:
                return None
            inliers = inliers.ravel().astype(bool)
            if int(inliers.sum()) < int(needed_inliers):
                return None

            in_a = pts_a[inliers]
            in_r = pts_r[inliers]
            pred_r = np.hstack([in_a, np.ones((in_a.shape[0], 1), dtype=np.float32)]) @ affine.T
            err = np.linalg.norm(pred_r - in_r, axis=1)
            med_err = float(np.median(err))
            if not np.isfinite(med_err) or med_err > float(reproj_limit):
                return None

            gaze_pred = np.array([ax, ay, 1.0], dtype=np.float64) @ np.asarray(affine, dtype=np.float64).T
            if not np.all(np.isfinite(gaze_pred)):
                return None
            if not (0 <= float(gaze_pred[0]) < ros_w and 0 <= float(gaze_pred[1]) < ros_h):
                return None

            return {
                "aria_inliers": in_a.astype(np.float64),
                "ros_inliers": in_r.astype(np.float64),
                "median_reproj_px": med_err,
                "gaze_ros_target": (float(gaze_pred[0]), float(gaze_pred[1])),
                "match_mode": mode,
            }

        local_mask_a = np.zeros_like(gray_a, dtype=np.uint8)
        local_mask_r = np.zeros_like(gray_r, dtype=np.uint8)
        cv2.circle(local_mask_a, (int(round(ax)), int(round(ay))), int(round(patch_radius_px)), 255, thickness=-1)
        cv2.circle(local_mask_r, (int(round(rx)), int(round(ry))), int(round(patch_radius_px)), 255, thickness=-1)
        local = _run_match(
            local_mask_a,
            local_mask_r,
            needed_matches=int(min_matches),
            needed_inliers=int(min_inliers),
            reproj_limit=float(max_reproj_px),
            mode="local",
        )
        return local

    @staticmethod
    def _collect_auto_affine_samples(
        aria_video: Path,
        aria_manifest_df: pd.DataFrame,
        mapped_df: pd.DataFrame,
        rosbag_dir: Path,
        ros_image_topic: str | None = None,
        mapped_time_offset_s: float = 0.0,
        aria_time_column: str = "wall_time_s",
        max_tol_s: float = 0.5,
        sample_step: int = 5,
        max_frames: int = 0,
        patch_radius_px: float = 180.0,
        min_matches: int = 14,
        min_inliers: int = 8,
        max_reproj_px: float = 4.0,
        collect_pairs: bool = True,
    ):
        """Gather per-frame local correspondences near gaze and optional point pool."""
        import cv2

        if "aria_rel_s" not in aria_manifest_df.columns:
            raise ValueError("aria_manifest_df must include aria_rel_s.")
        aria_rel_s = aria_manifest_df["aria_rel_s"].to_numpy(dtype=np.float64)
        aria_video_frame_indices = aria_manifest_df["video_frame_index"].to_numpy(dtype=np.int64)
        first_aria_wall_s = float(aria_manifest_df["wall_time_s"].iloc[0])

        if aria_time_column in mapped_df.columns:
            mapped_times = mapped_df[aria_time_column].astype(float).to_numpy()
            mapped_rel_s = mapped_times - first_aria_wall_s
        else:
            mapped_rel_s = None

        cap = cv2.VideoCapture(str(aria_video))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open Aria video for auto-affine: {aria_video}")

        current_aria_idx = -1
        cached_aria_frame = None
        rows = []
        point_records = []
        used = 0
        first_ros_stamp = None

        try:
            for ros_frame in TimestampAligner.iterate_rosbag_image_frames(rosbag_dir, image_topic=ros_image_topic):
                rf_idx = int(ros_frame["frame_index"])
                if sample_step > 1 and (rf_idx % int(sample_step)) != 0:
                    continue
                if max_frames and used >= int(max_frames):
                    break

                stamp_s = float(ros_frame["stamp_s"])
                if first_ros_stamp is None:
                    first_ros_stamp = stamp_s
                ros_rel_s = stamp_s - first_ros_stamp

                aria_idx_arr, valid_arr = TimestampAligner.pair_ros_frames_to_aria_frames(
                    aria_rel_s, np.asarray([ros_rel_s], dtype=np.float64), max_tol_s=max_tol_s
                )
                aria_local_idx = int(aria_idx_arr[0])
                if not bool(valid_arr[0]):
                    continue
                aria_t = float(aria_rel_s[aria_local_idx])
                aria_video_idx = int(aria_video_frame_indices[aria_local_idx])
                if mapped_rel_s is not None and mapped_rel_s.size > 0:
                    mapped_row_idx = TimestampAligner._pair_rows_for_times(
                        mapped_rel_s, aria_t + float(mapped_time_offset_s)
                    )
                else:
                    mapped_row_idx = min(aria_local_idx, len(mapped_df) - 1)
                row = mapped_df.iloc[mapped_row_idx]

                aria_x = float(row.get("model_dot_x", np.nan))
                aria_y = float(row.get("model_dot_y", np.nan))
                ros_x = float(row.get("eye_ros_x", np.nan))
                ros_y = float(row.get("eye_ros_y", np.nan))
                if not (np.isfinite(aria_x) and np.isfinite(aria_y) and np.isfinite(ros_x) and np.isfinite(ros_y)):
                    continue

                current_aria_idx, cached_aria_frame = TimestampAligner._decode_aria_frame(
                    cap, current_aria_idx, cached_aria_frame, aria_video_idx
                )
                if cached_aria_frame is None:
                    continue

                match = TimestampAligner._match_local_features_near_gaze(
                    aria_bgr=cached_aria_frame,
                    ros_bgr=ros_frame["image"],
                    aria_center=(aria_x, aria_y),
                    ros_center=(ros_x, ros_y),
                    patch_radius_px=patch_radius_px,
                    min_matches=min_matches,
                    min_inliers=min_inliers,
                    max_reproj_px=max_reproj_px,
                )
                if match is None:
                    continue

                target_x, target_y = match["gaze_ros_target"]
                rows.append(
                    {
                        "ros_frame": rf_idx,
                        "aria_video_frame": aria_video_idx,
                        "mapped_row": int(mapped_row_idx),
                        "ros_rel_s": float(ros_rel_s),
                        "aria_rel_s": float(aria_t),
                        "aria_x": aria_x,
                        "aria_y": aria_y,
                        "ros_target_x": float(target_x),
                        "ros_target_y": float(target_y),
                        "median_reproj_px": float(match["median_reproj_px"]),
                        "inlier_count": int(match["aria_inliers"].shape[0]),
                        "match_mode": "local",
                    }
                )

                if collect_pairs:
                    sigma = max(25.0, float(patch_radius_px) * 0.45)
                    in_a = match["aria_inliers"]
                    in_r = match["ros_inliers"]
                    for k in range(in_a.shape[0]):
                        da = float(np.linalg.norm(in_a[k] - np.array([aria_x, aria_y], dtype=np.float64)))
                        dr = float(np.linalg.norm(in_r[k] - np.array([ros_x, ros_y], dtype=np.float64)))
                        wa = np.exp(-0.5 * (da / sigma) ** 2)
                        wr = np.exp(-0.5 * (dr / sigma) ** 2)
                        w = float(max(1e-4, wa * wr))
                        point_records.append(
                            {
                                "ros_frame": rf_idx,
                                "aria_x": float(in_a[k, 0]),
                                "aria_y": float(in_a[k, 1]),
                                "ros_x": float(in_r[k, 0]),
                                "ros_y": float(in_r[k, 1]),
                                "weight": w,
                            }
                        )
                used += 1
        finally:
            cap.release()

        frame_df = pd.DataFrame(rows)
        points_df = pd.DataFrame(point_records)
        return frame_df, points_df

    @staticmethod
    def _estimate_best_time_shift_auto_affine(
        aria_video: Path,
        aria_manifest_df: pd.DataFrame,
        mapped_df_anchor: pd.DataFrame,
        rosbag_dir: Path,
        ros_image_topic: str | None = None,
        aria_time_column: str = "wall_time_s",
        max_tol_s: float = 0.5,
        search_range_s: float = 0.12,
        search_step_s: float = 0.015,
        sample_step: int = 8,
        patch_radius_px: float = 180.0,
        min_matches: int = 14,
        min_inliers: int = 8,
        max_reproj_px: float = 4.0,
        initial_offset_s: float = 0.0,
        holdout_mod: int = 5,
    ):
        candidates = np.arange(-float(search_range_s), float(search_range_s) + 0.5 * float(search_step_s), float(search_step_s))
        if np.isfinite(float(initial_offset_s)):
            candidates = np.unique(np.concatenate([candidates, np.asarray([float(initial_offset_s)], dtype=np.float64)]))
        best = None
        diagnostics = []

        for delta_s in candidates:
            frame_df, _ = TimestampAligner._collect_auto_affine_samples(
                aria_video=aria_video,
                aria_manifest_df=aria_manifest_df,
                mapped_df=mapped_df_anchor,
                rosbag_dir=rosbag_dir,
                ros_image_topic=ros_image_topic,
                mapped_time_offset_s=float(delta_s),
                aria_time_column=aria_time_column,
                max_tol_s=max_tol_s,
                sample_step=sample_step,
                max_frames=0,
                patch_radius_px=patch_radius_px,
                min_matches=min_matches,
                min_inliers=min_inliers,
                max_reproj_px=max_reproj_px,
                collect_pairs=False,
            )
            corr = np.nan
            n = int(len(frame_df))
            med_err = np.nan
            local_ratio = np.nan
            if "match_mode" in frame_df.columns and n > 0:
                local_ratio = float(np.mean(frame_df["match_mode"].astype(str).to_numpy() == "local"))
            if n >= 8:
                frame_df = frame_df.sort_values("ros_frame").reset_index(drop=True)
                dx_aria = np.diff(frame_df["aria_x"].to_numpy(dtype=np.float64))
                dx_ros = np.diff(frame_df["ros_target_x"].to_numpy(dtype=np.float64))
                if dx_aria.size >= 5 and np.nanstd(dx_aria) > 1e-6 and np.nanstd(dx_ros) > 1e-6:
                    corr = float(np.corrcoef(dx_aria, dx_ros)[0, 1])
                med_err = float(np.nanmedian(frame_df["median_reproj_px"].to_numpy(dtype=np.float64)))

            diagnostics.append(
                {
                    "offset_s": float(delta_s),
                    "sample_count": n,
                    "corr_dx": corr,
                    "median_local_reproj_px": med_err,
                    "local_match_ratio": local_ratio,
                    "holdout_median_error_px": np.nan,
                    "holdout_mean_error_px": np.nan,
                    "fit_valid": False,
                }
            )

            if n < 6:
                continue
            try:
                fit = TimestampAligner._fit_plain_affine_from_frame_samples(
                    frame_df=frame_df,
                    holdout_mod=int(holdout_mod),
                    max_reproj_px=float(max_reproj_px),
                )
            except Exception:
                continue

            hold_med = float(fit.get("median_holdout_error_px", np.nan))
            hold_mean = float(fit.get("mean_holdout_error_px", np.nan))
            diagnostics[-1]["holdout_median_error_px"] = hold_med
            diagnostics[-1]["holdout_mean_error_px"] = hold_mean
            diagnostics[-1]["fit_valid"] = bool(np.isfinite(hold_med))
            if not np.isfinite(hold_med):
                continue

            # Choose temporal shift by minimizing Euclidean holdout error.
            key = (
                hold_med,
                hold_mean if np.isfinite(hold_mean) else 1e9,
                -n,
            )
            if best is None or key < best["key"]:
                best = {
                    "offset_s": float(delta_s),
                    "corr_dx": float(corr) if np.isfinite(corr) else np.nan,
                    "sample_count": n,
                    "median_local_reproj_px": float(med_err) if np.isfinite(med_err) else np.nan,
                    "local_match_ratio": local_ratio,
                    "holdout_median_error_px": hold_med,
                    "holdout_mean_error_px": hold_mean,
                    "key": key,
                }

        if best is None:
            fallback = float(initial_offset_s)
            print(
                "[auto-affine] WARNING: Could not estimate temporal shift from feature matches; "
                f"falling back to provided offset {fallback:+.3f}s"
            )
            return fallback, diagnostics
        print(
            "[auto-affine] best temporal offset "
            f"{best['offset_s']:+.3f}s "
            f"(corr={best['corr_dx'] if np.isfinite(best['corr_dx']) else float('nan'):.3f}, "
            f"samples={best['sample_count']}, local_ratio={best.get('local_match_ratio', float('nan')):.2f}, "
            f"holdout_median_err={best.get('holdout_median_error_px', float('nan')):.2f}px)"
        )
        return float(best["offset_s"]), diagnostics

    @staticmethod
    def _lead_compensate_xy(x: np.ndarray, y: np.ndarray, tau: float) -> tuple[np.ndarray, np.ndarray]:
        """Legacy helper kept for compatibility; no lead compensation is applied."""
        return np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64)

    @staticmethod
    def _motion_state_from_dx(dx: float, threshold_px: float) -> str:
        if not np.isfinite(dx):
            return "neutral"
        if dx >= float(threshold_px):
            return "forward"
        if dx <= -float(threshold_px):
            return "backward"
        return "neutral"

    @staticmethod
    def _fit_piecewise_motion_model(
        aria_xy: np.ndarray,
        ros_xy: np.ndarray,
        weights: np.ndarray,
        dx: np.ndarray,
        min_state_samples: int = 5,
        base_threshold_px: float = 6.0,
    ):
        """Deprecated: piecewise motion model is disabled."""
        return None

    @staticmethod
    def _fit_motion_state_bias_model(
        residual_xy: np.ndarray,
        dx: np.ndarray,
        weights: np.ndarray,
        min_state_samples: int = 5,
        base_threshold_px: float = 6.0,
    ):
        """Deprecated: motion-state residual bias model is disabled."""
        return None

    @staticmethod
    def _fit_lead_compensated_affine_from_frame_samples(
        frame_df: pd.DataFrame,
        holdout_mod: int = 5,
        max_reproj_px: float = 4.0,
        tau_min: float = -1.0,
        tau_max: float = 1.0,
        tau_step: float = 0.05,
    ):
        """Deprecated: lead/velocity affine fit is disabled; uses plain affine fit."""
        return TimestampAligner._fit_plain_affine_from_frame_samples(
            frame_df=frame_df,
            holdout_mod=holdout_mod,
            max_reproj_px=max_reproj_px,
        )

    @staticmethod
    def _fit_plain_affine_from_frame_samples(
        frame_df: pd.DataFrame,
        holdout_mod: int = 5,
        max_reproj_px: float = 4.0,
    ):
        """Fallback affine fit without lead/velocity compensation."""
        import cv2

        if frame_df is None or frame_df.empty:
            raise RuntimeError("Fallback affine failed: empty frame samples.")
        work = frame_df.sort_values("ros_frame").reset_index(drop=True).copy()
        required = ["ros_frame", "aria_x", "aria_y", "ros_target_x", "ros_target_y"]
        for col in required:
            if col not in work.columns:
                raise RuntimeError(f"Fallback affine failed: missing column {col}.")

        valid = np.isfinite(work["aria_x"]) & np.isfinite(work["aria_y"]) & np.isfinite(work["ros_target_x"]) & np.isfinite(work["ros_target_y"])
        work = work.loc[valid].reset_index(drop=True)
        if len(work) < 5:
            raise RuntimeError("Fallback affine failed: not enough valid samples.")

        holdout_mod = max(2, int(holdout_mod))
        holdout_mask = (work["ros_frame"].astype(int).to_numpy() % holdout_mod) == 0
        train_mask = ~holdout_mask
        if int(np.sum(train_mask)) < 3:
            holdout_mask = (np.arange(len(work)) % holdout_mod) == 0
            train_mask = ~holdout_mask
        if int(np.sum(train_mask)) < 3:
            train_mask = np.ones(len(work), dtype=bool)
            holdout_mask = np.zeros(len(work), dtype=bool)

        train = work.loc[train_mask].reset_index(drop=True)
        a = train[["aria_x", "aria_y"]].to_numpy(dtype=np.float64)
        b = train[["ros_target_x", "ros_target_y"]].to_numpy(dtype=np.float64)

        affine_ransac, inliers = cv2.estimateAffine2D(
            a.astype(np.float32),
            b.astype(np.float32),
            method=cv2.RANSAC,
            ransacReprojThreshold=float(max_reproj_px),
            maxIters=4000,
            confidence=0.995,
            refineIters=20,
        )
        if affine_ransac is not None and inliers is not None:
            inlier_mask = inliers.ravel().astype(bool)
            if int(np.sum(inlier_mask)) >= 3:
                train = train.iloc[np.where(inlier_mask)[0]].reset_index(drop=True)

        if len(train) < 3:
            raise RuntimeError("Fallback affine failed: too few train inliers.")

        weight = np.ones(len(train), dtype=np.float64)
        if "inlier_count" in train.columns:
            weight = np.maximum(1.0, train["inlier_count"].astype(float).to_numpy(dtype=np.float64))
        if "median_reproj_px" in train.columns:
            weight = weight / (1.0 + np.maximum(0.0, train["median_reproj_px"].astype(float).to_numpy(dtype=np.float64)))

        point_pairs = [
            ((float(r["aria_x"]), float(r["aria_y"])), (float(r["ros_target_x"]), float(r["ros_target_y"])))
            for _idx, r in train.iterrows()
        ]
        transform = ImageCoordinateTransform.fit_weighted_affine(
            point_pairs,
            weights=[float(w) for w in weight],
            description_prefix="auto-affine-fallback",
        )

        hold = work.loc[holdout_mask].copy()
        if hold.empty:
            hold = work.copy()
        holdout_rows = []
        errors = []
        for _idx, row in hold.iterrows():
            pred = transform.apply((float(row["aria_x"]), float(row["aria_y"])))
            if pred is None:
                continue
            tx = float(row["ros_target_x"])
            ty = float(row["ros_target_y"])
            err = float(np.hypot(float(pred[0]) - tx, float(pred[1]) - ty))
            errors.append(err)
            holdout_rows.append(
                {
                    "ros_frame": int(row["ros_frame"]),
                    "aria_x": float(row["aria_x"]),
                    "aria_y": float(row["aria_y"]),
                    "aria_comp_x": float(row["aria_x"]),
                    "aria_comp_y": float(row["aria_y"]),
                    "target_ros_x": tx,
                    "target_ros_y": ty,
                    "pred_ros_x": float(pred[0]),
                    "pred_ros_y": float(pred[1]),
                    "aria_dx": np.nan,
                    "aria_dy": np.nan,
                    "vel_corr_x": 0.0,
                    "vel_corr_y": 0.0,
                    "error_px": err,
                }
            )

        direction_agreement = np.nan
        if len(holdout_rows) >= 3:
            eval_df = pd.DataFrame(holdout_rows).sort_values("ros_frame").reset_index(drop=True)
            dx_t = np.diff(eval_df["target_ros_x"].to_numpy(dtype=np.float64))
            dx_p = np.diff(eval_df["pred_ros_x"].to_numpy(dtype=np.float64))
            valid_dir = (np.abs(dx_t) >= 2.0) | (np.abs(dx_p) >= 2.0)
            if np.any(valid_dir):
                direction_agreement = float(np.mean(np.sign(dx_t[valid_dir]) == np.sign(dx_p[valid_dir])))

        train_df = pd.DataFrame(
            {
                "aria_x": train["aria_x"].astype(float).to_numpy(),
                "aria_y": train["aria_y"].astype(float).to_numpy(),
                "ros_x": train["ros_target_x"].astype(float).to_numpy(),
                "ros_y": train["ros_target_y"].astype(float).to_numpy(),
                "weight": weight[: len(train)],
            }
        )
        return {
            "transform": transform,
            "holdout_eval_df": pd.DataFrame(holdout_rows),
            "median_holdout_error_px": float(np.median(errors)) if errors else np.nan,
            "mean_holdout_error_px": float(np.mean(errors)) if errors else np.nan,
            "direction_agreement": direction_agreement,
            "final_fit_df": train_df,
        }

    @staticmethod
    def auto_fit_affine_from_gaze_local_features(
        aria_video: Path,
        aria_manifest_df: pd.DataFrame,
        mapped_df_anchor: pd.DataFrame,
        rosbag_dir: Path,
        ros_image_topic: str | None = None,
        aria_time_column: str = "wall_time_s",
        max_tol_s: float = 0.5,
        search_range_s: float = 0.12,
        search_step_s: float = 0.015,
        temporal_sample_step: int = 8,
        fit_sample_step: int = 4,
        patch_radius_px: float = 180.0,
        min_matches: int = 14,
        min_inliers: int = 8,
        max_reproj_px: float = 4.0,
        holdout_mod: int = 5,
        initial_time_offset_s: float = 0.0,
    ):
        best_offset_s, temporal_diagnostics = TimestampAligner._estimate_best_time_shift_auto_affine(
            aria_video=aria_video,
            aria_manifest_df=aria_manifest_df,
            mapped_df_anchor=mapped_df_anchor,
            rosbag_dir=rosbag_dir,
            ros_image_topic=ros_image_topic,
            aria_time_column=aria_time_column,
            max_tol_s=max_tol_s,
            search_range_s=search_range_s,
            search_step_s=search_step_s,
            sample_step=temporal_sample_step,
            patch_radius_px=patch_radius_px,
            min_matches=min_matches,
            min_inliers=min_inliers,
            max_reproj_px=max_reproj_px,
            initial_offset_s=initial_time_offset_s,
            holdout_mod=holdout_mod,
        )

        frame_df, points_df = TimestampAligner._collect_auto_affine_samples(
            aria_video=aria_video,
            aria_manifest_df=aria_manifest_df,
            mapped_df=mapped_df_anchor,
            rosbag_dir=rosbag_dir,
            ros_image_topic=ros_image_topic,
            mapped_time_offset_s=best_offset_s,
            aria_time_column=aria_time_column,
            max_tol_s=max_tol_s,
            sample_step=fit_sample_step,
            max_frames=0,
            patch_radius_px=patch_radius_px,
            min_matches=min_matches,
            min_inliers=min_inliers,
            max_reproj_px=max_reproj_px,
            collect_pairs=True,
        )
        if frame_df.empty or len(frame_df) < 8:
            raise RuntimeError(
                "Auto affine failed: not enough frame-level gaze neighborhood matches. "
                "Try increasing patch radius, lowering min-inliers, or using manual affine pairs."
            )

        fit = TimestampAligner._fit_plain_affine_from_frame_samples(
            frame_df=frame_df,
            holdout_mod=int(holdout_mod),
            max_reproj_px=float(max_reproj_px),
        )

        transform = fit["transform"]
        holdout_rows_df = fit["holdout_eval_df"]
        median_holdout_error = fit["median_holdout_error_px"]
        mean_holdout_error = fit["mean_holdout_error_px"]
        direction_agreement = fit["direction_agreement"]
        print(
            "[auto-affine] holdout validation: "
            f"median_err={median_holdout_error:.2f}px mean_err={mean_holdout_error:.2f}px "
            f"direction_agreement={direction_agreement if np.isfinite(direction_agreement) else float('nan'):.3f}"
        )

        train_df = fit["final_fit_df"].copy()
        affine_pairs = [[float(r["aria_x"]), float(r["aria_y"]), float(r["ros_x"]), float(r["ros_y"])] for _idx, r in train_df.iterrows()]
        return {
            "best_time_offset_s": float(best_offset_s),
            "temporal_diagnostics": temporal_diagnostics,
            "frame_samples_df": frame_df,
            "point_records_df": points_df,
            "train_points_df": train_df,
            "holdout_eval_df": holdout_rows_df,
            "affine_pairs": affine_pairs,
            "transform": transform,
            "median_holdout_error_px": median_holdout_error,
            "mean_holdout_error_px": mean_holdout_error,
            "direction_agreement": direction_agreement,
        }

    @staticmethod
    def render_synced_side_by_side(
        aria_video: Path,
        aria_manifest_df: pd.DataFrame,
        mapped_df: pd.DataFrame,
        rosbag_dir: Path,
        output_rosbag_overlay: Path,
        output_side_by_side: Path,
        ros_image_topic: str | None = None,
        dots=("eye", "head"),
        transform_mode: str = "anchor",
        trail_length: int = 40,
        debug_overlay: bool = True,
        debug_print_every: int = 50,
        max_frames: int = 0,
        max_tol_s: float = 0.5,
        aria_time_column: str = "wall_time_s",
        mapped_time_offset_s: float = 0.0,
        debug_eye_x: bool = False,
        side_by_side_height: int | None = None,
        sync_basis: str = "relative",
    ):
        """Produce the ROS overlay and side-by-side videos in one timestamp-paired pass.

        For each ROS frame we:
          1. compute ``ros_rel_s`` against the first ROS frame,
          2. pick the nearest Aria video frame via ``aria_manifest_df.aria_rel_s``,
          3. pick the nearest mapped_df row for drawing dots,
          4. decode that exact Aria frame from ``aria_video``,
          5. write the ROS-overlay frame and a horizontally stacked side-by-side frame.

        ROS frames outside the shared time window (``|delta| > max_tol_s``) are
        skipped, so the output duration is naturally clipped to where both
        streams have data. This directly fixes the "Aria ends while ROS keeps
        playing" mismatch.
        """
        import cv2

        aria_video = Path(aria_video)
        output_rosbag_overlay = Path(output_rosbag_overlay)
        output_side_by_side = Path(output_side_by_side)
        output_rosbag_overlay.parent.mkdir(parents=True, exist_ok=True)
        output_side_by_side.parent.mkdir(parents=True, exist_ok=True)

        if "aria_rel_s" not in aria_manifest_df.columns:
            raise ValueError("aria_manifest_df must have an 'aria_rel_s' column")
        aria_rel_s = aria_manifest_df["aria_rel_s"].to_numpy(dtype=np.float64)
        aria_wall_s = aria_manifest_df["wall_time_s"].to_numpy(dtype=np.float64)
        aria_video_frame_indices = aria_manifest_df["video_frame_index"].to_numpy(dtype=np.int64)
        first_aria_wall_s = float(aria_manifest_df["wall_time_s"].iloc[0])

        if aria_time_column in mapped_df.columns:
            mapped_times = mapped_df[aria_time_column].astype(float).to_numpy()
            # Share the origin with the manifest so both indexes use the same clock.
            mapped_rel_s = mapped_times - first_aria_wall_s
        else:
            print(
                f"[review] WARNING: '{aria_time_column}' missing from mapped_df; "
                "falling back to row-index pairing for dots."
            )
            mapped_rel_s = None

        cap = cv2.VideoCapture(str(aria_video))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open Aria video: {aria_video}")
        try:
            aria_frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            aria_h_full = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            aria_w_full = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)

            frame_iter = TimestampAligner.iterate_rosbag_image_frames(
                rosbag_dir, image_topic=ros_image_topic
            )
            try:
                first_ros = next(frame_iter)
            except StopIteration:
                raise SystemExit(f"No ROS image frames found in {rosbag_dir}")

            ros_h, ros_w = first_ros["image"].shape[:2]
            target_h = int(side_by_side_height or ros_h)

            # Aria tile height equals target_h, width scales by aspect ratio.
            if aria_h_full > 0:
                tile_scale = target_h / float(aria_h_full)
                aria_tile_w = max(2, int(round(aria_w_full * tile_scale)))
            else:
                aria_tile_w = max(2, ros_w)

            # Sample a handful of ROS stamps to estimate fps.
            sample_stamps = [first_ros["stamp_s"]]
            buffered = []
            for _ in range(8):
                try:
                    fr = next(frame_iter)
                except StopIteration:
                    break
                buffered.append(fr)
                sample_stamps.append(fr["stamp_s"])
            if len(sample_stamps) >= 2:
                diffs = np.diff(np.asarray(sample_stamps, dtype=np.float64))
                diffs = diffs[diffs > 0]
                ros_fps = float(1.0 / np.median(diffs)) if diffs.size else 10.0
            else:
                ros_fps = 10.0

            # ROS overlay writer (ROS resolution).
            ros_fourcc = cv2.VideoWriter_fourcc(*(
                "MJPG" if output_rosbag_overlay.suffix.lower() == ".avi" else "mp4v"
            ))
            ros_writer = cv2.VideoWriter(
                str(output_rosbag_overlay), ros_fourcc, ros_fps, (ros_w, ros_h)
            )
            if not ros_writer.isOpened():
                raise RuntimeError(f"Could not open ROS overlay writer: {output_rosbag_overlay}")

            # Side-by-side writer.
            sbs_fourcc = cv2.VideoWriter_fourcc(*(
                "mp4v" if output_side_by_side.suffix.lower() in (".mp4", ".m4v") else "MJPG"
            ))
            sbs_w = aria_tile_w + ros_w
            sbs_writer = cv2.VideoWriter(
                str(output_side_by_side), sbs_fourcc, ros_fps, (sbs_w, target_h)
            )
            if not sbs_writer.isOpened():
                raise RuntimeError(f"Could not open side-by-side writer: {output_side_by_side}")

            eye_trail: list[tuple[int, int]] = []
            head_trail: list[tuple[int, int]] = []
            current_aria_frame = -1
            cached_aria_bgr = None
            paired_count = 0
            skipped_start = 0
            skipped_end = 0

            def _advance_to(target_idx: int):
                nonlocal current_aria_frame, cached_aria_bgr
                if target_idx == current_aria_frame and cached_aria_bgr is not None:
                    return cached_aria_bgr
                if target_idx < current_aria_frame:
                    # Seek back (rare; only if ROS timestamps go backwards).
                    cap.set(cv2.CAP_PROP_POS_FRAMES, target_idx)
                    current_aria_frame = target_idx - 1
                while current_aria_frame < target_idx:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        return cached_aria_bgr
                    current_aria_frame += 1
                    cached_aria_bgr = frame
                return cached_aria_bgr

            def _process(ros_frame_info):
                nonlocal paired_count, skipped_start, skipped_end

                stamp_s = float(ros_frame_info["stamp_s"])
                if _process.first_ros_stamp_s is None:
                    _process.first_ros_stamp_s = stamp_s
                ros_rel_s = stamp_s - _process.first_ros_stamp_s

                if sync_basis == "epoch":
                    aria_idx_arr, valid_arr = TimestampAligner.pair_ros_frames_to_aria_frames(
                        aria_wall_s, np.asarray([stamp_s], dtype=np.float64), max_tol_s=max_tol_s
                    )
                else:
                    aria_idx_arr, valid_arr = TimestampAligner.pair_ros_frames_to_aria_frames(
                        aria_rel_s, np.asarray([ros_rel_s], dtype=np.float64), max_tol_s=max_tol_s
                    )
                aria_idx_local = int(aria_idx_arr[0])
                if not bool(valid_arr[0]):
                    compare_time = stamp_s if sync_basis == "epoch" else ros_rel_s
                    first_time = aria_wall_s[0] if sync_basis == "epoch" else aria_rel_s[0]
                    if compare_time < first_time:
                        skipped_start += 1
                    else:
                        skipped_end += 1
                    return True

                aria_video_idx = int(aria_video_frame_indices[aria_idx_local])

                aria_t = float(aria_rel_s[aria_idx_local])
                aria_wall_t = float(aria_wall_s[aria_idx_local])

                # Choose the mapped_df row for the dot.
                #
                # Important: pair the ROS dot row to the chosen Aria frame time,
                # not directly to ros_rel_s. This keeps the drawn ROS dot in phase
                # with the Aria frame shown on the left side-by-side tile.
                if mapped_rel_s is not None and mapped_rel_s.size > 0:
                    mapped_target_rel_s = (
                        (aria_wall_t - first_aria_wall_s)
                        if sync_basis == "epoch"
                        else aria_t
                    )
                    mapped_row_idx = TimestampAligner._pair_rows_for_times(
                        mapped_rel_s, mapped_target_rel_s + float(mapped_time_offset_s)
                    )
                else:
                    mapped_row_idx = min(paired_count, len(mapped_df) - 1)
                row = mapped_df.iloc[mapped_row_idx]

                ros_image = ros_frame_info["image"].copy()
                eye_dot = None
                if "eye" in dots and TimestampAligner.is_finite_dot(row, "eye_ros_x", "eye_ros_y"):
                    eye_dot = (float(row["eye_ros_x"]), float(row["eye_ros_y"]))
                    TimestampAligner.draw_dot(ros_image, eye_dot, (255, 0, 255), "eye")
                head_dot = None
                if "head" in dots and TimestampAligner.is_finite_dot(row, "head_ros_x", "head_ros_y"):
                    head_dot = (float(row["head_ros_x"]), float(row["head_ros_y"]))
                    TimestampAligner.draw_dot(ros_image, head_dot, (255, 255, 0), "head")

                if debug_overlay:
                    if eye_dot is not None:
                        eye_trail.append((int(round(eye_dot[0])), int(round(eye_dot[1]))))
                        if len(eye_trail) > max(1, int(trail_length)):
                            del eye_trail[: -int(trail_length)]
                    if head_dot is not None:
                        head_trail.append((int(round(head_dot[0])), int(round(head_dot[1]))))
                        if len(head_trail) > max(1, int(trail_length)):
                            del head_trail[: -int(trail_length)]
                    TimestampAligner.draw_trail(ros_image, eye_trail, (255, 0, 255))
                    TimestampAligner.draw_trail(ros_image, head_trail, (255, 255, 0))
                    delta_s = (
                        (stamp_s - aria_wall_t)
                        if sync_basis == "epoch"
                        else (ros_rel_s - aria_t)
                    )
                    delta_ms = delta_s * 1000.0
                    status_lines = [
                        f"synced {transform_mode}/{sync_basis} ros_frame={ros_frame_info['frame_index']} "
                        f"aria_vframe={aria_video_idx} dt={delta_ms:+.1f}ms",
                        f"ros_rel={ros_rel_s:.3f}s aria_rel={aria_t:.3f}s csv_row={mapped_row_idx}/{len(mapped_df) - 1}",
                        f"eye=({row.get('eye_ros_x', np.nan)}, {row.get('eye_ros_y', np.nan)}) "
                        f"head=({row.get('head_ros_x', np.nan)}, {row.get('head_ros_y', np.nan)})",
                        f"topic={ros_frame_info['topic']}",
                    ]
                    if debug_eye_x:
                        aria_eye_x = float(row.get("model_dot_x", np.nan))
                        ros_eye_x = float(row.get("eye_ros_x", np.nan))
                        eye_aria_anchor_x = float(row.get("eye_aria_anchor_x", np.nan))
                        eye_ros_anchor_x = float(row.get("eye_ros_anchor_x", np.nan))
                        sx = float(row.get("eye_sx", np.nan))
                        aria_dx = aria_eye_x - eye_aria_anchor_x if np.isfinite(aria_eye_x) and np.isfinite(eye_aria_anchor_x) else np.nan
                        ros_dx = ros_eye_x - eye_ros_anchor_x if np.isfinite(ros_eye_x) and np.isfinite(eye_ros_anchor_x) else np.nan
                        status_lines.append(
                            f"eye_x aria={aria_eye_x:.1f} ros={ros_eye_x:.1f} sx={sx:.4f} "
                            f"aria_dx={aria_dx:+.1f} ros_dx={ros_dx:+.1f}"
                        )
                    TimestampAligner.draw_status_panel(ros_image, status_lines)

                aria_bgr = _advance_to(aria_video_idx)
                if aria_bgr is None:
                    skipped_end += 1
                    return True

                if aria_bgr.shape[0] != target_h:
                    aria_tile = cv2.resize(aria_bgr, (aria_tile_w, target_h), interpolation=cv2.INTER_AREA)
                else:
                    aria_tile = aria_bgr if aria_bgr.shape[1] == aria_tile_w else cv2.resize(
                        aria_bgr, (aria_tile_w, target_h), interpolation=cv2.INTER_AREA
                    )
                if ros_image.shape[0] != target_h:
                    ros_tile = cv2.resize(ros_image, (ros_w, target_h), interpolation=cv2.INTER_AREA)
                else:
                    ros_tile = ros_image
                sbs = np.hstack([aria_tile, ros_tile])

                ros_writer.write(ros_image)
                sbs_writer.write(sbs)

                if debug_print_every and paired_count % max(1, int(debug_print_every)) == 0:
                    delta_s = (
                        (stamp_s - aria_wall_t)
                        if sync_basis == "epoch"
                        else (ros_rel_s - aria_rel_s[aria_idx_local])
                    )
                    print(
                        f"[sync] paired ros_frame={ros_frame_info['frame_index']} "
                        f"aria_vframe={aria_video_idx} dt_ms={delta_s * 1000.0:+.1f}"
                    )
                    if debug_eye_x:
                        aria_eye_x = float(row.get("model_dot_x", np.nan))
                        ros_eye_x = float(row.get("eye_ros_x", np.nan))
                        eye_aria_anchor_x = float(row.get("eye_aria_anchor_x", np.nan))
                        eye_ros_anchor_x = float(row.get("eye_ros_anchor_x", np.nan))
                        sx = float(row.get("eye_sx", np.nan))
                        aria_dx = aria_eye_x - eye_aria_anchor_x if np.isfinite(aria_eye_x) and np.isfinite(eye_aria_anchor_x) else np.nan
                        ros_dx = ros_eye_x - eye_ros_anchor_x if np.isfinite(ros_eye_x) and np.isfinite(eye_ros_anchor_x) else np.nan
                        print(
                            "[x-debug] "
                            f"ros_frame={ros_frame_info['frame_index']} aria_vframe={aria_video_idx} "
                            f"aria_x={aria_eye_x:.1f} ros_x={ros_eye_x:.1f} sx={sx:.4f} "
                            f"aria_dx={aria_dx:+.1f} ros_dx={ros_dx:+.1f}"
                        )

                paired_count += 1
                if max_frames and paired_count >= int(max_frames):
                    return False
                return True

            _process.first_ros_stamp_s = None
            try:
                if not _process(first_ros):
                    return
                for fr in buffered:
                    if not _process(fr):
                        return
                for fr in frame_iter:
                    if not _process(fr):
                        break
            finally:
                ros_writer.release()
                sbs_writer.release()

            print(
                f"[sync] wrote {paired_count} paired frames to {output_side_by_side}; "
                f"skipped {skipped_start} before-window and {skipped_end} after-window ROS frames."
            )
            print(f"[sync] ROS overlay: {output_rosbag_overlay}")
            print(f"[sync] Aria video: {aria_video} ({aria_frame_count} frames)")
        finally:
            cap.release()

    @staticmethod
    def run_stationary_review_mode(argv=None):
        import cv2

        args = TimestampAligner.parse_stationary_review_args(argv)

        output_dir = args.output_dir
        if output_dir is None:
            output_dir = args.aria_csv.resolve().parent / "stationary_review"
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        first_frame = TimestampAligner.load_rosbag_image_frame(
            args.rosbag_dir,
            image_topic=args.ros_image_topic,
            frame_index=0,
        )
        ros_w = int(first_frame["image"].shape[1])
        ros_h = int(first_frame["image"].shape[0])

        # Load the per-written-RGB-frame manifest (or synthesize it from the
        # gaze CSV as a fallback for older recordings). This is the key piece
        # that lets us pair ROS bag frames to exact Aria video frame indices.
        manifest = TimestampAligner.load_rgb_overlay_manifest(
            aria_csv=args.aria_csv, aria_video=args.aria_video
        )
        effective_transform_mode = str(args.transform_mode)
        auto_affine_result = None
        auto_affine_report_path = None

        if bool(getattr(args, "auto_affine_gaze_fit", False)):
            # Use anchor mapping as the seed for local gaze-neighborhood matching.
            mapped_anchor_seed = TimestampAligner.map_aria_motion_to_ros_camera(
                aria_csv=args.aria_csv,
                ros_width=ros_w,
                ros_height=ros_h,
                aria_width=args.aria_width,
                aria_height=args.aria_height,
                dots=args.dots,
                eye_ros_start=args.eye_ros_start,
                head_ros_start=args.head_ros_start,
                eye_aria_anchor=args.eye_aria_anchor,
                head_aria_anchor=args.head_aria_anchor,
                gain_x=args.gain_x,
                gain_y=args.gain_y,
                transform_mode="anchor",
                affine_pairs=None,
            )
            auto_affine_result = TimestampAligner.auto_fit_affine_from_gaze_local_features(
                aria_video=args.aria_video,
                aria_manifest_df=manifest,
                mapped_df_anchor=mapped_anchor_seed,
                rosbag_dir=args.rosbag_dir,
                ros_image_topic=args.ros_image_topic,
                aria_time_column=args.time_column,
                max_tol_s=float(getattr(args, "sync_tolerance_s", 0.5)),
                search_range_s=float(getattr(args, "auto_affine_offset_range_s", 0.12)),
                search_step_s=float(getattr(args, "auto_affine_offset_step_s", 0.015)),
                temporal_sample_step=int(getattr(args, "auto_affine_temporal_sample_step", 8)),
                fit_sample_step=int(getattr(args, "auto_affine_fit_sample_step", 4)),
                patch_radius_px=float(getattr(args, "auto_affine_patch_radius_px", 180.0)),
                min_matches=int(getattr(args, "auto_affine_min_matches", 14)),
                min_inliers=int(getattr(args, "auto_affine_min_inliers", 8)),
                max_reproj_px=float(getattr(args, "auto_affine_max_reproj_px", 4.0)),
                holdout_mod=int(getattr(args, "auto_affine_holdout_mod", 5)),
                initial_time_offset_s=float(getattr(args, "mapped_time_offset_s", 0.0)),
            )
            # Apply the learned temporal shift and affine pair set automatically.
            args.mapped_time_offset_s = float(auto_affine_result["best_time_offset_s"])
            effective_transform_mode = "affine"
            fitted_pairs = auto_affine_result["affine_pairs"]
            mapped = TimestampAligner.map_aria_motion_to_ros_camera(
                aria_csv=args.aria_csv,
                ros_width=ros_w,
                ros_height=ros_h,
                aria_width=args.aria_width,
                aria_height=args.aria_height,
                dots=args.dots,
                eye_ros_start=args.eye_ros_start,
                head_ros_start=args.head_ros_start,
                eye_aria_anchor=args.eye_aria_anchor,
                head_aria_anchor=args.head_aria_anchor,
                gain_x=args.gain_x,
                gain_y=args.gain_y,
                transform_mode="affine",
                affine_pairs=fitted_pairs,
                affine_transform_override=auto_affine_result["transform"],
            )

            # Persist diagnostics for inspection/reproducibility.
            temporal_scan_csv = output_dir / "auto_affine_temporal_scan.csv"
            frame_samples_csv = output_dir / "auto_affine_frame_samples.csv"
            point_records_csv = output_dir / "auto_affine_point_records.csv"
            train_points_csv = output_dir / "auto_affine_train_points.csv"
            holdout_eval_csv = output_dir / "auto_affine_holdout_eval.csv"
            pd.DataFrame(auto_affine_result["temporal_diagnostics"]).to_csv(temporal_scan_csv, index=False)
            auto_affine_result["frame_samples_df"].to_csv(frame_samples_csv, index=False)
            auto_affine_result["point_records_df"].to_csv(point_records_csv, index=False)
            auto_affine_result["train_points_df"].to_csv(train_points_csv, index=False)
            auto_affine_result["holdout_eval_df"].to_csv(holdout_eval_csv, index=False)

            auto_affine_summary = {
                "best_time_offset_s": float(auto_affine_result["best_time_offset_s"]),
                "median_holdout_error_px": float(auto_affine_result["median_holdout_error_px"])
                if np.isfinite(auto_affine_result["median_holdout_error_px"])
                else None,
                "mean_holdout_error_px": float(auto_affine_result["mean_holdout_error_px"])
                if np.isfinite(auto_affine_result["mean_holdout_error_px"])
                else None,
                "direction_agreement": float(auto_affine_result["direction_agreement"])
                if np.isfinite(auto_affine_result["direction_agreement"])
                else None,
                "transform": auto_affine_result["transform"].to_json(),
                "fitted_pair_count": int(len(auto_affine_result["affine_pairs"])),
                "temporal_scan_csv": str(temporal_scan_csv),
                "frame_samples_csv": str(frame_samples_csv),
                "point_records_csv": str(point_records_csv),
                "train_points_csv": str(train_points_csv),
                "holdout_eval_csv": str(holdout_eval_csv),
            }
            auto_affine_report_path = output_dir / "auto_affine_report.json"
            with open(auto_affine_report_path, "w", encoding="utf-8") as file:
                json.dump(auto_affine_summary, file, indent=2)
            print(f"[auto-affine] Saved report to {auto_affine_report_path}")
        else:
            mapped = TimestampAligner.map_aria_motion_to_ros_camera(
                aria_csv=args.aria_csv,
                ros_width=ros_w,
                ros_height=ros_h,
                aria_width=args.aria_width,
                aria_height=args.aria_height,
                dots=args.dots,
                eye_ros_start=args.eye_ros_start,
                head_ros_start=args.head_ros_start,
                eye_aria_anchor=args.eye_aria_anchor,
                head_aria_anchor=args.head_aria_anchor,
                gain_x=args.gain_x,
                gain_y=args.gain_y,
                transform_mode=args.transform_mode,
                affine_pairs=args.affine_pair,
            )

        mapped_csv = output_dir / f"gaze_stream_{effective_transform_mode}_mapped.csv"
        TimestampAligner.save_dataframe(mapped, mapped_csv, f"{effective_transform_mode} mapped CSV")

        overlay_video_path = output_dir / f"rosbag_overlay_{effective_transform_mode}.avi"
        side_by_side_video = output_dir / f"aria_ros_side_by_side_{effective_transform_mode}.mp4"

        TimestampAligner.render_synced_side_by_side(
            aria_video=args.aria_video,
            aria_manifest_df=manifest,
            mapped_df=mapped,
            rosbag_dir=args.rosbag_dir,
            output_rosbag_overlay=overlay_video_path,
            output_side_by_side=side_by_side_video,
            ros_image_topic=args.ros_image_topic,
            dots=args.dots,
            transform_mode=effective_transform_mode,
            trail_length=40,
            debug_overlay=True,
            debug_print_every=args.debug_print_every,
            max_frames=args.max_frames,
            max_tol_s=float(getattr(args, "sync_tolerance_s", 0.5)),
            aria_time_column=args.time_column,
            mapped_time_offset_s=float(getattr(args, "mapped_time_offset_s", 0.0)),
            debug_eye_x=bool(getattr(args, "debug_eye_x", False)),
            sync_basis=str(getattr(args, "sync_basis", "relative")),
        )

        stable_segments = TimestampAligner.compute_stable_gaze_segments(
            mapped,
            dot_name=args.stable_dot,
            time_column=args.time_column,
            max_step_px=args.stable_max_step_px,
            min_duration_s=args.stable_min_duration_s,
            min_samples=args.stable_min_samples,
        )
        stable_segments_csv = output_dir / f"stable_{args.stable_dot}_segments.csv"
        TimestampAligner.save_dataframe(stable_segments, stable_segments_csv, f"stable {args.stable_dot} segments")

        summary = {
            "mode": "stationary_review",
            "transform_mode": effective_transform_mode,
            "aria_csv": str(args.aria_csv),
            "aria_video": str(args.aria_video),
            "rosbag_dir": str(args.rosbag_dir),
            "ros_image_topic": first_frame.get("topic"),
            "ros_frame_size": [ros_w, ros_h],
            "mapped_csv": str(mapped_csv),
            "rosbag_overlay_video": str(overlay_video_path),
            "side_by_side_video": str(side_by_side_video),
            "side_by_side_pairing": "timestamp_synced_v2",
            "sync_basis": str(getattr(args, "sync_basis", "relative")),
            "sync_tolerance_s": float(getattr(args, "sync_tolerance_s", 0.5)),
            "mapped_time_offset_s": float(getattr(args, "mapped_time_offset_s", 0.0)),
            "debug_eye_x": bool(getattr(args, "debug_eye_x", False)),
            "aria_rgb_manifest_frames": int(len(manifest)),
            "stable_segments_csv": str(stable_segments_csv),
            "stable_segment_count": int(len(stable_segments)),
            "stable_dot": args.stable_dot,
            "stable_max_step_px": float(args.stable_max_step_px),
            "stable_min_duration_s": float(args.stable_min_duration_s),
            "stable_min_samples": int(args.stable_min_samples),
        }
        if auto_affine_result is not None:
            summary["auto_affine_enabled"] = True
            summary["auto_affine_report"] = str(auto_affine_report_path) if auto_affine_report_path is not None else None
            summary["auto_affine_best_time_offset_s"] = float(auto_affine_result["best_time_offset_s"])
            summary["auto_affine_median_holdout_error_px"] = (
                float(auto_affine_result["median_holdout_error_px"])
                if np.isfinite(auto_affine_result["median_holdout_error_px"])
                else None
            )
            summary["auto_affine_direction_agreement"] = (
                float(auto_affine_result["direction_agreement"])
                if np.isfinite(auto_affine_result["direction_agreement"])
                else None
            )
        summary_path = output_dir / "stationary_review_summary.json"
        with open(summary_path, "w", encoding="utf-8") as file:
            json.dump(summary, file, indent=2)
        print(f"[review] Saved stationary review summary to {summary_path}")
        print(f"[review] Side-by-side video: {side_by_side_video}")
        print(f"[review] Stable segments: {stable_segments_csv}")

    @staticmethod
    def run_2d_motion_mode(argv=None):
        args = TimestampAligner.parse_2d_motion_args(argv)

        if args.output_csv is not None:
            if args.ros_width is None or args.ros_height is None:
                raise SystemExit("--output-csv requires --ros-width and --ros-height")
            mapped = TimestampAligner.map_aria_motion_to_ros_camera(
                aria_csv=args.aria_csv,
                ros_width=args.ros_width,
                ros_height=args.ros_height,
                aria_width=args.aria_width,
                aria_height=args.aria_height,
                dots=args.dots,
                eye_ros_start=args.eye_ros_start,
                head_ros_start=args.head_ros_start,
                eye_aria_anchor=args.eye_aria_anchor,
                head_aria_anchor=args.head_aria_anchor,
                gain_x=args.gain_x,
                gain_y=args.gain_y,
                transform_mode=args.transform_mode,
                affine_pairs=args.affine_pair,
            )
            args.output_csv.parent.mkdir(parents=True, exist_ok=True)
            mapped.to_csv(args.output_csv, index=False)
            print(f"Saved mapped dots to {args.output_csv}")

        if args.overlay_ros:
            TimestampAligner.run_ros_overlay(args)

        if args.overlay_rosbag:
            TimestampAligner.run_rosbag_overlay(args)

        if args.output_csv is None and not args.overlay_ros:
            if not args.overlay_rosbag:
                raise SystemExit("Nothing to do. Use --output-csv, --overlay-ros, and/or --overlay-rosbag.")

    @staticmethod
    def run_ros_overlay(args):
        import cv2
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        def bgr8_to_image_msg(image, header):
            msg = Image()
            msg.header = header
            msg.height = int(image.shape[0])
            msg.width = int(image.shape[1])
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = int(image.shape[1] * 3)
            msg.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
            return msg

        class AriaDotOverlayNode(Node):
            def __init__(self):
                super().__init__("aria_dot_overlay")
                self.frame_count = 0
                self.last_stamp = None
                self.ros_start_time = None
                self.mapped_df = None
                self.relative_times = None
                self.motion_duration_s = 0.0
                self.bad_encodings = set()
                self.eye_trail = []
                self.head_trail = []
                self.publisher = self.create_publisher(Image, args.overlay_topic, 10)
                self.subscription = self.create_subscription(Image, args.image_topic, self.on_image, 10)
                self.get_logger().info(
                    f"Overlaying Aria dots from {args.aria_csv} on {args.image_topic}; "
                    f"publishing {args.overlay_topic}"
                )

            def ensure_mapping(self, width, height):
                if self.mapped_df is not None:
                    return
                self.mapped_df = TimestampAligner.map_aria_motion_to_ros_camera(
                    aria_csv=args.aria_csv,
                    ros_width=width,
                    ros_height=height,
                    aria_width=args.aria_width,
                    aria_height=args.aria_height,
                    dots=args.dots,
                    eye_ros_start=args.eye_ros_start,
                    head_ros_start=args.head_ros_start,
                    eye_aria_anchor=args.eye_aria_anchor,
                    head_aria_anchor=args.head_aria_anchor,
                    gain_x=args.gain_x,
                    gain_y=args.gain_y,
                    transform_mode=args.transform_mode,
                    affine_pairs=args.affine_pair,
                )
                if args.time_column in self.mapped_df.columns:
                    times = self.mapped_df[args.time_column].astype(float)
                    first_time = float(times.dropna().iloc[0])
                    self.relative_times = (times - first_time).to_numpy()
                    finite_times = self.relative_times[np.isfinite(self.relative_times)]
                    self.motion_duration_s = float(np.max(finite_times)) if finite_times.size else 0.0
                else:
                    self.relative_times = None
                    self.motion_duration_s = 0.0
                    self.get_logger().warning(
                        f"Time column {args.time_column!r} not found; falling back to frame-index matching."
                    )

            def row_for_message(self, msg):
                if self.relative_times is None:
                    idx = min(self.frame_count, len(self.mapped_df) - 1)
                    return self.mapped_df.iloc[idx], idx, None

                stamp_s = TimestampAligner.stamp_to_seconds(msg.header.stamp)
                if stamp_s <= 0.0:
                    stamp_s = self.get_clock().now().nanoseconds * 1e-9

                if self.ros_start_time is None or (
                    self.last_stamp is not None and stamp_s < self.last_stamp - 0.5
                ):
                    self.ros_start_time = stamp_s

                self.last_stamp = stamp_s
                elapsed_s = stamp_s - self.ros_start_time
                if args.loop_motion and self.motion_duration_s > 0:
                    elapsed_s = elapsed_s % self.motion_duration_s

                idx = int(np.searchsorted(self.relative_times, elapsed_s, side="left"))
                if idx <= 0:
                    return self.mapped_df.iloc[0], 0, elapsed_s
                if idx >= len(self.mapped_df):
                    return self.mapped_df.iloc[-1], len(self.mapped_df) - 1, elapsed_s

                prev_dt = abs(elapsed_s - self.relative_times[idx - 1])
                next_dt = abs(self.relative_times[idx] - elapsed_s)
                chosen_idx = idx - 1 if prev_dt <= next_dt else idx
                return self.mapped_df.iloc[chosen_idx], chosen_idx, elapsed_s

            def append_trail(self, trail, dot):
                if dot is None:
                    return
                trail.append((int(round(dot[0])), int(round(dot[1]))))
                max_len = max(1, int(args.trail_length))
                if len(trail) > max_len:
                    del trail[:-max_len]

            def on_image(self, msg):
                try:
                    image = TimestampAligner.image_msg_to_bgr8(msg)
                except Exception as exc:
                    if msg.encoding not in self.bad_encodings:
                        self.bad_encodings.add(msg.encoding)
                        self.get_logger().error(f"Could not convert image: {exc}")
                    return
                self.ensure_mapping(msg.width, msg.height)
                row, row_idx, elapsed_s = self.row_for_message(msg)

                eye_dot = None
                if "eye" in args.dots and TimestampAligner.is_finite_dot(row, "eye_ros_x", "eye_ros_y"):
                    eye_dot = (float(row["eye_ros_x"]), float(row["eye_ros_y"]))
                    TimestampAligner.draw_dot(image, eye_dot, (255, 0, 255), "eye")

                head_dot = None
                if "head" in args.dots and TimestampAligner.is_finite_dot(row, "head_ros_x", "head_ros_y"):
                    head_dot = (float(row["head_ros_x"]), float(row["head_ros_y"]))
                    TimestampAligner.draw_dot(image, head_dot, (255, 255, 0), "head")

                if args.debug_overlay:
                    self.append_trail(self.eye_trail, eye_dot)
                    self.append_trail(self.head_trail, head_dot)
                    TimestampAligner.draw_trail(image, self.eye_trail, (255, 0, 255))
                    TimestampAligner.draw_trail(image, self.head_trail, (255, 255, 0))
                    eye_motion_text = (
                        f"aria_d=({row.get('eye_aria_delta_x', np.nan)}, {row.get('eye_aria_delta_y', np.nan)})"
                        if args.transform_mode == "delta"
                        else f"aria_anchor=({row.get('eye_aria_anchor_x', np.nan)}, {row.get('eye_aria_anchor_y', np.nan)})"
                    )
                    head_motion_text = (
                        f"aria_d=({row.get('head_aria_delta_x', np.nan)}, {row.get('head_aria_delta_y', np.nan)})"
                        if args.transform_mode == "delta"
                        else f"aria_anchor=({row.get('head_aria_anchor_x', np.nan)}, {row.get('head_aria_anchor_y', np.nan)})"
                    )
                    status_lines = [
                        f"{args.transform_mode} frame={self.frame_count} csv_row={row_idx}/{len(self.mapped_df) - 1} elapsed={elapsed_s if elapsed_s is not None else -1:.2f}s",
                        f"eye=({row.get('eye_ros_x', np.nan)}, {row.get('eye_ros_y', np.nan)}) {eye_motion_text}",
                        f"head=({row.get('head_ros_x', np.nan)}, {row.get('head_ros_y', np.nan)}) {head_motion_text}",
                        f"topic={args.image_topic} -> {args.overlay_topic}",
                    ]
                    TimestampAligner.draw_status_panel(image, status_lines)

                if args.debug_print_every and self.frame_count % args.debug_print_every == 0:
                    self.get_logger().info(
                        f"frame={self.frame_count} csv_row={row_idx} "
                        f"eye=({row.get('eye_ros_x', np.nan)}, {row.get('eye_ros_y', np.nan)}) "
                        f"head=({row.get('head_ros_x', np.nan)}, {row.get('head_ros_y', np.nan)})"
                    )

                out_msg = bgr8_to_image_msg(image, msg.header)
                self.publisher.publish(out_msg)

                if args.display_window:
                    cv2.imshow(f"Aria {args.transform_mode} mapping on ROS camera", image)
                    cv2.waitKey(1)

                self.frame_count += 1

        rclpy.init(args=None)
        node = AriaDotOverlayNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if args.display_window:
                cv2.destroyAllWindows()

    @staticmethod
    def run_rosbag_overlay(args):
        import cv2

        if args.rosbag_dir is None:
            raise SystemExit("--overlay-rosbag requires --rosbag-dir")

        frame_iter = TimestampAligner.iterate_rosbag_image_frames(
            args.rosbag_dir,
            image_topic=args.ros_image_topic,
        )
        try:
            first_frame = next(frame_iter)
        except StopIteration:
            raise SystemExit(f"No ROS image frames found in {args.rosbag_dir}")

        frame_h, frame_w = first_frame["image"].shape[:2]
        mapped_df = TimestampAligner.map_aria_motion_to_ros_camera(
            aria_csv=args.aria_csv,
            ros_width=frame_w,
            ros_height=frame_h,
            aria_width=args.aria_width,
            aria_height=args.aria_height,
            dots=args.dots,
            eye_ros_start=args.eye_ros_start,
            head_ros_start=args.head_ros_start,
            eye_aria_anchor=args.eye_aria_anchor,
            head_aria_anchor=args.head_aria_anchor,
            gain_x=args.gain_x,
            gain_y=args.gain_y,
            transform_mode=args.transform_mode,
            affine_pairs=args.affine_pair,
        )

        if args.time_column in mapped_df.columns:
            times = mapped_df[args.time_column].astype(float)
            first_time = float(times.dropna().iloc[0])
            relative_times = (times - first_time).to_numpy()
            finite_times = relative_times[np.isfinite(relative_times)]
            motion_duration_s = float(np.max(finite_times)) if finite_times.size else 0.0
        else:
            relative_times = None
            motion_duration_s = 0.0

        def row_for_frame(elapsed_s, frame_index):
            if relative_times is None:
                idx = min(frame_index, len(mapped_df) - 1)
                return mapped_df.iloc[idx], idx

            effective_elapsed = elapsed_s
            if args.loop_motion and motion_duration_s > 0:
                effective_elapsed = effective_elapsed % motion_duration_s

            idx = int(np.searchsorted(relative_times, effective_elapsed, side="left"))
            if idx <= 0:
                return mapped_df.iloc[0], 0
            if idx >= len(mapped_df):
                return mapped_df.iloc[-1], len(mapped_df) - 1

            prev_dt = abs(effective_elapsed - relative_times[idx - 1])
            next_dt = abs(relative_times[idx] - effective_elapsed)
            chosen_idx = idx - 1 if prev_dt <= next_dt else idx
            return mapped_df.iloc[chosen_idx], chosen_idx

        writer = None
        if args.overlay_video is not None:
            args.overlay_video.parent.mkdir(parents=True, exist_ok=True)
            suffix = args.overlay_video.suffix.lower()
            fourcc = cv2.VideoWriter_fourcc(*("MJPG" if suffix == ".avi" else "mp4v"))
            writer = cv2.VideoWriter(str(args.overlay_video), fourcc, 10.0, (frame_w, frame_h))
            if not writer.isOpened():
                raise SystemExit(f"Could not open output video: {args.overlay_video}")

        eye_trail = []
        head_trail = []
        first_stamp_s = None
        frame_count = 0

        def process_frame(frame_info):
            nonlocal first_stamp_s, frame_count

            image = frame_info["image"].copy()
            stamp_s = frame_info["stamp_s"]
            if first_stamp_s is None:
                first_stamp_s = stamp_s
            elapsed_s = stamp_s - first_stamp_s
            row, row_idx = row_for_frame(elapsed_s, frame_info["frame_index"])

            eye_dot = None
            if "eye" in args.dots and TimestampAligner.is_finite_dot(row, "eye_ros_x", "eye_ros_y"):
                eye_dot = (float(row["eye_ros_x"]), float(row["eye_ros_y"]))
                TimestampAligner.draw_dot(image, eye_dot, (255, 0, 255), "eye")

            head_dot = None
            if "head" in args.dots and TimestampAligner.is_finite_dot(row, "head_ros_x", "head_ros_y"):
                head_dot = (float(row["head_ros_x"]), float(row["head_ros_y"]))
                TimestampAligner.draw_dot(image, head_dot, (255, 255, 0), "head")

            if args.debug_overlay:
                if eye_dot is not None:
                    eye_trail.append((int(round(eye_dot[0])), int(round(eye_dot[1]))))
                    max_len = max(1, int(args.trail_length))
                    if len(eye_trail) > max_len:
                        del eye_trail[:-max_len]
                if head_dot is not None:
                    head_trail.append((int(round(head_dot[0])), int(round(head_dot[1]))))
                    max_len = max(1, int(args.trail_length))
                    if len(head_trail) > max_len:
                        del head_trail[:-max_len]
                TimestampAligner.draw_trail(image, eye_trail, (255, 0, 255))
                TimestampAligner.draw_trail(image, head_trail, (255, 255, 0))
                status_lines = [
                    f"rosbag {args.transform_mode} frame={frame_info['frame_index']} csv_row={row_idx}/{len(mapped_df) - 1} elapsed={elapsed_s:.2f}s topic={frame_info['topic']}",
                    f"eye=({row.get('eye_ros_x', np.nan)}, {row.get('eye_ros_y', np.nan)})",
                    f"head=({row.get('head_ros_x', np.nan)}, {row.get('head_ros_y', np.nan)})",
                    f"bag={args.rosbag_dir}",
                ]
                TimestampAligner.draw_status_panel(image, status_lines)

            if args.debug_print_every and frame_count % args.debug_print_every == 0:
                print(
                    f"[rosbag-overlay] frame={frame_info['frame_index']} csv_row={row_idx} "
                    f"eye=({row.get('eye_ros_x', np.nan)}, {row.get('eye_ros_y', np.nan)}) "
                    f"head=({row.get('head_ros_x', np.nan)}, {row.get('head_ros_y', np.nan)})"
                )

            if writer is not None:
                writer.write(image)
            if args.display_window:
                cv2.imshow(f"ROS bag {args.transform_mode} overlay", image)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    return False

            frame_count += 1
            if args.max_frames and frame_count >= args.max_frames:
                return False
            return True

        try:
            if not process_frame(first_frame):
                return
            for frame_info in frame_iter:
                if not process_frame(frame_info):
                    break
        finally:
            if writer is not None:
                writer.release()
                print(f"[rosbag-overlay] Saved overlay video to {args.overlay_video}")
            if args.display_window:
                cv2.destroyAllWindows()

    @staticmethod
    def main():
        if "--collect-affine-points" in sys.argv:
            TimestampAligner.run_affine_collection_mode(sys.argv[1:])
            return

        if "--stationary-review" in sys.argv:
            TimestampAligner.run_stationary_review_mode(sys.argv[1:])
            return

        if "--map-2d-motion" in sys.argv or "--overlay-ros" in sys.argv:
            TimestampAligner.run_2d_motion_mode(sys.argv[1:])
            return

        # Now we shall call all the functions in the main function to perform the entire process of aligning timestamps, transforming coordinates, and saving the transformed data.
        parser = argparse.ArgumentParser(description="Align Aria timestamps with ROS bag timestamps and transform coordinates.")
        parser.add_argument("--aria_file", type=Path, required=True, help="Path to the Aria timestamps JSON file.")
        parser.add_argument("--rosbag_file", type=Path, required=True, help="Path to the ROS bag timestamps file.")
        parser.add_argument("--output_file", type=Path, required=True, help="Path to save the aligned timestamps JSON file.")
        parser.add_argument("--rotation_angles", type=float, nargs=3, required=True, help="Rotation angles (roll, pitch, yaw) in degrees for the transformation.")
        parser.add_argument("--translation_vector", type=float, nargs=3, required=True, help="Translation vector (x, y, z) for the transformation.")
        args = parser.parse_args()
        # Build the transformation matrix
        transformation_matrix = TimestampAligner.build_transformation_matrix(args.rotation_angles, args.translation_vector)
        # Load Aria data
        with open(args.aria_file, "r") as f:
            aria_data = json.load(f)

        # Align timestamps
        TimestampAligner.align_timestamps(args.aria_file, args.rosbag_file, args.output_file)
        # Transform coordinates
        transformed_data = TimestampAligner.transform_coordinates(aria_data, transformation_matrix)
        # Save transformed data
        TimestampAligner.save_transformed_data(transformed_data, args.output_file)


if __name__ == "__main__":
    TimestampAligner.main()
