#!/bin/bash
# Record only the robot or vehicle ROS camera bag on the robot computer.

set -euo pipefail

CAMERA_TOPIC="/cam_0/image_raw"
CAMERA_INFO_TOPIC="/cam_0/camera_info"
OUTPUT_NAME="case1_robot_bag_$(date +%Y%m%d_%H%M%S)"

usage() {
    cat <<'EOF'
Usage:
  ./record_case1_robot_bag.sh [options]

Options:
  --camera-topic TOPIC         ROS image topic to record. Default: /cam_0/image_raw
  --camera-info-topic TOPIC    ROS camera_info topic to record. Default: /cam_0/camera_info
  --output-name NAME           ros2 bag output name. Default: case1_robot_bag_<timestamp>
  --no-camera-info             Record only the image topic.
  --help                       Show this message.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --camera-topic)
            CAMERA_TOPIC="$2"
            shift 2
            ;;
        --camera-info-topic)
            CAMERA_INFO_TOPIC="$2"
            shift 2
            ;;
        --output-name)
            OUTPUT_NAME="$2"
            shift 2
            ;;
        --no-camera-info)
            CAMERA_INFO_TOPIC=""
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

CMD=(ros2 bag record -o "$OUTPUT_NAME" "$CAMERA_TOPIC")
if [ -n "$CAMERA_INFO_TOPIC" ]; then
    CMD+=("$CAMERA_INFO_TOPIC")
fi

echo "=== Case-1 Robot ROS Bag Recorder ==="
echo "Output:       $OUTPUT_NAME"
echo "Camera topic: $CAMERA_TOPIC"
if [ -n "$CAMERA_INFO_TOPIC" ]; then
    echo "Info topic:   $CAMERA_INFO_TOPIC"
else
    echo "Info topic:   <disabled>"
fi
echo ""
echo "[case1-robot] Starting ros2 bag record..."
echo "[case1-robot] Stop with Ctrl+C when the run is finished."
echo ""

"${CMD[@]}"
