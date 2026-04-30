#!/bin/bash
# Record a case-1 stationary gaze test:
# - start ROS bag recording for the robot or vehicle camera topics
# - launch the Aria eye viewer with recording enabled
# - stop the ROS bag when the viewer exits
# - print the exact review command for the captured run

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

CAMERA_TOPIC="/cam_0/image_raw"
CAMERA_INFO_TOPIC="/cam_0/camera_info"
SESSION_NAME="case1_$(date +%Y%m%d_%H%M%S)"
OUTPUT_BASE="$SCRIPT_DIR/recordings"
REVIEW_MODE="anchor"
ARIA_EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  ./record_case1_test.sh [options] [-- <extra aria_eye_viewer args>]

Options:
  --camera-topic TOPIC         ROS image topic to record. Default: /cam_0/image_raw
  --camera-info-topic TOPIC    ROS camera_info topic to record. Default: /cam_0/camera_info
  --output-base DIR            Base directory for outputs. Default: ./recordings
  --session-name NAME          Session directory name. Default: case1_<timestamp>
  --review-mode MODE           Review transform mode to suggest later. Default: anchor
  --no-camera-info             Record only the image topic.
  --help                       Show this message.

Any arguments after '--' are passed through to aria_teleop/aria_eye_viewer.py
via run_aria_eye_viewer.sh. Example:

  ./record_case1_test.sh -- --gaze-source open-model --gaze-model-device cpu
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
        --output-base)
            OUTPUT_BASE="$2"
            shift 2
            ;;
        --session-name)
            SESSION_NAME="$2"
            shift 2
            ;;
        --review-mode)
            REVIEW_MODE="$2"
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
        --)
            shift
            while [ $# -gt 0 ]; do
                ARIA_EXTRA_ARGS+=("$1")
                shift
            done
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

SESSION_DIR="$OUTPUT_BASE/$SESSION_NAME"
ROSBAG_DIR="$SESSION_DIR/rosbag"
ARIA_ROOT="$SESSION_DIR/aria"
LOG_DIR="$SESSION_DIR/logs"
mkdir -p "$ARIA_ROOT" "$LOG_DIR"

if [ -e "$ROSBAG_DIR" ]; then
    echo "[case1] ERROR: ROS bag output path already exists: $ROSBAG_DIR" >&2
    echo "[case1] Choose a new --session-name or remove the existing directory first." >&2
    exit 1
fi

ROSBAG_LOG="$LOG_DIR/rosbag_record.log"
REVIEW_DIR="$SESSION_DIR/stationary_review_${REVIEW_MODE}"
ROSBAG_PID=""

cleanup() {
    if [ -n "${ROSBAG_PID:-}" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo ""
        echo "[case1] Stopping ros2 bag record (PID $ROSBAG_PID)..."
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        wait "$ROSBAG_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "=== Case-1 Stationary Gaze Test Recorder ==="
echo "Session:      $SESSION_DIR"
echo "ROS bag dir:  $ROSBAG_DIR"
echo "Aria root:    $ARIA_ROOT"
echo "Camera topic: $CAMERA_TOPIC"
if [ -n "$CAMERA_INFO_TOPIC" ]; then
    echo "Info topic:   $CAMERA_INFO_TOPIC"
else
    echo "Info topic:   <disabled>"
fi
echo ""

ROSBAG_CMD=(ros2 bag record -o "$ROSBAG_DIR" "$CAMERA_TOPIC")
if [ -n "$CAMERA_INFO_TOPIC" ]; then
    ROSBAG_CMD+=("$CAMERA_INFO_TOPIC")
fi

echo "[case1] Starting ROS bag recording..."
"${ROSBAG_CMD[@]}" >"$ROSBAG_LOG" 2>&1 &
ROSBAG_PID="$!"
sleep 2
if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
    echo "[case1] ERROR: ros2 bag record exited early. Check $ROSBAG_LOG" >&2
    exit 1
fi
echo "[case1] ros2 bag record PID: $ROSBAG_PID"
echo ""

ARIA_CMD=(sudo bash "$SCRIPT_DIR/run_aria_eye_viewer.sh" --record-dir "$ARIA_ROOT")
if [ ${#ARIA_EXTRA_ARGS[@]} -gt 0 ]; then
    ARIA_CMD+=("${ARIA_EXTRA_ARGS[@]}")
fi

echo "[case1] Launching Aria viewer with recording enabled..."
echo "[case1] Close the Aria viewer window or press Ctrl+C there when the run is done."
echo ""
"${ARIA_CMD[@]}"

cleanup
trap - EXIT INT TERM

ARIA_SESSION_DIR="$(find "$ARIA_ROOT" -mindepth 1 -maxdepth 1 -type d -name 'aria_eye_*' | sort | tail -n 1)"
if [ -z "$ARIA_SESSION_DIR" ]; then
    echo "[case1] WARNING: could not find the recorded Aria session directory under $ARIA_ROOT" >&2
    exit 1
fi

ARIA_CSV="$ARIA_SESSION_DIR/gaze_stream.csv"
ARIA_VIDEO="$ARIA_SESSION_DIR/rgb_overlay.avi"

echo ""
echo "[case1] Recording complete."
echo "Aria session: $ARIA_SESSION_DIR"
echo "ROS bag:      $ROSBAG_DIR"
echo "ROS log:      $ROSBAG_LOG"
echo ""
echo "[case1] Review command:"
printf '%q ' python3 aria_teleop/align_aria_and_rosbag.py \
    --stationary-review \
    --aria-csv "$ARIA_CSV" \
    --aria-video "$ARIA_VIDEO" \
    --rosbag-dir "$ROSBAG_DIR" \
    --transform-mode "$REVIEW_MODE" \
    --output-dir "$REVIEW_DIR"
echo ""
