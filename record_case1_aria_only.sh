#!/bin/bash
# Record only the Aria side of a case-1 stationary gaze test on the laptop
# connected to the glasses.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

SESSION_NAME="case1_aria_$(date +%Y%m%d_%H%M%S)"
OUTPUT_BASE="$SCRIPT_DIR/recordings"
ARIA_EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  ./record_case1_aria_only.sh [options] [-- <extra aria_eye_viewer args>]

Options:
  --output-base DIR            Base directory for outputs. Default: ./recordings
  --session-name NAME          Session directory name. Default: case1_aria_<timestamp>
  --help                       Show this message.

Any arguments after '--' are passed through to aria_teleop/aria_eye_viewer.py
via run_aria_eye_viewer.sh. Example:

  ./record_case1_aria_only.sh -- --gaze-source open-model --gaze-model-device cpu
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --output-base)
            OUTPUT_BASE="$2"
            shift 2
            ;;
        --session-name)
            SESSION_NAME="$2"
            shift 2
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
ARIA_ROOT="$SESSION_DIR/aria"
mkdir -p "$ARIA_ROOT"

echo "=== Case-1 Aria Recorder ==="
echo "Session:   $SESSION_DIR"
echo "Aria root: $ARIA_ROOT"
echo ""

ARIA_CMD=(sudo bash "$SCRIPT_DIR/run_aria_eye_viewer.sh" --record-dir "$ARIA_ROOT")
if [ ${#ARIA_EXTRA_ARGS[@]} -gt 0 ]; then
    ARIA_CMD+=("${ARIA_EXTRA_ARGS[@]}")
fi

echo "[case1-aria] Launching Aria viewer with recording enabled..."
echo "[case1-aria] Close the Aria viewer window or press Ctrl+C there when the run is done."
echo ""
"${ARIA_CMD[@]}"

ARIA_SESSION_DIR="$(find "$ARIA_ROOT" -mindepth 1 -maxdepth 1 -type d -name 'aria_eye_*' | sort | tail -n 1)"
if [ -z "$ARIA_SESSION_DIR" ]; then
    echo "[case1-aria] WARNING: could not find the recorded Aria session directory under $ARIA_ROOT" >&2
    exit 1
fi

echo ""
echo "[case1-aria] Recording complete."
echo "Aria session: $ARIA_SESSION_DIR"
echo "Aria CSV:     $ARIA_SESSION_DIR/gaze_stream.csv"
echo "Aria video:   $ARIA_SESSION_DIR/rgb_overlay.avi"
