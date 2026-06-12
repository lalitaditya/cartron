#!/bin/bash

# ROS 2 MoveIt quick start.
# Usage:
#   ./quick_start_moveit.sh
#   ./quick_start_moveit.sh no-gripper

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$REPO_ROOT/src/piper_ros"
MODE="${1:-with-gripper}"

echo "========================================"
echo "      Piper MoveIt Quick Start          "
echo "========================================"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble setup file was not found at /opt/ros/humble/setup.bash"
    exit 1
fi

source /opt/ros/humble/setup.bash

if ! python3 -c "import moveit_configs_utils" > /dev/null 2>&1; then
    echo "MoveIt Python config utilities are missing."
    echo "Install the MoveIt runtime dependencies with:"
    echo "  sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers"
    exit 1
fi

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "ROS workspace has not been built yet. Building now..."
    cd "$ROS_WS"
    colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE="$(command -v python3)"
fi

source "$ROS_WS/install/setup.bash"

case "$MODE" in
    with-gripper)
        exec ros2 launch piper_with_gripper_moveit demo.launch.py
        ;;
    no-gripper)
        exec ros2 launch piper_no_gripper_moveit demo.launch.py
        ;;
    *)
        echo "Unknown mode: $MODE"
        echo "Use one of: with-gripper, no-gripper"
        exit 1
        ;;
esac
