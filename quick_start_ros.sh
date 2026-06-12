#!/bin/bash

# ROS 2 Piper driver quick start.
# Usage:
#   ./quick_start_ros.sh
#   ./quick_start_ros.sh log_level:=warn auto_enable:=false

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$REPO_ROOT/src/piper_ros"
INTERFACE="${CAN_INTERFACE:-can0}"
BAUDRATE="${CAN_BAUDRATE:-1000000}"
GRIPPER_VAL_MUTIPLE="${GRIPPER_VAL_MUTIPLE:-2}"
AUTO_ENABLE="${AUTO_ENABLE:-false}"

echo "========================================"
echo "      Piper ROS 2 Driver Quick Start    "
echo "========================================"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble setup file was not found at /opt/ros/humble/setup.bash"
    exit 1
fi

source /opt/ros/humble/setup.bash

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "ROS workspace has not been built yet. Building now..."
    cd "$ROS_WS"
    colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE="$(command -v python3)"
fi

source "$ROS_WS/install/setup.bash"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

echo "[1/2] Activating CAN interface '$INTERFACE'..."
bash "$ROS_WS/can_activate.sh" "$INTERFACE" "$BAUDRATE"

echo "[2/2] Launching Piper ROS driver..."
exec ros2 launch piper start_single_piper.launch.py \
    can_port:="$INTERFACE" \
    gripper_val_mutiple:="$GRIPPER_VAL_MUTIPLE" \
    auto_enable:="$AUTO_ENABLE" \
    "$@"
