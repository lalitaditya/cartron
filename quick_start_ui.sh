#!/bin/bash

# Robot Arm UI Quick Start Script (Cartron)
# Usage: ./quick_start_ui.sh

# Path to the repo root
REPO_ROOT="/home/lalit/Robotic_arm/cartron"
UI_DIR="ui"

echo "========================================"
echo "      Piper Robot Arm UI (Cartron)      "
echo "========================================"

# 1. Configure CAN Interface (similar to quick_start.sh)
INTERFACE="can0"
BAUDRATE="1000000"

echo "[1/3] Checking CAN interface '$INTERFACE'..."

if ip link show "$INTERFACE" | grep -q "UP"; then
    echo "      $INTERFACE is already UP."
else
    echo "      $INTERFACE is DOWN. Activating..."
    sudo bash "$REPO_ROOT/can_activate.sh" "$INTERFACE" "$BAUDRATE"
    
    if [ $? -ne 0 ]; then
        echo "      Failed to activate CAN interface."
        exit 1
    fi
    echo "      $INTERFACE activated successfully."
fi

# 2. Activate Conda Environment
echo "[2/3] Activating 'piper_env'..."
CONDA_PATH="$HOME/miniconda3/etc/profile.d/conda.sh"
if [ -f "$CONDA_PATH" ]; then
    source "$CONDA_PATH"
else
    echo "      Warning: Could not find conda.sh, assuming 'conda' in path."
fi

conda activate piper_env

# 3. Launch UI
echo "[3/3] Launching UI..."
echo "      Starting piper_ui.py..."

cd "$UI_DIR"
python piper_ui.py
