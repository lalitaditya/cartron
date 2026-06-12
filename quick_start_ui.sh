#!/bin/bash

# Robot Arm UI Quick Start Script (Cartron)
# Usage: ./quick_start_ui.sh

# Path to this repo, derived from the script location so the checkout is portable.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$REPO_ROOT/ui"

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

# 2. Activate Python Environment
echo "[2/3] Activating Python environment..."
CONDA_FOUND=0
for CONDA_PATH in \
    "$HOME/miniconda3/etc/profile.d/conda.sh" \
    "$HOME/anaconda3/etc/profile.d/conda.sh" \
    "$HOME/miniforge3/etc/profile.d/conda.sh" \
    "$HOME/mambaforge/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"
do
    if [ -f "$CONDA_PATH" ]; then
        source "$CONDA_PATH"
        CONDA_FOUND=1
        break
    fi
done

if command -v conda > /dev/null 2>&1; then
    CONDA_FOUND=1
fi

if [ "$CONDA_FOUND" -eq 1 ] && conda activate piper_env > /dev/null 2>&1; then
    echo "      Conda environment 'piper_env' active."
else
    if [ "$CONDA_FOUND" -eq 0 ]; then
        echo "      Conda was not found; checking system Python instead..."
    else
        echo "      Conda environment 'piper_env' was not found; checking system Python instead..."
    fi

    export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
    if python -c "import piper_sdk, can" > /dev/null 2>&1; then
        echo "      System Python is ready."
    else
        echo "      Python environment is not ready."
        echo "      Install/setup one of these:"
        echo "        conda create -n piper_env python=3.10"
        echo "        conda activate piper_env"
        echo "        pip install ."
        echo "      Or install dependencies for system Python:"
        echo "        python -m pip install ."
        exit 1
    fi
fi

# 3. Launch UI
echo "[3/3] Launching UI..."
echo "      Starting piper_ui.py..."

cd "$UI_DIR"
python piper_ui.py
