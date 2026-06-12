#!/bin/bash

# Robot Arm Quick Start Script
# Usage: 
#   ./quick_start.sh             -> Sets up CAN and drops you into a shell with piper_env active
#   ./quick_start.sh <command>   -> Sets up CAN and runs the specific command

# Path to this repo, derived from the script location so the checkout is portable.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "      Piper Robot Arm Quick Start       "
echo "========================================"

# 1. Configure CAN Interface
# We default to can0 and 1000000 baud as confirmed previously
INTERFACE="can0"
BAUDRATE="1000000"

echo "[1/2] Checking CAN interface '$INTERFACE'..."

# Check if interface exists
if ! ip link show "$INTERFACE" > /dev/null 2>&1; then
    echo "      Interface $INTERFACE not found! Checking connected USB devices..."
    bash "$REPO_ROOT/find_all_can_port.sh"
    echo "      Please ensure the USB-to-CAN module is plugged in."
    exit 1
fi

# Check if interface is UP
if ip link show "$INTERFACE" | grep -q "UP"; then
    echo "      $INTERFACE is already UP."
else
    echo "      $INTERFACE is DOWN. Activating... (sudo password may be required)"
    sudo bash "$REPO_ROOT/can_activate.sh" "$INTERFACE" "$BAUDRATE"
    
    if [ $? -ne 0 ]; then
        echo "      Failed to activate CAN interface."
        exit 1
    fi
    echo "      $INTERFACE activated successfully."
fi

# 2. Activate Python Environment
echo "[2/2] Activating Python environment..."

ENV_MODE="system"
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
    ENV_MODE="conda"
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

echo "========================================"
echo "          Setup Complete!               "
echo "========================================"

# 3. Execute payload
if [ $# -eq 0 ]; then
    # No arguments provided, spawn a new shell
    if [ "$ENV_MODE" = "conda" ]; then
        echo "Spawning a new shell with piper_env active..."
    else
        echo "Spawning a new shell using system Python..."
    fi
    echo "Type 'exit' to leave this session."
    exec bash
else
    # Run the provided command
    echo "Running command: $@"
    exec "$@"
fi
