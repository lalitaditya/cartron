#!/bin/bash

# Robot Arm Quick Start Script
# Usage: 
#   ./quick_start.sh             -> Sets up CAN and drops you into a shell with piper_env active
#   ./quick_start.sh <command>   -> Sets up CAN and runs the specific command

# Path to the repo root
REPO_ROOT="/home/networking/cartron/piper_sdk"

echo "========================================"
echo "      Piper Robot Arm Quick Start       "
echo "========================================"

# 1. Configure CAN Interface
INTERFACE="can0"
BAUDRATE="1000000"

echo "[1/2] Checking CAN interface '$INTERFACE'..."

if ! ip link show "$INTERFACE" > /dev/null 2>&1; then
    echo "      Interface $INTERFACE not found! Checking connected USB devices..."
    bash "$REPO_ROOT/find_all_can_port.sh"
    echo "      Please ensure the USB-to-CAN module is plugged in."
    exit 1
fi

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

# 2. Activate Conda Environment
echo "[2/2] Activating 'piper_env' environment..."

# Dynamically find conda base path
CONDA_BASE=$(conda info --base 2>/dev/null)

if [ -z "$CONDA_BASE" ]; then
    echo "      Conda not found in PATH."
    echo "      Please ensure conda is installed and initialized."
    exit 1
fi

CONDA_SH="$CONDA_BASE/etc/profile.d/conda.sh"

if [ -f "$CONDA_SH" ]; then
    source "$CONDA_SH"
else
    echo "      Could not find conda.sh at $CONDA_SH"
    exit 1
fi

conda activate piper_env

if [ $? -ne 0 ]; then
    echo "      Failed to activate piper_env."
    exit 1
fi

echo "      Environment active."
echo "========================================"
echo "          Setup Complete!               "
echo "========================================"

# 3. Execute payload
if [ $# -eq 0 ]; then
    echo "Spawning a new shell with piper_env active..."
    echo "Type 'exit' to leave this session."
    exec bash --rcfile <(echo ". ~/.bashrc; source \"$CONDA_SH\"; conda activate piper_env")
else
    echo "Running command: $@"
    exec "$@"
fi
