# Cartron: Piper Robot Arm Controller

This repository contains the SDK, drivers, and custom control scripts for the Piper robot arm.
It is a repackaged version of the `piper_sdk` designed for ease of use with the **Cartron** project.

## 🚀 Quick Start

We provide a helper script to handle CAN initialization and environment activation.

### 1. Connect Hardware
Ensure the USB-to-CAN module is plugged into the computer and connected to the robot arm.

### 2. Run Quick Start
This script will activate the CAN interface (asking for `sudo` if needed) and start the Python environment.

```bash
./quick_start.sh
```

You are now in a shell with `piper_env` active and the hardware ready!

---

## 🛠️ Installation (First Time Only)

If you have not set up the environment yet:

1.  **System Dependencies**:
    ```bash
    sudo apt install can-utils ethtool
    ```

2.  **Conda Environment**:
    Inside this directory:
    ```bash
    conda create -n piper_env python=3.10
    conda activate piper_env
    pip install .
    ```

---

## 🎮 Demos

All demos are located in `piper_sdk/demo/V2/`.

### 👋 Wave & Say Hi
A custom script that makes the robot wave at you and speak.

**Run with Quick Start:**
```bash
./quick_start.sh python piper_sdk/demo/V2/piper_wave.py
```

**Run Manually (if env is active):**
```bash
python piper_sdk/demo/V2/piper_wave.py
```

### 🕹️ Joint Control
Moves the robot joints through a test pattern.

```bash
python piper_sdk/demo/V2/piper_ctrl_joint.py
```

---

## 📂 Project Structure

-   `quick_start.sh`: **Main entry point**. Sets up CAN and environment.
-   `piper_sdk/`: Core SDK Python package.
-   `piper_sdk/demo/V2/`:
    -   `piper_wave.py`: **Custom waving demo**.
    -   `piper_ctrl_enable.py`: Enable motors.
    -   `piper_read_*.py`: Read status scripts.

## 🔧 Troubleshooting

-   **CAN Error**: If you see `CAN port can0 is not UP`, simply run `./quick_start.sh` again.
-   **Permission Denied**: `quick_start.sh` and other scripts must be executable (`chmod +x script.sh`).

---

## 🤖 Simulation (RViz2)

An interactive 3D RViz2 simulation of the Piper arm is available in `piper_simulation/`.

### Prerequisites
```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

### Launch
```bash
source /opt/ros/humble/setup.bash
ros2 launch piper_simulation display.launch.py
```

Or run directly without building:
```bash
source /opt/ros/humble/setup.bash
ros2 launch /path/to/cartron/piper_simulation/launch/display.launch.py
```

Features:
-   URDF model built from real DH parameters
-   Joint sliders via `joint_state_publisher_gui`
-   All 6 joints with accurate limits
-   Gripper open/close control
-   TF tree visualization

---
*Based on AgileX Piper SDK.*
