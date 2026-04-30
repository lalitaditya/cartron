# Cartron: Piper Robot Arm Controller

This repository contains the SDK, drivers, and custom control scripts for the Piper robot arm.
It is a repackaged version of the `piper_sdk` designed for ease of use with the **Cartron** project.

---

## 🥽 VR Teleoperation (Zero-Lag Pipeline)

Control the physical Piper robot arm using a VR headset over a dual-machine network setup.

### 💻 Windows Laptop (VR Machine)
Run these commands in three separate terminals:

1. **Start VR Controller Sync**:
   ```bash
   python vr_teleop\vr_udp_sender.py
   ```

2. **Start VR-to-Robot Bridge** (Optimization: Background threading & TCP_NODELAY):
   ```bash
   python vr_teleop\vr_piper_bridge.py --ros --grpc --grpc-host 10.131.184.135
   ```

3. **Start RViz Visualization** (Optional):
   ```bash
   ros2 launch vr_teleop\launch_piper_rviz.py
   ```

### 🐧 Linux Laptop (Arm Control Machine)
Run this command to start the state-based high-speed control server:

1. **Start Slave Controller**:
   ```bash
   python slave.py
   ```

---

## 🚀 Quick Start (Local Setup)

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

### 🖥️ Robot Control UI
A graphical interface to control the robot, view status, and manage settings.

**Run with Quick Start:**
```bash
./quick_start_ui.sh
```

**Features:**
-   **Connect**: Auto-detects CAN port.
-   **Control**: Joint sliders, gripper control.
-   **Status**: Real-time feedback (angles, current, errors).
-   **Settings**: Master/Slave mode, zero calibration.

👉 **[Full UI Documentation (Steps & Controls)](ui/README(EN).md)**

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
*Based on AgileX Piper SDK.*

---

## 🤖 ROS 2 & MoveIt Integration

This repository now includes **ROS 2 Humble** support with **MoveIt 2** for motion planning.

### 1. Build the Workspace

From the root of  (or your workspace):

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with colcon
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=$(which python3)
```

### 2. Running Real Robot (Driver + MoveIt)

You will need **Two Terminals**.

#### Terminal 1: Hardware Driver
Responsible for communicating with the arm via CAN.

```bash
cd cartron  # or workspace root
source /opt/ros/humble/setup.bash
source install/setup.bash

# Activate CAN (if not already done via quick_start.sh)
sudo ip link set can0 up type can bitrate 1000000

# Add piper_sdk to path (CRITICAL)
export PYTHONPATH=$PYTHONPATH:$(pwd)/piper_sdk

# Launch Driver
ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
```

#### Terminal 2: MoveIt Planning
Responsible for path planning and visualization.

```bash
cd cartron  # or workspace root
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch MoveIt
ros2 launch piper_with_gripper_moveit demo.launch.py
```

(Use  if you don't have a gripper).

### 3. Simulation Mode (No Robot)

If you don't have the hardware connected:

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```
*Note: In simulation mode, execution will only move the ghost robot in RViz.*

### 🔧 Troubleshooting

-   **Robot Not Moving?**
    Check if the robot is **Enabled**. The logs in Terminal 1 should say .
    To manually enable:
    ```bash
    ros2 topic pub --once /enable_flag std_msgs/msg/Bool "{data: true}"
    ```

-   **Dependencies Missing?**
    If  fails, ensure you have installed:
    ```bash
    sudo apt install ros-humble-moveit* ros-humble-ros2-control* ros-humble-ros2-controllers*
    ```

---

## 🕹️ Dual Arm Teleoperation (gRPC)

Scripts to control one arm (Slave) using another arm (Master) over the network.

### 1. Setup
- **Hardware**: Two Piper arms.
- **Connection**: Connect Master to `can0`, Slave to `can1` (or separate machines).
- **Dependencies**: `pip install grpcio grpcio-tools protobuf`

### 2. Run
**Step 1: Activate CAN interfaces**
```bash
./piper_teleop/setup_dual_can.sh
```

**Step 2: Start Slave (Controlled Arm)**
```bash
python3 piper_teleop/slave.py
```

**Step 3: Start Master (Controller Arm)**
```bash
python3 piper_teleop/master.py
```
*(Edit `master.py` to set `slave_ip` if using two different computers).*
