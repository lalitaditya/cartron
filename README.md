# Cartron: Piper Robot Arm Controller

This repository contains the SDK, drivers, and custom control scripts for the Piper robot arm.
It is a repackaged version of the `piper_sdk` designed for ease of use with the **Cartron** project.

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

### 🤖 ROS 2 Driver
Builds the ROS workspace if needed, activates CAN, sources the local ROS install, and launches the Piper driver.

```bash
./quick_start_ros.sh
```

To pass ROS launch arguments:

```bash
./quick_start_ros.sh log_level:=warn auto_enable:=false
```

### Real-Time Low-Level Teleop Through Bridge

The low-level teleop scripts are organized here:

```text
piper_teleop/High-Level control/smooth_joint_trajectory.py
piper_teleop/Low level control/bridge/trajectory_to_jointstate_bridge.py
piper_teleop/Low level control/keyboard_teleop/realtime_joint_teleop.py
piper_teleop/Low level control/keyboard_teleop/publish_joint_command.py
piper_teleop/Low level control/vr_teleop/joy_joint_teleop.py
```

Use these three terminals for keyboard real-time control:

Terminal 1, hardware driver:

```bash
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=/home/tejaszz12/cartron:$PYTHONPATH
ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
```

Terminal 2, trajectory bridge:

```bash
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 "/home/tejaszz12/cartron/piper_teleop/Low level control/bridge/trajectory_to_jointstate_bridge.py"
```

Terminal 3, keyboard teleop:

```bash
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 "/home/tejaszz12/cartron/piper_teleop/Low level control/keyboard_teleop/realtime_joint_teleop.py" --step 0.01 --speed 10
```

Topic flow:

```text
keyboard/VR teleop -> /arm_controller/joint_trajectory
trajectory bridge  -> /joint_ctrl_cmd
hardware driver    -> CAN
```

For joystick control, start a `/joy` publisher and replace Terminal 3 with:

```bash
python3 "/home/tejaszz12/cartron/piper_teleop/Low level control/vr_teleop/joy_joint_teleop.py" --joint-speed 0.15 --speed-percent 10
```

### Perception Camera Check

The OAK/DepthAI USB camera should appear in `lsusb` as:

```text
03e7:2485 Intel Movidius MyriadX
```

If DepthAI reports `Insufficient permissions to communicate with X_LINK_UNBOOTED`, install the udev rule once:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the camera. Test RGB capture:

```bash
cd /home/tejaszz12/cartron
python3 piper_teleop/Perception/depthai_health_check.py
```

Open a live RGB camera window:

```bash
cd /home/tejaszz12/cartron
python3 piper_teleop/Perception/depthai_health_check.py --show
```

Press `q` or `Esc` to close the window.

---

## 📂 Project Structure

-   `quick_start.sh`: **Main entry point**. Sets up CAN and environment.
-   `quick_start_ros.sh`: Builds/sources the ROS 2 workspace and launches the Piper driver.
-   `piper_sdk/`: Core SDK Python package.
-   `piper_sdk/demo/V2/`:
    -   `piper_wave.py`: **Custom waving demo**.
    -   `piper_ctrl_enable.py`: Enable motors.
    -   `piper_read_*.py`: Read status scripts.

## 🔧 Troubleshooting

-   **CAN Error**: If you see `CAN port can0 is not UP`, simply run `./quick_start.sh` again.
-   **Permission Denied**: `quick_start.sh` and other scripts must be executable (`chmod +x script.sh`).

### CAN Debug Quick Checklist (Easy to Find)

If the arm node starts but logs errors like:
- `can0 has no CAN feedback yet; waiting...`
- `SendCanMessage(SEND_MESSAGE_FAILED)`
- `Failed to transmit: No buffer space available`

run this sequence:

1. Stop ROS/CAN users first:
   ```bash
   pkill -f piper_single_ctrl || true
   pkill -f ros2 || true
   pkill -f candump || true
   ```

2. Bring CAN up:
   ```bash
   cd /home/tejaszz12/cartron/src/piper_ros
   bash can_activate.sh can0 1000000
   ip -details -statistics link show can0
   ```

3. Monitor raw bus:
   ```bash
   candump -tz can0
   ```

4. If `can0` is still down, do a manual reset:
   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000 restart-ms 100
   sudo ip link set can0 up
   ip -details -statistics link show can0
   ```

5. In another terminal, start the driver:
   ```bash
   cd /home/tejaszz12/cartron/src/piper_ros
   source /opt/ros/humble/setup.bash
   source /home/tejaszz12/cartron/src/piper_ros/install/setup.bash
   bash can_activate.sh can0 1000000
   ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2 auto_enable:=false
   ```

6. Interpret quickly:
   - If `candump` shows frames but ROS topics stay zeros, check node parsing/config.
   - If `candump` shows no frames and `RX` stays `0`, this is bus/hardware side (power, CANH/CANL, GND, termination, pinout).
   - If `candump` says `read: Network is down`, bring `can0` up first (step 1).

---
*Based on AgileX Piper SDK.*

---

## 🤖 ROS 2 & MoveIt Integration

This repository now includes **ROS 2 Humble** support with **MoveIt 2** for motion planning.

### 1. Build the Workspace

From the ROS workspace:

```bash
cd /home/tejaszz12/cartron/src/piper_ros

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
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

# Activate CAN (if not already done via quick_start.sh)
sudo ip link set can0 up type can bitrate 1000000

# Add piper_sdk to path (CRITICAL)
export PYTHONPATH=/home/tejaszz12/cartron:$PYTHONPATH

# Launch Driver
ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
```

#### Terminal 1.5: Enable the Real Arm (Easy to Miss)
If the physical arm does not move, enable it explicitly:

```bash
source /opt/ros/humble/setup.bash
source /home/tejaszz12/cartron/src/piper_ros/install/setup.bash
ros2 service call /enable_srv piper_msgs/srv/Enable "{enable_request: true}"
```

#### Terminal 2: MoveIt Planning
Responsible for path planning and visualization.

```bash
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch MoveIt
ros2 launch piper_with_gripper_moveit demo.launch.py
```

(Use `piper_no_gripper_moveit` if you don't have a gripper).

Or use the helper from the repository root:

```bash
./quick_start_moveit.sh
```

### 3. MoveIt Servo Twist Control

MoveIt Servo is the programmatic real-time control path. It accepts Cartesian velocity commands as `geometry_msgs/msg/TwistStamped`, converts them to joint trajectory commands, and sends them to the arm controller.

Command path:

```text
TwistStamped publisher
  -> /servo_node/delta_twist_cmds
  -> MoveIt Servo
  -> /arm_controller/joint_trajectory
  -> ros2_control arm_controller
```

Start Servo after the robot driver and controllers are running:

```bash
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper_with_gripper_moveit servo.launch.py
```

Send a small test command from another terminal:

```bash
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run piper_with_gripper_moveit publish_servo_twist.py --linear-x 0.01 --duration 1.0
```

The Servo parameters live in:

```text
src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/servo/servo_parameters.yaml
```

### 4. AprilTag 10 cm Standoff Test

This runs the external DepthAI/OAK camera AprilTag detector, transforms the detected tag center into the robot base frame, and asks MoveIt to move the gripper tip to a fixed standoff above the tag. Start with `--plan-only`; use `--enable` only after the plan and CAN feedback look healthy.

Terminal 1, Piper hardware driver:

```bash
cd /home/tejaszz12/cartron
bash can_activate.sh can0 1000000 "" 10000

cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=/home/tejaszz12/cartron:$PYTHONPATH

ros2 launch piper start_single_piper.launch.py \
  gripper_val_mutiple:=2 \
  auto_enable_timeout:=20.0 \
  enable_service_timeout:=20.0
```

Leave this terminal running. Wait for `Enable status: True`; if it later prints `can0 has no CAN feedback yet`, fix CAN before continuing.

Terminal 2, trajectory bridge:

```bash
cd /home/tejaszz12/cartron
source /opt/ros/humble/setup.bash && source src/piper_ros/install/setup.bash

python3 "piper_teleop/Low level control/bridge/trajectory_to_jointstate_bridge.py" \
  --ros-args \
  -p max_publish_rate:=15.0
```

Terminal 3, MoveIt:

```bash
cd /home/tejaszz12/cartron/src/piper_ros
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
```

Terminal 4, AprilTag standoff planner:

```bash
cd /home/tejaszz12/cartron
source /opt/ros/humble/setup.bash && source src/piper_ros/install/setup.bash
export PYTHONPATH=/home/tejaszz12/cartron:$PYTHONPATH

python3 piper_teleop/Planning/apriltag_grasp_demo.py \
  --dictionary DICT_APRILTAG_36h11 \
  --tag-size-m 0.08 \
  --depth-source stereo \
  --stereo-preset FAST_DENSITY \
  --frame-color-mode bgr \
  --fx 514.80883789 \
  --fy 514.80883789 \
  --cx 323.42358398 \
  --cy 244.64537048 \
  --standoff-m 0.10 \
  --camera-x YOUR_CAMERA_X \
  --camera-y YOUR_CAMERA_Y \
  --camera-z YOUR_CAMERA_Z \
  --camera-roll YOUR_CAMERA_ROLL \
  --camera-pitch YOUR_CAMERA_PITCH \
  --camera-yaw YOUR_CAMERA_YAW \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03 \
  --stable-samples 5 \
  --stable-position-tolerance 0.015 \
  --gripper-backend none \
  --plan-only
```

Replace the `YOUR_CAMERA_*` values with the output from the calibration command below. Once `--plan-only` succeeds and the target is plausible, rerun Terminal 4 with `--enable` instead of `--plan-only`.

One-shot manual calibration for `YOUR_CAMERA_*`:

```bash
cd /home/tejaszz12/cartron
source /opt/ros/humble/setup.bash && source src/piper_ros/install/setup.bash

python3 piper_teleop/Perception/apriltag_point_pair_calibrator.py \
  --dictionary DICT_APRILTAG_36h11 \
  --frame-color-mode bgr \
  --base-source manual \
  --depth-source stereo \
  --stereo-preset FAST_DENSITY \
  --fx 514.80883789 \
  --fy 514.80883789 \
  --cx 323.42358398 \
  --cy 244.64537048 \
  --solve-mode fixed_rpy \
  --solve-after 1 \
  --last-tag-timeout 0
```

When the tag is detected, type the measured tag-center position in the robot base frame as `x y z` in meters. This one-shot mode only solves translation while assuming fixed camera RPY; if the camera is tilted differently, use multi-point calibration or closed-loop visual servoing instead.

### 5. Simulation Mode (No Robot)

If you don't have the hardware connected:

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```
*Note: In simulation mode, execution will only move the ghost robot in RViz.*

### 🔧 Troubleshooting

-   **Robot Not Moving?**
    Check if the robot is **Enabled**. To manually enable:
    ```bash
    source /opt/ros/humble/setup.bash
    source /home/tejaszz12/cartron/src/piper_ros/install/setup.bash
    ros2 service call /enable_srv piper_msgs/srv/Enable "{enable_request: true}"
    ```

-   **Dependencies Missing?**
    If  fails, ensure you have installed:
    ```bash
    sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
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
