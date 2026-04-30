# Robot + Aria + FLIR Runbook (All Terminals)

Use this as your copy-paste checklist during AV testing.

## Rules
- Run FLIR/ROS camera commands in a normal shell (do not activate `aria_env`).
- Run Aria recording commands in `/home/aryan/cartron` as usual.
- Keep laptop and robot on the same NTP source.

## -1) Aria SDK Setup (New Laptop, One-Time)

Run on the laptop that is connected to Aria glasses.

```bash
cd /home/aryan
git clone https://github.com/lalitaditya/cartron.git
cd cartron

python3 -m venv aria_env
source aria_env/bin/activate
python -m pip install --upgrade pip

# Core Aria + Python dependencies used by this repo
python -m pip install \
  projectaria-client-sdk==2.3.0 \
  projectaria-tools==2.1.2 \
  projectaria_eyetracking==1.0 \
  opencv-python \
  matplotlib \
  numpy \
  pandas \
  scipy \
  PyYAML
```

If `projectaria_eyetracking==1.0` is unavailable on your machine:

```bash
python -m pip install git+https://github.com/facebookresearch/projectaria_eyetracking.git
```

Quick import validation:

```bash
source /home/aryan/cartron/aria_env/bin/activate
python - <<'PY'
import aria.sdk as aria
from projectaria_tools.core import calibration
print("OK: aria.sdk and projectaria_tools imports work")
PY
```

Optional eye-gaze model validation:

```bash
source /home/aryan/cartron/aria_env/bin/activate
python - <<'PY'
from projectaria_eyetracking.inference.infer import EyeGazeInference
print("OK: projectaria_eyetracking import works")
PY
```

## 0) Time Sync Check (Both Machines)

Run on laptop and robot:

```bash
sudo chronyc tracking
sudo chronyc sources -v
date -u +"%Y-%m-%d %H:%M:%S.%N UTC"
```

Pass criteria:
- `Leap status: Normal`
- valid reference source on both machines
- ms-level offsets are fine

## 1) Laptop FLIR Network Prep (USB-Ethernet Adapter)

If DHCP is unavailable on PoE network, use static IP on laptop adapter:

```bash
ip -br link
sudo ip addr flush dev enx0050b6120b86
sudo ip addr add 192.168.50.20/24 dev enx0050b6120b86
sudo ip link set enx0050b6120b86 up
ip -br addr show enx0050b6120b86
```

If interface name differs, replace `enx0050b6120b86`.

## 2) Laptop FLIR Stream Validation (3 Terminals)

### Terminal L1 (camera driver)

```bash
source /opt/ros/humble/setup.bash
ros2 launch spinnaker_camera_driver driver_node.launch.py serial:="'24203528'"
```

### Terminal L2 (debayer for color view)

```bash
source /opt/ros/humble/setup.bash
ros2 run image_proc debayer_node --ros-args \
  -r image_raw:=/flir_camera/image_raw \
  -r image_color:=/flir_camera/image_color
```

### Terminal L3 (view + rate check)

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep flir_camera
ros2 topic hz /flir_camera/image_raw
rqt
```

In `rqt`:
- `Plugins` -> `Visualization` -> `Image View`
- choose `/flir_camera/image_color` for normal color image

## 3) Robot Camera + Bag Recording (Robot Computer)

### Terminal R1 (camera driver)

```bash
cd ~/sensor_ws
source install/setup.bash
ros2 launch spinnaker_camera_driver two_cameras.launch.py
```

### Terminal R2 (bag record)

```bash
source /opt/ros/humble/setup.bash
cd <robot_output_folder>
ros2 bag record -o case_bag_$(date +%Y%m%d_%H%M%S) /cam_0/image_raw /cam_0/camera_info
```

## 4) Aria Recording (Laptop)

### Terminal L4 (Aria)

```bash
cd /home/aryan/cartron
./record_case1_aria_only.sh -- --gaze-source open-model --gaze-model-device cpu
```

## 5) During Experiment

1. Do a visible sync cue: countdown `3,2,1` + hand cue/clap.
2. Run your motion/test scenario.
3. Stop Aria and ROS bag close in time.

## 6) Copy Bag to Laptop (if needed)

From laptop:

```bash
scp -r <robot_user>@<robot_ip>:<robot_bag_dir> /home/aryan/cartron/
```

## 7) Build Side-by-Side Review (Laptop)

```bash
python3 /home/aryan/cartron/aria_teleop/align_aria_and_rosbag.py \
  --stationary-review \
  --aria-csv <ARIA_SESSION_DIR>/gaze_stream.csv \
  --aria-video <ARIA_SESSION_DIR>/rgb_overlay.avi \
  --rosbag-dir <ROSBAG_DIR> \
  --output-dir <OUTPUT_DIR> \
  --transform-mode affine \
  --auto-affine-gaze-fit \
  --sync-basis epoch
```

Main output:

```text
<OUTPUT_DIR>/aria_ros_side_by_side_affine.mp4
```

## 8) Quick Troubleshooting

- `two_cameras.launch.py not found` on laptop:
  - use `driver_node.launch.py` (laptop apt package layout differs from robot workspace)
- `camera <serial> not found`, but another serial is shown:
  - launch with `serial:="'<detected_serial>'"`
- `showimage` crashes with unsupported encoding:
  - use `debayer_node` and view `/flir_camera/image_color`
- no DHCP on PoE network:
  - use static IP on adapter as in section 1
