# Laptop FLIR Stream Test (USB-to-Ethernet Adapter)

This is the exact command flow to test FLIR camera streaming on this laptop through the PoE switch using the USB-to-Ethernet adapter.

Use a **system terminal** (do **not** activate `aria_env`).

---

## Terminal 0: One-Time Setup / Quick Checks

```bash
source /opt/ros/humble/setup.bash
```

Check adapter is detected:

```bash
lsusb | grep -Ei "ether|lan|realtek|asix"
ip -br link
nmcli device status
```

Expected adapter on this laptop:
- interface: `enx0050b6120b86`

---

## Terminal 0: Bring Up Adapter Network

### A) Try DHCP first

```bash
sudo nmcli device connect enx0050b6120b86
sudo dhclient enx0050b6120b86
ip -br addr show enx0050b6120b86
ip route
```

### B) If DHCP fails (common on PoE camera network), use static IP

```bash
sudo ip addr flush dev enx0050b6120b86
sudo ip addr add 192.168.50.20/24 dev enx0050b6120b86
sudo ip link set enx0050b6120b86 up
ip -br addr show enx0050b6120b86
```

Optional traffic counter check:

```bash
watch -n1 "cat /sys/class/net/enx0050b6120b86/statistics/rx_bytes; cat /sys/class/net/enx0050b6120b86/statistics/tx_bytes"
```

---

## Terminal 1: Launch FLIR Driver

Use detected serial number `24203528`:

```bash
source /opt/ros/humble/setup.bash
ros2 launch spinnaker_camera_driver driver_node.launch.py serial:="'24203528'"
```

---

## Terminal 2: Verify ROS Topics + Frame Rate

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep camera
ros2 topic hz /flir_camera/image_raw
```

---

## Terminal 3: Debayer to Normal Color Topic

(`image_raw` may be Bayer and not directly viewable as normal RGB)

```bash
source /opt/ros/humble/setup.bash
ros2 run image_proc debayer_node --ros-args \
  -r image_raw:=/flir_camera/image_raw \
  -r image_color:=/flir_camera/image_color
```

---

## Terminal 4: View Color Stream

Option 1 (rqt):

```bash
source /opt/ros/humble/setup.bash
rqt
```

Then:
- `Plugins` -> `Visualization` -> `Image View`
- Select `/flir_camera/image_color`

Option 2 (CLI viewer):

```bash
source /opt/ros/humble/setup.bash
ros2 run image_tools showimage --ros-args -r image:=/flir_camera/image_color
```

---

## Optional: SpinView Launch

```bash
/opt/spinnaker/bin/spinview
```

If needed:

```bash
/opt/spinnaker/bin/SpinView_QT
```

---

## Quick Stop

In each terminal, press `Ctrl+C`.

---

## Notes

- Calibration-file warnings are normal for first run.
- If `/flir_camera/image_raw` viewer crashes with unsupported encoding, always view `/flir_camera/image_color` after running `debayer_node`.
