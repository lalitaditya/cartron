# MoveIt 2 Setup Guide for Piper Arm (WSL2)

Complete guide for setting up MoveIt 2 with the Piper robotic arm on WSL2 Ubuntu 22.04.

---

## Prerequisites

- Windows 11 with WSL2 enabled
- Ubuntu 22.04 installed in WSL2
- ROS 2 Humble installed in WSL2
- Piper arm URDF available at `C:\Users\aryan\cartron\src\piper_ros\src\piper_description\`

---

## 1. Install ROS 2 Humble in WSL2

```bash
# Open WSL2
wsl

# Check if ROS 2 is already installed
ros2 --help

# If not installed:
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y
```

## 2. Install MoveIt 2

```bash
sudo apt install ros-humble-moveit -y
sudo apt install ros-humble-moveit-visual-tools -y
sudo apt install ros-humble-moveit-ros-visualization -y
```

## 3. Create Workspace & Build Tutorials

```bash
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src

# Clone MoveIt Task Constructor and tutorials
git clone https://github.com/moveit/moveit2_tutorials.git -b humble
git clone https://github.com/moveit/moveit_task_constructor.git -b humble

# Install dependencies
sudo apt install python3-rosdep -y
sudo rosdep init   # skip if already done
rosdep update
cd ~/ws_moveit2
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 4. Add to bashrc

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc

# Fix for WSL2 GPU rendering issues (if RViz crashes)
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
source ~/.bashrc
```

## 5. Test with Panda Demo

```bash
ros2 launch moveit2_tutorials demo.launch.py
```

If RViz crashes with `D3D12: Removing Device` or `exit code -11`:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch moveit2_tutorials demo.launch.py
```

---

## 6. Copy Piper URDF into WSL2

```bash
# Create piper_description package
mkdir -p ~/ws_moveit2/src/piper_description/urdf
mkdir -p ~/ws_moveit2/src/piper_description/meshes

# Copy URDF and meshes from Windows
cp /mnt/c/Users/aryan/cartron/src/piper_ros/src/piper_description/urdf/piper_description.urdf \
  ~/ws_moveit2/src/piper_description/urdf/
cp /mnt/c/Users/aryan/cartron/src/piper_ros/src/piper_description/meshes/*.STL \
  ~/ws_moveit2/src/piper_description/meshes/

# Create package.xml
cat > ~/ws_moveit2/src/piper_description/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>piper_description</name>
  <version>1.0.0</version>
  <description>Piper robot description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

# Create CMakeLists.txt
cat > ~/ws_moveit2/src/piper_description/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.5)
project(piper_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
ament_package()
EOF

# Build
cd ~/ws_moveit2
colcon build --packages-select piper_description
source install/setup.bash
```

## 7. Generate Piper MoveIt Config (Setup Assistant)

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### Setup Assistant Steps:

1. **Start Screen** → "Create New MoveIt Configuration Package"
2. **Load URDF** → Browse to:
   `~/ws_moveit2/install/piper_description/share/piper_description/urdf/piper_description.urdf`
3. **Self-Collisions** → Click "Generate Collision Matrix"
4. **Planning Groups** → Add group:
   - Group Name: `piper_arm`
   - Kinematic Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
   - Group Default Planner: `RRTConnect`
   - Click "Add Kin. Chain" → Base Link: `base_link`, Tip Link: `link6`
5. **Robot Poses** → Add:
   - `home` (piper_arm): all joints = 0
   - `ready` (piper_arm): joint1=0, joint2=0.6, joint3=-0.8, joint4=0, joint5=-1.1, joint6=0
6. **Author Information** → Fill in name/email
7. **Configuration Files** → Save to: `/home/aryanj2003/ws_moveit2/src/piper_moveit_config`
   - Type the path directly in the text field
   - Click "Generate Package"

## 8. Manually Add Gripper Group to SRDF

The Setup Assistant may have issues adding prismatic gripper joints. Edit the SRDF directly:

```bash
nano ~/ws_moveit2/src/piper_moveit_config/config/piper.srdf
```

Replace the entire file with:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot name="piper" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
       xsi:noNamespaceSchemaLocation="https://raw.githubusercontent.com/ros/srdfdom/main/srdf.xsd">
    <group name="piper_arm">
        <chain base_link="base_link" tip_link="link6"/>
    </group>
    <group name="piper_gripper">
        <joint name="joint7"/>
        <joint name="joint8"/>
    </group>
    <end_effector name="piper_gripper" parent_link="link6"
                  group="piper_gripper" parent_group="piper_arm"/>
    <group_state name="home" group="piper_arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="0"/>
        <joint name="joint3" value="0"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="0"/>
        <joint name="joint6" value="0"/>
    </group_state>
    <group_state name="ready" group="piper_arm">
        <joint name="joint1" value="0"/>
        <joint name="joint2" value="0.6"/>
        <joint name="joint3" value="-0.8"/>
        <joint name="joint4" value="0"/>
        <joint name="joint5" value="-1.1"/>
        <joint name="joint6" value="0"/>
    </group_state>
    <group_state name="open" group="piper_gripper">
        <joint name="joint7" value="0.04"/>
        <joint name="joint8" value="0.04"/>
    </group_state>
    <group_state name="close" group="piper_gripper">
        <joint name="joint7" value="0.0"/>
        <joint name="joint8" value="0.0"/>
    </group_state>
    <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="link2" reason="Never"/>
    <disable_collisions link1="base_link" link2="link3" reason="Never"/>
    <disable_collisions link1="gripper_base" link2="link3" reason="Never"/>
    <disable_collisions link1="gripper_base" link2="link4" reason="Never"/>
    <disable_collisions link1="gripper_base" link2="link5" reason="Never"/>
    <disable_collisions link1="gripper_base" link2="link6" reason="Adjacent"/>
    <disable_collisions link1="gripper_base" link2="link7" reason="Adjacent"/>
    <disable_collisions link1="gripper_base" link2="link8" reason="Adjacent"/>
    <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
    <disable_collisions link1="link1" link2="link3" reason="Never"/>
    <disable_collisions link1="link1" link2="link4" reason="Never"/>
    <disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
    <disable_collisions link1="link2" link2="link4" reason="Never"/>
    <disable_collisions link1="link3" link2="link4" reason="Adjacent"/>
    <disable_collisions link1="link3" link2="link5" reason="Never"/>
    <disable_collisions link1="link3" link2="link6" reason="Never"/>
    <disable_collisions link1="link3" link2="link7" reason="Never"/>
    <disable_collisions link1="link3" link2="link8" reason="Never"/>
    <disable_collisions link1="link4" link2="link5" reason="Adjacent"/>
    <disable_collisions link1="link4" link2="link6" reason="Never"/>
    <disable_collisions link1="link4" link2="link7" reason="Never"/>
    <disable_collisions link1="link4" link2="link8" reason="Never"/>
    <disable_collisions link1="link5" link2="link6" reason="Adjacent"/>
    <disable_collisions link1="link5" link2="link7" reason="Never"/>
    <disable_collisions link1="link5" link2="link8" reason="Never"/>
    <disable_collisions link1="link6" link2="link7" reason="Never"/>
    <disable_collisions link1="link6" link2="link8" reason="Never"/>
    <disable_collisions link1="link7" link2="link8" reason="Default"/>
</robot>
```

## 9. Fix Config Files

### Fix joint_limits.yaml (integer → double)

```bash
cd ~/ws_moveit2/src/piper_moveit_config
sed -i 's/max_velocity: \([0-9]*\)$/max_velocity: \1.0/' config/joint_limits.yaml
sed -i 's/max_effort: \([0-9]*\)$/max_effort: \1.0/' config/joint_limits.yaml
sed -i 's/max_acceleration: \([0-9]*\)$/max_acceleration: \1.0/' config/joint_limits.yaml
```

### Fix moveit_controllers.yaml

```bash
nano ~/ws_moveit2/src/piper_moveit_config/config/moveit_controllers.yaml
```

Replace with:

```yaml
# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - piper_arm_controller

  piper_arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
```

## 10. Build & Launch Piper MoveIt Demo

```bash
cd ~/ws_moveit2
colcon build --packages-select piper_moveit_config
source install/setup.bash
ros2 launch piper_moveit_config demo.launch.py
```

### What to Test:

1. **Drag the interactive marker** to a new position
2. Click **"Plan"** → should compute a trajectory
3. Click **"Execute"** → arm moves to goal
4. Set **Goal State** to `ready` → **Plan & Execute**
5. Set **Goal State** to `home` → **Plan & Execute**

---

## 11. Add Collision Objects for Pick-and-Place Testing

Create the script:

```bash
mkdir -p ~/ws_moveit2/src/piper_moveit_config/scripts
nano ~/ws_moveit2/src/piper_moveit_config/scripts/pick_place_demo.py
```

```python
#!/usr/bin/env python3
"""Pick-and-place demo for Piper arm using MoveIt 2."""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        self.co_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)
        time.sleep(1.0)
        self.get_logger().info('Pick-Place Demo Ready!')

    def add_box(self, name, x, y, z, sx=0.04, sy=0.04, sz=0.04):
        co = CollisionObject()
        co.header.frame_id = 'base_link'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = name
        co.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        self.co_pub.publish(co)
        self.get_logger().info(f'Added box "{name}" at ({x},{y},{z})')


def main():
    rclpy.init()
    node = PickPlaceDemo()
    node.add_box('table', 0.3, 0.0, -0.025, 0.6, 0.6, 0.05)
    time.sleep(0.5)
    node.add_box('target_cube', 0.3, -0.1, 0.04, 0.04, 0.04, 0.04)
    print("\nObjects added! Drag the interactive marker near the cube.")
    print("Plan & Execute to test collision avoidance.")
    print("Press Ctrl+C to exit.\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run (in a second terminal while demo.launch.py is running):

```bash
source ~/ws_moveit2/install/setup.bash
python3 ~/ws_moveit2/src/piper_moveit_config/scripts/pick_place_demo.py
```

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| RViz crashes with `D3D12: Removing Device` | `export LIBGL_ALWAYS_SOFTWARE=1` |
| `exit code -11` (segfault) | Use software rendering (above) or update GPU drivers |
| `expected [double] got [integer]` | Run the sed commands in Step 9 to add `.0` to numeric values |
| `Group cannot be its own parent` | Remove malformed end_effector from SRDF, use the SRDF in Step 8 |
| `No action namespace specified` | Add `action_ns: follow_joint_trajectory` to moveit_controllers.yaml |
| `Unable to identify controllers` | Fix moveit_controllers.yaml per Step 9 |
| `Missing joint7, joint8` | Expected warning — gripper joints not in ros2_control (harmless) |
| `load_yaml() deprecated` | Harmless warning, ignore |

---

## Next Steps

- [ ] Set up MoveIt Task Constructor (MTC) for full pick-and-place
- [ ] Create Piper-specific MTC config (`piper_config.yaml`)
- [ ] Integrate with physical Piper arm via ROS 2 DDS bridge (WSL2 ↔ Windows)
- [ ] Add depth camera perception for autonomous grasping
