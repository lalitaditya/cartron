import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Paths relative to this file's location inside cartron/vr_teleop/
    this_dir = os.path.dirname(os.path.abspath(__file__))
    cartron_root = os.path.dirname(this_dir)  # cartron/
    base_path = os.path.join(cartron_root, 'src', 'piper_ros', 'src', 'piper_description')
    model_path = os.path.join(base_path, 'urdf', 'piper_description.urdf')
    rviz_config_path = os.path.join(base_path, 'rviz', 'piper_ctrl.rviz')

    print(f"--- Loading Piper Model from: {model_path} ---")

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    
    # Read and patch the URDF to use absolute paths for meshes
    with open(model_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Convert C:\Paths to file:///C:/Paths for RViz
    abs_uri = 'file:///' + base_path.replace('\\', '/') + '/'
    content = content.replace('package://piper_description/', abs_uri)
    
    # Append a static test cube to the URDF for grasping trials
    test_cube_urdf = """
  <link name="test_cube">
    <visual>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
      <material name="green"><color rgba="0 1 0 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.05"/></geometry>
    </collision>
  </link>
  <joint name="test_cube_joint" type="fixed">
    <parent link="base_link"/>
    <child link="test_cube"/>
    <origin xyz="0.40 -0.10 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Intel RealSense D435 wrist camera (90x25x25mm) -->
  <link name="wrist_camera_link">
    <visual>
      <geometry><box size="0.025 0.09 0.025"/></geometry>
      <material name="dark_gray"><color rgba="0.15 0.15 0.15 1"/></material>
    </visual>
  </link>
  <joint name="wrist_camera_joint" type="fixed">
    <parent link="gripper_base"/>
    <child link="wrist_camera_link"/>
    <!-- Mounted on top of gripper base, tilted down ~30° to view workspace -->
    <origin xyz="0 0.04 0.02" rpy="0 -0.5 0"/>
  </joint>

  <!-- Optical frame: Z-forward, X-right, Y-down (standard camera convention) -->
  <!-- gripper_base has Z along gripper (forward), Y left, X up -->
  <!-- So: optical Z = link Z, optical X = link -Y, optical Y = link -X -->
  <link name="camera_optical_frame"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="wrist_camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 0"/>
  </joint>
</robot>"""
    content = content.replace('</robot>', test_cube_urdf)
    
    robot_description = ParameterValue(content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        rviz_node
    ])
