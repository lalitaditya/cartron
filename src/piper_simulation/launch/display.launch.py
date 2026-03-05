"""
Launch file for Piper Arm RViz2 simulation.
Starts: robot_state_publisher, joint_state_publisher_gui, rviz2
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Try to locate via ament, fall back to local path
    try:
        pkg_share = get_package_share_directory('piper_simulation')
        urdf_file = os.path.join(pkg_share, 'urdf', 'piper_arm.urdf')
        rviz_config = os.path.join(pkg_share, 'rviz', 'piper_config.rviz')
    except Exception:
        # If package is not installed, use paths relative to this file
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        urdf_file = os.path.join(pkg_dir, 'urdf', 'piper_arm.urdf')
        rviz_config = os.path.join(pkg_dir, 'rviz', 'piper_config.rviz')

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot State Publisher — publishes TF from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Joint State Publisher GUI — slider panel to control joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2 — 3D visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
