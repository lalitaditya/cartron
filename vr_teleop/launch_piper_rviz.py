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
