import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "piper",
        package_name="piper_with_gripper_moveit",
    ).to_moveit_configs()

    servo_yaml = load_yaml(
        "piper_with_gripper_moveit",
        "config/servo/servo_parameters.yaml",
    )
    servo_params = {"moveit_servo": servo_yaml}

    servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    bridge = Node(
        package="piper",
        executable="trajectory_to_jointstate_bridge",
        name="servo_to_piper_joint_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": "/arm_controller/joint_trajectory",
                "output_topic": "/joint_ctrl_cmd",
            }
        ],
    )

    return LaunchDescription([servo, bridge])
