import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    desc_pkg = get_package_share_directory('ur5e_2fg7')
    moveit_pkg = get_package_share_directory('ur5e_2fg7_moveit_config')

    # 1. Load the URDF (Robot Description)
    xacro_file = os.path.join(desc_pkg, 'urdf', 'ur5e_2fg7_main.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # 2. Load the SRDF (Semantic Description)
    srdf_file = os.path.join(moveit_pkg, 'config', 'ur5e_con_2fg7.srdf')
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 3. Load kinematics parameters
    kinematics_file = os.path.join(moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    pipeline_node = Node(
        package='ur5e_2fg7',
        executable='pick_place_pipeline_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([pipeline_node])