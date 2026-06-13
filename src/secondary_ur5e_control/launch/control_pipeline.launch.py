import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    pkg_ur5e_2fg7 = get_package_share_directory('ur5e_2fg7')
    
    # 1. Read the robot URDF file
    urdf_file = os.path.join(pkg_ur5e_2fg7, 'urdf', 'ur5e_2fg7_final.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # 2. Read the robot semantic description (SRDF)
    srdf_file = os.path.join(pkg_ur5e_2fg7, 'config', 'ur5e_2fg7.srdf')
    with open(srdf_file, 'r') as infp:
        robot_desc_semantic = infp.read()
        
    # 3. Read the kinematics YAML file directly using standard yaml parser
    kinematics_file = os.path.join(pkg_ur5e_2fg7, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as infp:
        kinematics_yaml = yaml.safe_load(infp)
        
    # Combine parameters for MoveIt 2
    moveit_parameters = {
        "robot_description": robot_desc,
        "robot_description_semantic": robot_desc_semantic,
        "robot_description_kinematics": kinematics_yaml
    }

    return LaunchDescription([
        Node(
            package="secondary_ur5e_control",
            executable="pick_and_place_node",
            name="pick_and_place_node",
            output="screen",
            parameters=[moveit_parameters]
        )
    ])