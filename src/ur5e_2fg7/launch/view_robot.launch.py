import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ur5e_2fg7 = get_package_share_directory('ur5e_2fg7')
    
    # Read robot URDF file 
    urdf_file = os.path.join(pkg_ur5e_2fg7, 'urdf', 'ur5e_2fg7_final.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Read target object URDF file 
    object_urdf_file = os.path.join(pkg_ur5e_2fg7, 'urdf', 'target_object.urdf')
    with open(object_urdf_file, 'r') as infp:
        object_desc = infp.read()
    
    rviz_config_file = os.path.join(pkg_ur5e_2fg7, 'config', 'view_robot.rviz')

    return LaunchDescription([
        # Node 1: Robot State Publisher 
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}]
        ),
        
        # Static TF to anchor the robot base link to the global world frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="robot_to_world_tf_publisher",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
        ),
        
        # State publisher for target object 
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="object_state_publisher",
            output="screen",
            parameters=[{"robot_description": object_desc}],
            remappings=[('/robot_description', '/object_description')]
        ),
        
        # Static TF to position object with respect to the 'world'
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="object_tf_publisher",
            arguments=["0.4", "0.0", "0.05", "0.0", "0.0", "0.0", "world", "object_link"]
        ),
        
        # Node 2: Joint State Publisher GUI 
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{
                'zeros': {
                    'shoulder_pan_joint': 0.0,
                    'shoulder_lift_joint': -1.5708,
                    'elbow_joint': 0.0,
                    'wrist_1_joint': -1.5708,
                    'wrist_2_joint': 0.0,
                    'wrist_3_joint': 0.0,
                    'gripper_gripper_joint': 0.000
                }
            }]
        ),
        
        # Node 3: RViz2 
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', rviz_config_file]
        )
    ])