import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('ur5e_2fg7')
    moveit_pkg = get_package_share_directory('ur5e_2fg7_moveit_config')

    # 1. Load Robot Description (URDF via Xacro)
    xacro_file = os.path.join(desc_pkg, 'urdf', 'ur5e_2fg7_main.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # 2. Load Cylinder URDF (Target Object)
    object_urdf_file = os.path.join(desc_pkg, 'urdf', 'target_object.urdf')
    with open(object_urdf_file, 'r') as infp:
        object_desc = infp.read()

    # 3. MoveIt Semantic Configuration (SRDF)
    srdf_file = os.path.join(moveit_pkg, 'config', 'ur5e_con_2fg7.srdf')
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 4. Kinematics parameters and joint limits
    kinematics_file = os.path.join(moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_file = os.path.join(moveit_pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # 5. OMPL Pipeline Configuration
    planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join([
                'default_planner_request_adapters/ResolveConstraintFrames',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints',
                'default_planner_request_adapters/AddTimeOptimalParameterization',
            ]),
            'simplify_solutions': True,
            'minimum_waypoint_count': 24, # Più waypoint = traiettorie ricampionate molto più morbide
            'start_state_max_bounds_error': 0.1,
            'planner_configs': {
                'RRTConnect': {'range': 0.02}, # Ridotto drasticamente da 0.2 a 0.02 per passi controllati
                'TRRT': {'range': 0.02, 'goal_bias': 0.05},
                'RRTstar': {'range': 0.02, 'goal_bias': 0.05}
            },
        }
    }

    # 6. Controller Manager Configuration (MoveIt -> Simple Controller Manager)
    moveit_controllers_config = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['ur_manipulator_controller', 'gripper_controller'],
            'ur_manipulator_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': [
                    'shoulder_pan_joint',
                    'shoulder_lift_joint',
                    'elbow_joint',
                    'wrist_1_joint',
                    'wrist_2_joint',
                    'wrist_3_joint',
                ],
            },
            'gripper_controller': {
                'action_ns': 'gripper_cmd',
                'type': 'GripperCommand',
                'default': True,
                'joints': [
                    'gripper_gripper_joint',
                ],
            },
        },
    }

    trajectory_execution_config = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Controller configuration file for ros2_control
    ros2_controllers_file = os.path.join(moveit_pkg, 'config', 'ros2_controllers.yaml')

    # NODES

    # ros2_control_node: Handles the hardware interface (mock) and active controllers
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_file],
        output='screen',
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner for the arm controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_manipulator_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner for the gripper controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Publish robot transforms and links
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Publish target cylinder transforms and links
    object_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='object_state_publisher',
        output='screen',
        parameters=[{"robot_description": object_desc}],
        remappings=[('/robot_description', '/object_description')]
    )

    # Anchor the robot base to the world frame
    robot_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # MoveIt MoveGroup Node (The main planning pipeline)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution_config,
            moveit_controllers_config,
            {'publish_monitored_planning_scene': True, 'use_sim_time': False},
        ]
    )

    # RViz2 Visualizer
    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ]
    )

    return LaunchDescription([
        ros2_control_node,
        rsp_node,
        object_rsp_node,
        robot_to_world_tf,
        joint_state_broadcaster_spawner,
        # Start arm and gripper spawners only after the joint_state_broadcaster exits successfully
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        move_group_node,
        rviz_node,
    ])