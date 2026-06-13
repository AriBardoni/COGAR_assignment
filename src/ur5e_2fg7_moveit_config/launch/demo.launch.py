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

    # 1. Caricamento Robot Description (URDF via Xacro)
    xacro_file = os.path.join(desc_pkg, 'urdf', 'ur5e_2fg7_main.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # 2. Caricamento dell'URDF del Cilindro (Target Object)
    object_urdf_file = os.path.join(desc_pkg, 'urdf', 'target_object.urdf')
    with open(object_urdf_file, 'r') as infp:
        object_desc = infp.read()

    # 3. Configurazione Semantica di MoveIt (SRDF)
    srdf_file = os.path.join(moveit_pkg, 'config', 'ur5e_con_2fg7.srdf')
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 4. Parametri di cinematica e limiti di giunto
    kinematics_file = os.path.join(moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_file = os.path.join(moveit_pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # 5. Configurazione OMPL Pipeline
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
            'start_state_max_bounds_error': 0.1,
        }
    }

    # 6. Configurazione Controller Manager (MoveIt -> Simple Controller Manager)
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

    # File dei controller per ros2_control
    ros2_controllers_file = os.path.join(moveit_pkg, 'config', 'ros2_controllers.yaml')

    # --- NODI ---

    # ros2_control_node: il nodo che gestisce l'hardware (mock) e i controller
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_file],
        output='screen',
    )

    # Spawner per joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner per il controller del braccio
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_manipulator_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner per il controller della pinza
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Pubblica i link del robot
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Pubblica la struttura del cilindro
    object_rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="object_state_publisher",
        output="screen",
        parameters=[{"robot_description": object_desc}],
        remappings=[('/robot_description', '/object_description')]
    )

    # Ancoraggio al mondo
    robot_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]
    )

    # Cilindro davanti (Y = 0.5 positivo)
    object_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.5", "0.0", "0.0", "0.0", "0.0", "world", "object_link"]
    )

    # Il Cervello di MoveIt
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

    # RViz2
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
        object_to_world_tf,
        joint_state_broadcaster_spawner,
        # Avvia gli spawner del braccio e della pinza solo dopo il joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        move_group_node,
        rviz_node,
    ])