import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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

    # 5. Configurazione OMPL Pipeline (Senza Ruckig per evitare l'errore -100)
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

    # 6. CONFIGURAZIONE FAKE NATIIVA (Funzionante con ros-humble-moveit-plugins)
    fake_controllers_param = {
        'moveit_controller_manager': 'moveit_fake_controller_manager/MoveItFakeControllerManager',
        'fake_initial_joints': {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': -1.5708,
            'elbow_joint': 0.1,
            'wrist_1_joint': -1.5708,
            'wrist_2_joint': 0.0,
            'wrist_3_joint': 0.0,
            'gripper_gripper_joint': 0.0
        },
        'moveit_fake_controller_manager': {
            'controller_names': ['ur_manipulator_controller', 'gripper_controller'],
            'ur_manipulator_controller': {
                'type': 'fake',
                'joints': [
                    'shoulder_pan_joint',
                    'shoulder_lift_joint',
                    'elbow_joint',
                    'wrist_1_joint',
                    'wrist_2_joint',
                    'wrist_3_joint'
                ]
            },
            'gripper_controller': {
                'type': 'fake',
                'joints': [
                    'gripper_gripper_joint',
                    'gripper_right_finger_joint'
                ]
            }
        },
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'moveit_manage_controllers': True
    }

    # --- NODI ---

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['/move_group/fake_controller_joint_states'],
            'zeros': {
                'shoulder_pan_joint': 0.0,
                'shoulder_lift_joint': -1.5708,
                'elbow_joint': 0.1, 
                'wrist_1_joint': -1.5708,
                'wrist_2_joint': 0.0,
                'wrist_3_joint': 0.0
            }
        }]
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    object_rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="object_state_publisher",
        output='screen',
        parameters=[{"robot_description": object_desc}],
        remappings=[('/robot_description', '/object_description')]
    )

    robot_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]
    )

    object_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.5", "0.0", "0.0", "0.0", "0.0", "world", "object_link"]
    )

    # Il Cervello di MoveIt (Aggiornato con parametri forzati in linea)
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
            fake_controllers_param,
            {"publish_monitored_planning_scene": True},
            {"use_sim_time": False}
        ]
    )

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
        joint_state_publisher_node,
        rsp_node,
        object_rsp_node,
        robot_to_world_tf,
        object_to_world_tf,
        move_group_node,
        rviz_node
    ])