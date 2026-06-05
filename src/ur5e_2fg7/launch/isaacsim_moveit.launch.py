import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Percorsi ai pacchetti
    desc_pkg = get_package_share_directory('ur5e_2fg7')
    moveit_pkg = get_package_share_directory('config_moveit')

    # 2. Carica l'URDF
    xacro_file = os.path.join(desc_pkg, 'urdf', 'ur5e_2fg7_main.urdf.xacro')
    initial_positions_file = os.path.join(desc_pkg, 'config', 'initial_positions.yaml')
    robot_description_content = Command(['xacro ', xacro_file, ' initial_positions_file:=', initial_positions_file])
    robot_description = {'robot_description': robot_description_content}

    # 3. Carica l'SRDF (le regole di collisione generate dal Setup Assistant)
    srdf_file = os.path.join(moveit_pkg, 'config', 'ur5e_con_2fg7.srdf') 
    with open(srdf_file, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_content}

    # 4. Carica la configurazione cinematica
    kinematics_file = os.path.join(moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_file = os.path.join(moveit_pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}
    
    # 5. I file dei controller che abbiamo scritto noi!
    ros2_controllers_file = os.path.join(desc_pkg, 'config', 'ros2_controllers.yaml')
    
    moveit_controllers_file = os.path.join(desc_pkg, 'config', 'moveit_controllers.yaml')
    with open(moveit_controllers_file, 'r') as f:
        moveit_controllers_yaml = yaml.safe_load(f)

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
                'default_planner_request_adapters/AddRuckigTrajectorySmoothing',
            ]),
            'start_state_max_bounds_error': 0.1,
        }
    }

    # --- NODI DA AVVIARE ---

    # A. Robot State Publisher (fondamentale per le trasformate 3D)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # B. Move Group (Il "cervello" di MoveIt)
    joint_limits_file = os.path.join(moveit_pkg, 'config', 'joint_limits.yaml')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,   # <-- dizionario Python puro
            moveit_controllers_yaml,
            planning_pipeline_config,
            {'use_sim_time': True},
            {'trajectory_execution.allowed_start_tolerance': 0.0}
        ]
    )

    # C. RViz2 (L'interfaccia grafica)
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
            {'use_sim_time': True}
        ]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_file,
            {'use_sim_time': True}
        ],
        output="screen",
    )

    # E. Gli Spawners (Accendono i singoli controller)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ur_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_controller", "--controller-manager", "/controller_manager"],
    )

    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],
    )

    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "/controller_manager", "--inactive"], # Inattivo all'inizio
    )
    
    gripper_force_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_force_controller", "-c", "/controller_manager", "--inactive"], # Inattivo all'inizio
    )

    return LaunchDescription([
        rsp_node,
        move_group_node,
        rviz_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        ur_arm_controller_spawner,
        # fts_broadcaster_spawner,
        gripper_position_controller_spawner,
        gripper_force_controller_spawner
    ])