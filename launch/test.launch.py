import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,ExecuteProcess,TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory of your package
    pkg_share = get_package_share_directory('rcup_simulations')

    # --- File Paths ---
    world_path = os.path.join(pkg_share, 'worlds', 'rcup_custom_arena_fixed_table.world')
    # world_path = os.path.join(pkg_share, 'worlds', 'turtlebot3_dqn_stage2.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ArmPlate.urdf')
    
    # Path to the controller config file
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')
    # controllers_yaml_path = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')

    # --- Robot Description in XML---
    with open(urdf_path, 'r') as infp:
        robot_description_xml = infp.read()

    # --- Launch Arguments ---
    x_pose = LaunchConfiguration('x_pose', default='3.0')
    y_pose = LaunchConfiguration('y_pose', default='11.0')



    declare_x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='3.0',
        description='Initial X position of the robot'
    )
    declare_y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='11.0',
        description='Initial Y position of the robot'
    )

    # --- Core ROS 2 Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True
        }]
    )

    # --- Gazebo Simulation ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path ,'use_sim_time': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # --- Spawning Robot ---
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rcup_bot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.2'
        ],
        output='screen'
    )


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    # --- LaunchDescription Assembly ---
    return LaunchDescription([
        # Launch arguments
        declare_x_pose_arg,
        declare_y_pose_arg,

        # Start Gazebo
        gzserver_cmd,
        gzclient_cmd,

        spawn_entity_node,
        # Core nodes
        robot_state_publisher_node,
        
        diff_drive_spawner,
        joint_broad_spawner
    ]
    )