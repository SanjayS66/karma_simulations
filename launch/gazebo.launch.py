from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    share_dir = get_package_share_directory('rcup_simulations')

    # Gazebo world path
    # world = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'worlds',
    #     'turtlebot3_dqn_stage2.world'  # Replace with your world file if needed
    # )
    world = "/home/sanjay/ros2_workspaces/rcup_simul/src/rcup_simulations/worlds/rcup_custom_arena_fixed_table.world"

    # Xacro file for robot description
    robot_urdf = "/home/sanjay/ros2_workspaces/rcup_simul/src/rcup_simulations/urdf/ArmPlate.urdf"
    with open(robot_urdf, 'r') as infp:  
        robot_description_xml = infp.read()



    # Robot spawn position
    x_pose = LaunchConfiguration('x_pose', default='4.0')
    y_pose = LaunchConfiguration('y_pose', default='9.5')

    # Declare Launch Arguments
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='5.0', description='X position for spawning the robot'
    )

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='6.0', description='Y position for spawning the robot'
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_xml,
                     'use_sim_time':True}],
    )

    #joint state publisher
    joint_state_publisher_node = Node(
    # condition=UnlessCondition(show_gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    # parameters=[{"use_sim_time": True}], 
    )

    # Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_sim_time' : 'true'
            }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # {"robot_description": robot_description_xml,
            #  "use_sim_time":True},
            "/home/sanjay/ros2_workspaces/rcup_simul/src/rcup_simulations/config/mecanum_controllers.yaml",
        ],
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
    )

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
        parameters=[{"use_sim_time": True}], 
    )

    # Spawn the robot in Gazebo with topic remapping
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rcup_bot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose')
        ],
        output='screen',
        parameters=[{"use_sim_time": True}],
        remappings=[('/gazebo_ros_laser_controller/out', '/scan')]
    )


    return LaunchDescription([

        declare_x_pose_cmd,
        declare_y_pose_cmd,

        gzserver_cmd,
        gzclient_cmd,
        urdf_spawn_node,

        joint_state_publisher_node,
        robot_state_publisher_node,

        controller_manager,
        spawner
        
    ])