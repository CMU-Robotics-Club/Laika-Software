from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import sys


def generate_launch_description():
    fly = "false"
    slide = "false"
    log_level = 'error'

    for arg in sys.argv:
        if arg.startswith("info:="):
            content = arg.split(":=")[1]
            if content == "true":
                log_level = 'info'
        if arg.startswith("debug:="):
            content = arg.split(":=")[1]
            if content == "true":
                log_level = 'debug'
        if arg.startswith("fly:="):
            content = arg.split(":=")[1]
            if content == "true":
                fly = "true"
        if arg.startswith("slide:="):
            content = arg.split(":=")[1]
            if content == "true":
                slide = "true"
    
    print("")
    print("log_level:           " + str(log_level))
    print("fly:                 " + str(fly))
    print("slide:               " + str(slide))
    print("")
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ld = LaunchDescription()

    # Get URDF via xacro
    if fly=="true":
        urdf_file_name = "flying_laika.xacro.urdf"
    elif slide == "true":
        urdf_file_name = "sliding_laika.xacro.urdf"
    else:
        urdf_file_name = "laika.xacro.urdf"

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('laika_sim'), 'urdf', urdf_file_name]
                ),
            ]
    )
    robot_description = {'robot_description': robot_description}

    # Specify Gazebo world file (does not include robot itself)
    world_path = PathJoinSubstitution(
            [
                FindPackageShare('laika_sim'),
                'config',
                'world.sdf',
                ]
            )

    # path to controller config file
    robot_controller_config_path = PathJoinSubstitution(
            [
                FindPackageShare('laika_sim'),
                'config',
                'laika_controller.yaml',
                ]
            )

    # Start Gazebo with world
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                launch_arguments={
                    'gz_args': [' -r -v 1 ', world_path],
                    'on_exit_shutdown': 'True'
                    }.items(),
                )
    ld.add_action(gazebo)

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    ld.add_action(robot_state_publisher)
    
    # Spawn robot in gz
    gz_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description',
                       '-name', 'laika', '-allow_renaming', 'true'],
            )
    ld.add_action(gz_spawn_entity)

    joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            )
    # ld.add_action(joint_state_broadcaster_spawner)

    joint_trajectory_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                # 'joint_trajectory_controller',
                'pidf_controller',
                '--param-file',
                robot_controller_config_path,
                ],
            )
    # ld.add_action(joint_trajectory_controller_spawner)

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
            )
        ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
            )
        ))
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    ld.add_action(bridge)

    # Launch Arguments
    ld.add_action(
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description='If true, use simulated clock'))


    return ld
