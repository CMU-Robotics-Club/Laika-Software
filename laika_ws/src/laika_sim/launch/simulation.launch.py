import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def launch_setup(context, *args, **kwargs):
    log_level = context.launch_configurations['log_level']
    config = context.launch_configurations['config']

    print("")
    print("log_level:           " + log_level)
    print("config:              " + config)
    print("")

    world_file_path = PathJoinSubstitution([FindPackageShare('laika_sim'), 'config', 'world.sdf'])
    robot_description_file_path = os.path.join(get_package_share_directory('laika_description'), 'xacro', 'robot.xacro')

    if (config == "flying_leg"):
        fly = 'true'
        slide = 'false'
        leg_only = 'true'
    elif (config == "sliding_leg"):
        fly = 'false'
        slide = 'true'
        leg_only = 'true'
    else:
        fly = 'false'
        slide = 'false'
        leg_only = 'true'

    robot_description = xacro.process_file(robot_description_file_path, mappings={
        'fly': fly,
        'slide': slide,
        'leg_only': leg_only,
        'simulation': 'true'
    }).toxml()
    
    # robot description publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 100.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    # Gazebo
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                launch_arguments={
                    'gz_args': [' -r -v 1 ', world_file_path],
                    'on_exit_shutdown': 'True'
                    }.items(),
                )


    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-z', '0.0'],
    )

    joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return [
        gz_bridge,
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
    ]


def generate_launch_description():
    # add launch arguments
    launch_arguments = []
    launch_arguments.append(
            DeclareLaunchArgument(
                'log_level',
                default_value='error',
                description='log_level [info, error, debug]'
                )
            )

    launch_arguments.append(
            DeclareLaunchArgument(
                'config',
                default_value='leg',
                description='config [leg, flying_leg, sliding_leg]'
                )
            )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
