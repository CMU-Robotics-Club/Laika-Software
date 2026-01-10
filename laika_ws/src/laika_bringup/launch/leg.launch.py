import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    log_level = context.launch_configurations['log_level']
    # config = context.launch_configurations['config']

    print("")
    print("log_level:           " + log_level)
    print("")

    robot_description_file_path = os.path.join(get_package_share_directory('laika_description'), 'xacro', 'robot.xacro')
    controller_config_path = os.path.join(get_package_share_directory('laika_pid_controller'), 'config', 'leg_pid_controllers.yaml')

    robot_description = xacro.process_file(robot_description_file_path, mappings={
        'fly': 'false',
        'slide': 'false',
        'leg_only': 'true',
        'simulation': 'false'
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
        arguments=["--ros-args", "--log-level", log_level]
    )

    # Controller Manager (starts and loads hardware interface)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        output="both",
        arguments=["--ros-args", "--log-level", log_level]
    )

    # Joint State Broadcaster (publishes position, velocity and effort of each joint from the harware interface)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["joint_state_broadcaster", "--param-file", controller_config_path, "--ros-args", "--log-level", log_level]
    )

    # PID Controller Spawner (starts the custom controller)
    laika_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["laika_pid_controller", "--param-file", controller_config_path, "--ros-args", "--log-level", log_level]
    )

    return [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        laika_pid_controller_spawner,
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
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
