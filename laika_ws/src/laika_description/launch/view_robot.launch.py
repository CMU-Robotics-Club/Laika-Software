import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def launch_setup(context, *args, **kwargs):
    gui = context.launch_configurations['gui']
    rviz = context.launch_configurations['rviz']
    slide = context.launch_configurations['slide']

    print("")
    print("gui:               " + gui)
    print("rviz:              " + rviz)
    print("slide:             " + slide)
    print("")

    robot_description_file_path = os.path.join(get_package_share_directory('laika_description'), 'xacro', 'robot.xacro')
    rviz_config_file_path = os.path.join(get_package_share_directory('laika_description'), 'config', 'visualize_urdf.rviz')

    nodes = []

    robot_description = xacro.process_file(robot_description_file_path, mappings={
        'fly': 'false',
        'slide': slide,
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
    )
    nodes.append(robot_state_publisher)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file_path],
    )
    if (rviz == "true"):
        nodes.append(rviz_node)

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    if (gui == "true"):
        nodes.append(joint_state_publisher_node)

    return nodes

def generate_launch_description():
    # add launch arguments
    launch_arguments = []
    launch_arguments.append(
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                description='enable joint_state_publisher_gui [true, false]'
                )
            )

    launch_arguments.append(
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                description='enable rviz [true, false]'
                )
            )

    launch_arguments.append(
            DeclareLaunchArgument(
                'slide',
                default_value='false',
                description='enable slider [true, false]'
                )
            )

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
