from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import sys

def generate_launch_description():
    important_log_level = "info"
    other_log_level = "error"

    for arg in sys.argv:
        if arg.startswith("log:="):
            content = arg.split(":=")[1]
            if content == "error":
                important_log_level = "error"
                other_log_level = "error"
            elif content == "all":
                important_log_level = "info"
                other_log_level = "info"
    
    # Get file paths
    robot_description_path = PathJoinSubstitution([FindPackageShare("odrive_ros2_control_example"), "urdf", "robot.urdf.xacro"])
    controller_config_path = PathJoinSubstitution([FindPackageShare("odrive_ros2_control_example"), "config", "robot_controllers.yaml"])

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Define Nodes
    # Robot State Publisher (publishes urdf)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", other_log_level]
    )
    # Controller Manager (starts and loads hardware interface)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        output="both",
        arguments=["--ros-args", "--log-level", important_log_level]
    )
    # Joint State Broadcaster (publishes position, velocity and effort of each joint from the harware interface)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["joint_state_broadcaster", "--ros-args", "--log-level", other_log_level]
    )
    # Odrive Controller Spawner (starts the custom controller)
    odrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=["odrive_controller", "--param-file", controller_config_path, "--ros-args", "--log-level", important_log_level]
    )
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        odrive_controller_spawner,
    ])

