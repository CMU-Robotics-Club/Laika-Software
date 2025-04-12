from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/fl_knee_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fl_hip_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fl_lat_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fr_knee_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fr_hip_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fr_lat_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rr_knee_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rr_hip_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rr_lat_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rl_knee_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rl_hip_torque@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rl_lat_torque@std_msgs/msg/Float64]ignition.msgs.Double"
        ]
    )

    return LaunchDescription([gz_bridge])