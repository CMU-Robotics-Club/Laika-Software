from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/fl_knee_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fl_hip_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fl_lat_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fr_knee_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fr_hip_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/fr_lat_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rr_knee_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rr_hip_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rr_lat_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rl_knee_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rl_hip_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/rl_lat_torque_cmd@std_msgs/msg/Float64]ignition.msgs.Double"
        ] + [
            "/forcetorque/fl_knee@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/fl_hip@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/fl_lat@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/fr_knee@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/fr_hip@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/fr_lat@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/rr_knee@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/rr_hip@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/rr_lat@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/rl_knee@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/rl_hip@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench",
            "/forcetorque/rl_lat@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench"
        ] + [
            "/robot_joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ]
    )

    return LaunchDescription([gz_bridge])