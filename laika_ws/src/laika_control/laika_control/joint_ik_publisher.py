import math
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState, InterfaceValue


def get_path_coordinates(
    current_time: float, 
    side_length: float = 0.2, 
    top_y: float = -0.2, 
    center_x: float = 0.0, 
    t_square: float = 15.0, 
    t_circle: float = 5.0
) -> Tuple[float, float]:
    """
    Calculates the (x, y) setpoint for a robot path tracing a square 
    followed by a circumscribed circle.
    """
    half_side = side_length / 2.0
    left_x = center_x - half_side
    right_x = center_x + half_side
    bottom_y = top_y - side_length
    
    center_y = (top_y + bottom_y) / 2.0
    radius = math.sqrt((left_x - center_x)**2 + (top_y - center_y)**2)
    
    total_cycle_time = t_square + t_circle
    t = current_time % total_cycle_time
    
    if t < t_square:
        t_segment = t_square / 4.0
        segment_idx = int(t // t_segment)
        ratio = (t % t_segment) / t_segment
        
        if segment_idx == 0:
            x = left_x + (right_x - left_x) * ratio
            y = top_y
        elif segment_idx == 1:
            x = right_x
            y = top_y + (bottom_y - top_y) * ratio
        elif segment_idx == 2:
            x = right_x + (left_x - right_x) * ratio
            y = bottom_y
        else:
            x = left_x
            y = bottom_y + (top_y - bottom_y) * ratio
    else:
        t_local = t - t_square
        start_angle = math.atan2(top_y - center_y, left_x - center_x)
        angle_offset = (2 * math.pi) * (t_local / t_circle)
        current_angle = start_angle + angle_offset
        
        x = center_x + radius * math.cos(current_angle)
        y = center_y + radius * math.sin(current_angle)

    return x, y


def calculate_ik(
    l1: float, 
    l2: float, 
    x: float, 
    y: float, 
    knee_dir: int = 1
) -> Tuple[Optional[float], Optional[float]]:
    """
    Calculates alpha2 and theta1 for a 2-link planar leg.
    """
    numerator = x**2 + y**2 - l1**2 - l2**2
    denominator = 2 * l1 * l2
    
    d_val = numerator / denominator

    if d_val < -1.0 or d_val > 1.0:
        if abs(d_val) < 1.0001:
            d_val = max(-1.0, min(1.0, d_val))
        else:
            return None, None
        
    alpha2 = knee_dir * math.acos(d_val)
    
    global_angle = math.atan2(-y, -x)
    internal_offset = math.atan2(l2 * math.sin(alpha2), l1 + l2 * math.cos(alpha2))
    theta1 = global_angle - internal_offset
    
    return alpha2, theta1


class SetJointsIK(Node):
    """
    Node to calculate and publish inverse kinematics joint states for a robotic leg.
    """
    
    def __init__(self) -> None:
        super().__init__('set_joints_ik')
        
        self.publisher = self.create_publisher(
            DynamicJointState, 
            'laika_pid_controller/command', 
            10
        )
        
        self.secs_per_pub = 0.025
        self.timer = self.create_timer(self.secs_per_pub, self.publish_trajectory)
        
        self.dof_names = ['hip_joint', 'knee_joint']
        self.secs_elapsed = 0.0
        
        self.upper_link_len = 0.35
        self.lower_link_len = 0.41

    def publish_trajectory(self) -> None:
        """
        Calculates the next IK target, constructs the DynamicJointState message, 
        and publishes it to the controller.
        """
        x, y = get_path_coordinates(self.secs_elapsed)
        alpha2, hip_angle = calculate_ik(self.upper_link_len, self.lower_link_len, x, y)
        
        self.secs_elapsed += self.secs_per_pub
        
        if alpha2 is None or hip_angle is None:
            self.get_logger().error(f"Error: Target ({x:.3f}, {y:.3f}) is out of reach.")
            return
        
        knee_angle = math.pi - alpha2
        
        msg = DynamicJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.dof_names

        for angle in [hip_angle, knee_angle]:
            joint_interface = InterfaceValue()
            joint_interface.interface_names = ["position"]
            joint_interface.values = [angle]
            msg.interface_values.append(joint_interface)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published IK -> x: {x:.3f} y: {y:.3f} "
            f"hip: {hip_angle:.3f} knee: {knee_angle:.3f}"
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SetJointsIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
