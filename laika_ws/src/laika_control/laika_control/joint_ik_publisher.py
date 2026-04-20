import math
import sys
import select
import termios
import tty
import threading
from typing import Tuple, Optional, Callable

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState, InterfaceValue


class TerminalKeyLogger(threading.Thread):
    """
    Background thread to continuously monitor standard input.
    Keeps the terminal in cbreak mode to pass keystrokes immediately.
    """
    def __init__(self, key_callback: Callable[[str], None]) -> None:
        super().__init__(daemon=True)
        self.key_callback = key_callback
        self.fd = sys.stdin.fileno()
        self.original_settings = termios.tcgetattr(self.fd)

    def run(self) -> None:
        try:
            tty.setcbreak(self.fd)
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                
                if not rlist:
                    continue
                    
                key = sys.stdin.read(1)
                self.key_callback(key)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)


def get_path_coordinates(
    current_time: float, 
    side_length: float = 0.2, 
    top_y: float = -0.2, 
    center_x: float = 0.0, 
    t_square: float = 15.0, 
    t_circle: float = 5.0
) -> Tuple[float, float]:
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
    knee_dir: int = 1,
    x_limits: Optional[Tuple[float, float]] = None,
    y_limits: Optional[Tuple[float, float]] = None
) -> Tuple[Optional[float], Optional[float]]:
    if x_limits and not (x_limits[0] <= x <= x_limits[1]):
        return None, None
        
    if y_limits and not (y_limits[0] <= y <= y_limits[1]):
        return None, None

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
    def __init__(self) -> None:
        super().__init__('set_joints_ik')
        
        self.publisher = self.create_publisher(
            DynamicJointState, 
            'laika_pid_controller/command', 
            # 'laika_simple_pid_controller/command', 
            10
        )
        
        self.secs_per_pub = 0.025
        self.timer = self.create_timer(self.secs_per_pub, self.publish_trajectory)
        
        self.dof_names = ['hip_joint', 'knee_joint']
        self.secs_elapsed = 0.0
        
        self.upper_link_len = 0.35
        self.lower_link_len = 0.41

        # Compensates for the built-in yaw rotation in leg_current.xacro
        self.knee_yaw_offset: float = 0.04573

        self.is_auto_mode = False
        self.step_size = 0.01 
        
        side_length = 0.2
        top_y = -0.2
        center_x = 0.0
        bottom_y = top_y - side_length
        center_y = (top_y + bottom_y) / 2.0
        
        radius = math.sqrt(((side_length / 2.0)**2) + ((top_y - center_y)**2))
        
        self.min_x = center_x - radius
        self.max_x = center_x + radius
        self.min_y = center_y - radius
        self.max_y = center_y + radius
        
        self.manual_x = center_x
        self.manual_y = center_y

        self.key_logger = TerminalKeyLogger(self.handle_keypress)
        self.key_logger.start()
        
        print("Node started. Press SPACE to toggle modes, WASD to move manually.\r")

    def handle_keypress(self, key: str) -> None:
        """
        Callback executed by the background thread when a key is pressed.
        """
        if key == ' ':
            self.is_auto_mode = not self.is_auto_mode
            mode_str = "AUTO" if self.is_auto_mode else "MANUAL"
            # Using \r to overwrite the line and prevent console spam
            print(f"\rSwitched to {mode_str} mode.                      \r", end='', flush=True)
            return
            
        if self.is_auto_mode:
            return

        char = key.lower()
        if char == 'w':
            self.manual_y = min(self.manual_y + self.step_size, self.max_y)
        elif char == 's':
            self.manual_y = max(self.manual_y - self.step_size, self.min_y)
        elif char == 'd':
            self.manual_x = max(self.manual_x - self.step_size, self.min_x)
        elif char == 'a':
            self.manual_x = min(self.manual_x + self.step_size, self.max_x)

    def publish_trajectory(self) -> None:
        if self.is_auto_mode:
            x, y = get_path_coordinates(self.secs_elapsed)
            self.secs_elapsed += self.secs_per_pub
        else:
            x, y = self.manual_x, self.manual_y

        alpha2, hip_angle = calculate_ik(
            self.upper_link_len, 
            self.lower_link_len, 
            x, 
            y,
            x_limits=(self.min_x, self.max_x),
            y_limits=(self.min_y, self.max_y)
        )
        
        if alpha2 is None or hip_angle is None:
            return
        
        knee_angle = math.pi - alpha2
        #apply the knee offset in the sim's xacro file
        knee_angle = knee_angle - self.knee_yaw_offset

        msg = DynamicJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.dof_names

        for angle in [hip_angle, knee_angle]:
            joint_interface = InterfaceValue()
            joint_interface.interface_names = ["position"]
            joint_interface.values = [angle]
            msg.interface_values.append(joint_interface)

        self.publisher.publish(msg)


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
