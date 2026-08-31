import sys
import select
import termios
import tty
import threading
import math
from typing import Tuple, Optional, Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


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
    """Generates the automated square and circle trajectory."""
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


class CartesianCommandPublisher(Node):
    def __init__(self) -> None:
        super().__init__('cartesian_command_publisher')
        
        # Publish to the multi-array topic
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            'laika_cartesian_impedance_controller/command', 
            10
        )
        
        self.secs_per_pub = 0.025
        self.timer = self.create_timer(self.secs_per_pub, self.publish_trajectory)
        self.secs_elapsed = 0.0
        
        self.is_auto_mode = False
        self.step_size = 0.001 
        
        # Define limits for the automated path bounding box
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

        # Keep track of previous coordinates to calculate finite-difference velocity
        self.prev_x = center_x
        self.prev_y = center_y

        self.key_logger = TerminalKeyLogger(self.handle_keypress)
        self.key_logger.start()
        
        print("Node started. Press SPACE to toggle modes, WASD to move manually.\n")

    def handle_keypress(self, key: str) -> None:
        """Callback executed by the background thread when a key is pressed."""
        if key == ' ':
            self.is_auto_mode = not self.is_auto_mode
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

        # Clamp the coordinates to prevent pushing the leg too far physically
        x = max(self.min_x, min(x, self.max_x))
        y = max(self.min_y, min(y, self.max_y))

        # Calculate finite-difference velocities
        dx = (x - self.prev_x) / self.secs_per_pub
        dy = (y - self.prev_y) / self.secs_per_pub

        # Update previous values for the next tick
        self.prev_x = x
        self.prev_y = y

        # No feed-forward forces for this test script
        fx = 0.0
        fy = 0.0

        # Create and publish the 6-value array: [x, y, dx, dy, Fx, Fy]
        msg = Float64MultiArray()
        msg.data = [float(x), float(y), float(dx), float(dy), float(fx), float(fy)]
        self.publisher.publish(msg)

        # Print live status 
        mode_str = "AUTO" if self.is_auto_mode else "MANUAL"
        sys.stdout.write(f"\r[{mode_str}] Target -> X: {x:+.3f}, Y: {y:+.3f} | Vel -> dX: {dx:+.3f}, dY: {dy:+.3f}       ")
        sys.stdout.flush()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = CartesianCommandPublisher()
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
