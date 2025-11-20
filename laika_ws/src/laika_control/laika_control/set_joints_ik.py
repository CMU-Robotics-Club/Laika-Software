import rclpy
from rclpy.node import Node
import random
from control_msgs.msg import MultiDOFCommand
import math
def get_path_coordinates(
    current_time, 
    side_length=0.2, 
    top_y=-0.2, 
    center_x=0.0, 
    t_square=15.0, 
    t_circle=5.0
):
    """
    Calculates the (x, y) setpoint for a robot path that traces a square 
    followed by a circumscribed circle.

    Args:
        current_time (float): The current system time in seconds.
        side_length (float): Length of the square's sides (default: 0.2).
        top_y (float): Y-coordinate of the square's top edge (default: -0.2).
        center_x (float): X-coordinate of the square's center (default: 0.0).
        t_square (float): Duration to complete the square path (default: 15s).
        t_circle (float): Duration to complete the circle path (default: 5s).
    
    Returns:
        tuple: (x, y) coordinates.
    """
    
    # --- 1. Derived Geometry (No Magic Numbers) ---
    half_side = side_length / 2.0
    left_x = center_x - half_side
    right_x = center_x + half_side
    bottom_y = top_y - side_length
    
    # Circle geometry (Center is midpoint of square)
    center_y = (top_y + bottom_y) / 2.0
    
    # Radius is distance from Center to Top-Left corner
    radius = math.sqrt((left_x - center_x)**2 + (top_y - center_y)**2)
    
    # --- 2. Time Normalization ---
    total_cycle_time = t_square + t_circle
    t = current_time % total_cycle_time
    
    # --- 3. Square Phase (Clockwise) ---
    if t < t_square:
        # Determine time per side
        t_segment = t_square / 4.0
        
        # Which side are we on? (0=Top, 1=Right, 2=Bottom, 3=Left)
        segment_idx = int(t // t_segment)
        
        # Normalized ratio (0.0 to 1.0) along the current side
        ratio = (t % t_segment) / t_segment
        
        if segment_idx == 0:   # Top: Left -> Right
            x = left_x + (right_x - left_x) * ratio
            y = top_y
            
        elif segment_idx == 1: # Right: Top -> Bottom
            x = right_x
            y = top_y + (bottom_y - top_y) * ratio
            
        elif segment_idx == 2: # Bottom: Right -> Left
            x = right_x + (left_x - right_x) * ratio
            y = bottom_y
            
        else:                  # Left: Bottom -> Top
            x = left_x
            y = bottom_y + (top_y - bottom_y) * ratio

    # --- 4. Circle Phase (Counter-Clockwise) ---
    else:
        t_local = t - t_square
        
        # Start angle: Angle from Center to Top-Left Corner (Start point)
        start_angle = math.atan2(top_y - center_y, left_x - center_x)
        
        # Calculate current angle (Start + Progress * 2pi)
        # We add 2pi because the green arrows indicate CCW rotation
        angle_offset = (2 * math.pi) * (t_local / t_circle)
        current_angle = start_angle + angle_offset
        
        x = center_x + radius * math.cos(current_angle)
        y = center_y + radius * math.sin(current_angle)

    return x, y

def calculate_ik(l1, l2, x, y, knee_dir=1):
    """
    Calculates alpha2 and theta1 for a 2-link planar leg.
    
    Args:
        l1 (float): Length of the first link
        l2 (float): Length of the second link
        x (float): Target x coordinate
        y (float): Target y coordinate
        knee_dir (int): Direction of the knee bend. 
                        1 for positive alpha2 solution, 
                        -1 for negative alpha2 solution.
        
    Returns:
        tuple: (alpha2, theta1) in radians.
               Returns (None, None) if target is out of reach.
    """
    
    # 1. Law of Cosines
    numerator = x**2 + y**2 - l1**2 - l2**2
    denominator = 2 * l1 * l2
    
    D = numerator / denominator

    if D < -1.0 or D > 1.0:
        # Clamp slightly to handle floating point noise at boundaries
        if abs(D) < 1.0001:
            D = max(-1.0, min(1.0, D))
        else:
            print(f"Error: Target ({x}, {y}) is out of reach.")
            return None, None
        
    # 2. Calculate alpha2
    # knee_dir determines if we take the positive or negative arccos branch
    alpha2 = knee_dir * math.acos(D)
    
    # 3. Calculate theta1
    # Global angle of vector pointing to (-x, -y)
    global_angle = math.atan2(-y, -x)
    
    # Internal offset angle due to link 2
    # Note: sin(alpha2) will flip sign if knee_dir is -1, changing the offset direction
    internal_offset = math.atan2(l2 * math.sin(alpha2), l1 + l2 * math.cos(alpha2))
    
    theta1 = global_angle - internal_offset
    
    return alpha2, theta1

class SetJointsIK(Node):
    def __init__(self):
        super().__init__('SetJointsRandom')
        self.publisher_ = self.create_publisher(MultiDOFCommand, '/pidf_controller/reference', 10)
        self.secs_per_pub = 0.025;
        self.timer_ = self.create_timer(self.secs_per_pub, self.publish_trajectory) # Publish every second
        # self.dof_names = ['fl_lat_joint', 'fl_hip_joint', 'fl_knee_joint',
        #                   'fr_lat_joint', 'fr_hip_joint', 'fr_knee_joint',
        #                   'br_lat_joint', 'br_hip_joint', 'br_knee_joint',
        #                   'bl_lat_joint', 'bl_hip_joint', 'bl_knee_joint'
        #                   ]
        self.dof_names = ['fr_hip_joint', 'fr_knee_joint']
        self.secs_elapsed = 0

    def publish_trajectory(self):
        pid_msg = MultiDOFCommand()
        pid_msg.dof_names = self.dof_names
        
        #hip  = (math.pi)/2
        #knee = (math.pi)/2
        #x = 0;
        #y = (math.cos(self.i/(math.pi * 10)) + 1)*-0.25
        
        x, y = get_path_coordinates(self.secs_elapsed)

        upper_link_len = .35 #meters
        lower_link_len = .41 #meters
        alpha2,hip = calculate_ik(upper_link_len,lower_link_len,x,y)
        self.secs_elapsed = self.secs_elapsed + self.secs_per_pub
        if alpha2 == None or hip == None: return
        
        knee = math.pi - alpha2;
        pid_msg.values = [hip, knee]

        self.publisher_.publish(pid_msg)

        self.get_logger().info(f"Publishing MultiDOFCommand message, x: {x} y: {y} hip: {hip} knee: {knee} alpha2: {alpha2}")
        
def main(args=None):
    rclpy.init(args=args)
    set_joints_ik = SetJointsIK()
    rclpy.spin(set_joints_ik)
    set_joints_ik.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
