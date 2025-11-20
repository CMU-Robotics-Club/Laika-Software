import rclpy
from rclpy.node import Node
import random
from control_msgs.msg import MultiDOFCommand
import math

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
        self.timer_ = self.create_timer(0.025, self.publish_trajectory) # Publish every second
        # self.dof_names = ['fl_lat_joint', 'fl_hip_joint', 'fl_knee_joint',
        #                   'fr_lat_joint', 'fr_hip_joint', 'fr_knee_joint',
        #                   'br_lat_joint', 'br_hip_joint', 'br_knee_joint',
        #                   'bl_lat_joint', 'bl_hip_joint', 'bl_knee_joint'
        #                   ]
        self.dof_names = ['fr_hip_joint', 'fr_knee_joint']
        self.i = 0

    def publish_trajectory(self):
        pid_msg = MultiDOFCommand()
        pid_msg.dof_names = self.dof_names
        
        #hip  = (math.pi)/2
        #knee = (math.pi)/2
        x = 0;
        y = (math.cos(self.i/(math.pi * 10)) + 1)*-0.25

        alpha2,hip = calculate_ik(1,1,x,y)
        if alpha2 == None or hip == None: return
        
        knee = math.pi - alpha2;
        pid_msg.values = [hip, knee]

        self.publisher_.publish(pid_msg)

        self.get_logger().info(f"Publishing MultiDOFCommand message, x: {x} y: {y} hip: {hip} knee: {knee} alpha2: {alpha2}")
        
        self.i = self.i + 1
def main(args=None):
    rclpy.init(args=args)
    set_joints_ik = SetJointsIK()
    rclpy.spin(set_joints_ik)
    set_joints_ik.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
