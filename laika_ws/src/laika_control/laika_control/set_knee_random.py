import rclpy
from rclpy.node import Node
import random
from control_msgs.msg import MultiDOFCommand
import math

class SetKneeRandom(Node):
    def __init__(self):
        super().__init__('SetKneeRandom')
        self.publisher_ = self.create_publisher(MultiDOFCommand, '/pidf_controller/reference', 10)
        self.timer_ = self.create_timer(5.0, self.publish_trajectory) # Publish every second
        self.dof_names = ['fl_knee_joint',
                          'fr_knee_joint',
                          'br_knee_joint',
                          'bl_knee_joint'
                          ]
        self.extend = True

    def publish_trajectory(self):
        pid_msg = MultiDOFCommand()
        pid_msg.dof_names = self.dof_names
        
        if self.extend:
            knee = math.pi/2
            self.extend = False
            self.get_logger().info("Extend")
        else:
            knee = 0
            self.extend = True
            self.get_logger().info("Collapse")

        pid_msg.values = [knee,
                          knee,
                          knee,
                          knee
                          ]

        self.publisher_.publish(pid_msg)

def main(args=None):
    rclpy.init(args=args)
    set_knee_random = SetKneeRandom()
    rclpy.spin(set_knee_random)
    set_knee_random.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
