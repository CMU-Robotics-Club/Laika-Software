import rclpy
from rclpy.node import Node
import random
from control_msgs.msg import MultiDOFCommand

class SetJointsRandom(Node):
    def __init__(self):
        super().__init__('SetJointsRandom')
        self.publisher_ = self.create_publisher(MultiDOFCommand, '/pidf_controller/reference', 10)
        self.timer_ = self.create_timer(3.0, self.publish_trajectory) # Publish every second
        self.dof_names = ['fl_lat_joint', 'fl_hip_joint', 'fl_knee_joint',
                          'fr_lat_joint', 'fr_hip_joint', 'fr_knee_joint',
                          'br_lat_joint', 'br_hip_joint', 'br_knee_joint',
                          'bl_lat_joint', 'bl_hip_joint', 'bl_knee_joint'
                          ]

    def publish_trajectory(self):
        pid_msg = MultiDOFCommand()
        pid_msg.dof_names = self.dof_names

        pid_msg.values = [0.0, -(random.random()*1), (random.random()*1),
                          0.0, -(random.random()*1), (random.random()*1),
                          0.0, -(random.random()*1), (random.random()*1),
                          0.0, -(random.random()*1), (random.random()*1)
                          ]

        self.publisher_.publish(pid_msg)

        self.get_logger().info('Publishing MultiDOFCommand message')

def main(args=None):
    rclpy.init(args=args)
    set_joints_random = SetJointsRandom()
    rclpy.spin(set_joints_random)
    set_joints_random.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
