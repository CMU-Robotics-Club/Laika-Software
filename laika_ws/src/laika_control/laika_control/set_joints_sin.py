import rclpy
from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

from control_msgs.msg import MultiDOFCommand

class SetJointsSin(Node):
    def __init__(self):
        super().__init__('SetJointsSin')
        self.publisher_ = self.create_publisher(MultiDOFCommand, '/pidf_controller/reference', 10)
        self.timer_ = self.create_timer(0.02, self.publish_trajectory) # Publish every second
        self.dof_names = ['fl_lat_joint', 'fl_hip_joint', 'fl_knee_joint',
                          'fr_lat_joint', 'fr_hip_joint', 'fr_knee_joint',
                          'br_lat_joint', 'br_hip_joint', 'br_knee_joint',
                          'bl_lat_joint', 'bl_hip_joint', 'bl_knee_joint'
                          ]
        self.i = 0


    def publish_trajectory(self):
        pid_msg = MultiDOFCommand()
        pid_msg.dof_names = self.dof_names

        step = (self.i * 2 * math.pi)/1000
        lat = 0.0
        hip = 0.1 + (-math.cos(step) + 1) * (math.pi/4) * 0.8
        knee = 0.1 + (-math.cos(step) + 1) * (math.pi/2) * 0.8
        pid_msg.values = [lat, hip, knee,
                          lat, hip, knee,
                          lat, hip, knee,
                          lat, hip, knee
                          ]

        self.publisher_.publish(pid_msg)

        if self.i == 1000:
            self.i = 0
        else:
            self.i = self.i + 1

        self.get_logger().info('Publishing MultiDOFCommand message')

def main(args=None):
    rclpy.init(args=args)
    set_joints_sin = SetJointsSin()
    rclpy.spin(set_joints_sin)
    set_joints_sin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
