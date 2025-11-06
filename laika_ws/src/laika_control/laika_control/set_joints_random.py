import rclpy
from rclpy.node import Node
import random
from control_msgs.msg import MultiDOFCommand
import math

class SetJointsRandom(Node):
    def __init__(self):
        super().__init__('SetJointsRandom')
        self.publisher_ = self.create_publisher(MultiDOFCommand, '/pidf_controller/reference', 10)
        self.timer_ = self.create_timer(0.05, self.publish_trajectory) # Publish every second
        self.dof_names = ['fl_lat_joint', 'fl_hip_joint', 'fl_knee_joint',
                          'fr_lat_joint', 'fr_hip_joint', 'fr_knee_joint',
                          'br_lat_joint', 'br_hip_joint', 'br_knee_joint',
                          'bl_lat_joint', 'bl_hip_joint', 'bl_knee_joint'
                          ]

    def publish_trajectory(self):
        pid_msg = MultiDOFCommand()
        pid_msg.dof_names = self.dof_names
        
        latfl = -4.0#(random.random()*0.8)-0.5
        hipfl = (random.random()*math.pi/2)
        kneefl = (random.random()*math.pi)

        latfr = -4.0#(random.random()*0.8)-0.5
        hipfr = (random.random()*math.pi/2)
        kneefr = (random.random()*math.pi)

        latbr = -4.0#(random.random()*0.8)-0.5
        hipbr = (random.random()*math.pi/2)
        kneebr = (random.random()*math.pi)

        latbl = -4.0#(random.random()*0.8)-0.5
        hipbl = (random.random()*math.pi/2)
        kneebl = (random.random()*math.pi)

        pid_msg.values = [latfl, hipfl, kneefl,
                          latfl, hipfl, kneefl,
                          latbr, hipbr, kneebr,
                          latbr, hipbr, kneebr
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
