import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
import math
from control_msgs.msg import MultiDOFCommand

class SetJointsSin(Node):
    def __init__(self):
        super().__init__('SetJointsSin')
        self.publisher_ = self.create_publisher(MultiDOFCommand, '/pidf_controller/reference', 10)
        self.timer_ = self.create_timer(0.02, self.publish_trajectory)  # 50 Hz
        self.dof_names = [
            'fl_lat_joint', 'fl_hip_joint', 'fl_knee_joint', # Lat joint is going outwards
            'fr_lat_joint', 'fr_hip_joint', 'fr_knee_joint',
            'br_lat_joint', 'br_hip_joint', 'br_knee_joint',
            'bl_lat_joint', 'bl_hip_joint', 'bl_knee_joint'
        ]
        self.i = 0
        self.freq = 0.5 # walking frequency
        self.amp = 0.7 # knee amplitude

    def publish_trajectory(self):
        pid_msg = MultiDOFCommand()
        pid_msg.dof_names = self.dof_names

        t = self.i * 0.02
        omega = 2 * math.pi * self.freq

        lat = -4.1
        hip_f = 1.5
        # hip_b = 1.2 # This may need to oscillate
        hip_br = 0.35 * math.sin(omega * t) + 0.7
        hip_bl = 0.35 * math.sin(omega * t + math.pi) + 0.7

        knee_offset_f = 2.5
        knee_offset_b = 2

        fl_knee = self.amp * math.sin(omega * t + math.pi*3/2) + knee_offset_f
        fr_knee = self.amp * math.sin(omega * t + math.pi/2) + knee_offset_f
        br_knee = self.amp * math.sin(omega * t) + knee_offset_b
        bl_knee = self.amp * math.sin(omega * t + math.pi) + knee_offset_b

        pid_msg.values = [
            lat, hip_f, fl_knee,   # front-left
            lat, hip_f, fr_knee,   # front-right
            lat, hip_br, br_knee,   # back-right
            lat, hip_bl, bl_knee    # back-left
        ]

        self.publisher_.publish(pid_msg)

        self.i += 1
        if self.i >= 100000:
            self.i = 0

        # Optional for debugging
        self.get_logger().info(f"Publishing knees: FL={fl_knee:.3f}, FR={fr_knee:.3f}, BR={br_knee:.3f}, BL={bl_knee:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = SetJointsSin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
