import math
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState, InterfaceValue

class ODriveControllerSinPublisher(Node):

    def __init__(self):
        super().__init__('odrive_controller_sin_publisher')
        
        self.declare_parameter('period', 6.0)
        self.declare_parameter('amplitude', 5.0)
        self.declare_parameter('joint', 'hip_joint')
        
        self.period = self.get_parameter('period').value
        self.amplitude = self.get_parameter('amplitude').value
        self.joint = self.get_parameter('joint').value
        
        self.counter = 0
        self.publisher_ = self.create_publisher(DynamicJointState, 'laika_pid_controller/command', 10)
        self.timer = self.create_timer(0.001, self.publish_value)

    def publish_value(self):
        msg = DynamicJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [self.joint]

        joint_interfaces = InterfaceValue()
        joint_interfaces.interface_names = ["position"]
        
        step = (self.counter * 2 * math.pi) / (self.period * 1000)
        position = math.sin(step) * self.amplitude
        
        joint_interfaces.values = [position]
        msg.interface_values.append(joint_interfaces)

        self.get_logger().info(f'Publishing Value: {position}')
        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = ODriveControllerSinPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
