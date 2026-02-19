import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState, InterfaceValue

class ODriveControllerValuePublisher(Node):

    def __init__(self):
        super().__init__('odrive_controller_value_publisher')
        
        self.declare_parameter('frequency', 0.0)
        self.declare_parameter('interface', 'position')
        self.declare_parameter('value', 0.0)
        self.declare_parameter('joint', 'hip_joint')
        
        self.frequency = self.get_parameter('frequency').value
        self.interface = self.get_parameter('interface').value
        self.value = self.get_parameter('value').value
        self.joint = self.get_parameter('joint').value
        
        self.publisher_ = self.create_publisher(DynamicJointState, 'laika_pid_controller/command', 10)

        if self.frequency > 0.0:
            period = 1.0 / self.frequency
            self.get_logger().info(f'Mode: REPEAT at {self.frequency:.2f} Hz')
            self.timer = self.create_timer(period, self.publish_value)
        else:
            self.timer = self.create_timer(0.5, self.publish_once)

    def publish_once(self):
        self.publish_value()
        self.timer.cancel()
        rclpy.shutdown()

    def publish_value(self):
        msg = DynamicJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [self.joint]

        joint_interfaces = InterfaceValue()
        joint_interfaces.interface_names = [self.interface]
        joint_interfaces.values = [self.value]
        msg.interface_values.append(joint_interfaces)

        self.get_logger().info(f'Publishing Value: {self.value}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ODriveControllerValuePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
