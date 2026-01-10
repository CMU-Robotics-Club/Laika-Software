#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "control_msgs/msg/interface_value.hpp"

using namespace std::chrono_literals;

class ODriveControllerSinPublisher : public rclcpp::Node
{
public:
  ODriveControllerSinPublisher()
  : Node("odrive_controller_sin_publisher")
  {
    this->declare_parameter("period", 6.0);
    this->declare_parameter("amplitude", 8*M_PI);
    this->declare_parameter("joint", "arm");
    period = this->get_parameter("period").as_double();
    amplitude = this->get_parameter("amplitude").as_double();
    joint = this->get_parameter("joint").as_string();

    counter = 0;
    publisher_ = this->create_publisher<control_msgs::msg::DynamicJointState>("laika_pid_controller/command", 10);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(0.001), std::bind(&ODriveControllerSinPublisher::publish_value, this));

  }

private:
  void publish_value() {
    // Create the message
    auto message = control_msgs::msg::DynamicJointState();
    message.header.stamp = this->now();
    message.joint_names = {joint};

    control_msgs::msg::InterfaceValue joint_interfaces;
    joint_interfaces.interface_names = {"position"};
    double step = (counter * 2 * M_PI)/(period * 1000);
    double position = std::sin(step) * amplitude;
    joint_interfaces.values = {position};
    message.interface_values.push_back(joint_interfaces);

    // Publish
    RCLCPP_INFO(this->get_logger(), "Publishing Value: %f", position);
    publisher_->publish(message);
    counter++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr publisher_;
  uint32_t counter;
  double period;
  double amplitude;
  std::string joint;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODriveControllerSinPublisher>());
  rclcpp::shutdown();
  return 0;
}
