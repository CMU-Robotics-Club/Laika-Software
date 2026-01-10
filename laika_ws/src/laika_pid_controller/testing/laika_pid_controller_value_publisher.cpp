#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "control_msgs/msg/interface_value.hpp"

using namespace std::chrono_literals;

class ODriveControllerValuePublisher : public rclcpp::Node
{
public:
  ODriveControllerValuePublisher()
  : Node("odrive_controller_value_publisher")
  {
    this->declare_parameter("frequency", 0.0);
    this->declare_parameter("interface", "position");
    this->declare_parameter("value", 0.0);
    this->declare_parameter("joint", "arm");
    frequency = this->get_parameter("frequency").as_double();
    interface = this->get_parameter("interface").as_string();
    value = this->get_parameter("value").as_double();
    joint = this->get_parameter("joint").as_string();

    publisher_ = this->create_publisher<control_msgs::msg::DynamicJointState>("laika_pid_controller/command", 10);

    if (frequency > 0.0) {
      // --- REPEATED MODE ---
      auto timer_period = std::chrono::duration<double>(1.0 / frequency);
      RCLCPP_INFO(this->get_logger(), "Mode: REPEAT at %.2f Hz", frequency);
      timer_ = this->create_wall_timer(timer_period, std::bind(&ODriveControllerValuePublisher::publish_value, this));

    } else {
      // --- SINGLE SHOT MODE ---
      timer_ = this->create_wall_timer(500ms, [this]() {
          this->publish_value();
          this->timer_->cancel();
          rclcpp::shutdown();
          });
    }
  }

private:
  void publish_value() {
    // Create the message
    auto message = control_msgs::msg::DynamicJointState();
    message.header.stamp = this->now();
    message.joint_names = {joint};

    control_msgs::msg::InterfaceValue joint_interfaces;
    joint_interfaces.interface_names = {interface};
    joint_interfaces.values = {value};
    message.interface_values.push_back(joint_interfaces);

    // Publish
    RCLCPP_INFO(this->get_logger(), "Publishing Value: %f", value);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr publisher_;
  std::string joint;
  std::string interface;
  double value;
  double frequency;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODriveControllerValuePublisher>());
  rclcpp::shutdown();
  return 0;
}
