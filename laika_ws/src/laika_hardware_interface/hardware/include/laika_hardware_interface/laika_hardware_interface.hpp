#ifndef laika_hardware_interface__laika_hardware_interface_HPP_
#define laika_hardware_interface__laika_hardware_interface_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "socket_can.hpp"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "odrive_enums.h"

namespace laika_hardware_interface
{
class LaikaHardwareInterface : public hardware_interface::SystemInterface
{
  enum Modes {
    IDLE = 0,
    POSITION_FILTERED = 1,
    POSITION_TRAJECTORY = 2,
    VELOCITY_RAMPED = 3,
    TORQUE_CONTROL = 4
  };
  struct Joint {
    // parameters
    std::string name;
    uint8_t can_id;
    double motor_velocity_limit = std::numeric_limits<double>::quiet_NaN();
    double motor_current_limit = std::numeric_limits<double>::quiet_NaN();

    // command variables
    double effort_command = 0.0;
    double mode;
    double previous_mode;

    // state variables
    double position_state = 0.0;
    double velocity_state = 0.0;
    double effort_state = 0.0;
    double effort_target = 0.0;

    // other variables
    uint32_t odrive_error = 1;
    uint8_t odrive_state = 0;
    uint8_t odrive_procedure_result = 0;
    uint8_t odrive_trajectory_done = 0;
    std::string fw_version = "";
    std::string hw_version = "";
    bool ready = false;

    // CAN variables
    SocketCanIntf* can_intf;

    // response functions
    void on_can_msg(const can_frame& frame);
    void on_heartbeat(const can_frame& frame);
    void on_encoder_feedback(const can_frame& frame);
    void on_torque_feedback(const can_frame& frame);
    void on_version_msg(const can_frame& frame);

    // helper functions
    void clear_error();
    void request_encoder_feedback();
    void request_torques_feedback();
    void set_motor_limits();
    void set_mode();
    template <typename V>
    void write_parameter(uint16_t endpoint_id, V value);
    template <typename T>
    void send(const T& msg, bool rtr = false) const {
        struct can_frame frame;
        frame.can_id = can_id << 5 | msg.cmd_id;
        if (rtr) {
          frame.can_id |= CAN_RTR_FLAG;
          frame.can_dlc = 0;
        } else {
          frame.can_dlc = msg.msg_length;
          msg.encode_buf(frame.data);
        }
        can_intf->send_can_frame(frame);
    }
  };

public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


private:
  // parameters
  std::string can_interface_name_;

  // can variables
  SocketCanIntf can_intf_;
  EpollEventLoop event_loop_;

  // joints
  std::vector<Joint> joints;

  // on can msg
  void on_can_msg(const can_frame& frame);

  // helper functions
  void clear_all_errors();
};

}  // namespace laika_hardware_interface

#endif  // laika_hardware_interface__laika_hardware_interface_HPP_
