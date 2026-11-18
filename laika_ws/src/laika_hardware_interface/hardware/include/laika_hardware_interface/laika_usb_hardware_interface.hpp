// ros2_control hardware interface that sends joint setpoints to the STM32 over USB
// (Quads USB protocol, CMD_JOINTS). The STM32 runs the 1 kHz PD loop and talks
// CAN to the ODrives; this side only streams q_des / qd_des / tau_ff.
//
// Not done yet (the protocol has no STM32 -> host state message): read() echoes
// the commands back as the state. No CMD_GAINS / ACK handling yet.
#ifndef LAIKA_HARDWARE_INTERFACE__LAIKA_USB_HARDWARE_INTERFACE_HPP_
#define LAIKA_HARDWARE_INTERFACE__LAIKA_USB_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "laika_hardware_interface/quads_usb_protocol.hpp"
#include "laika_hardware_interface/usb_serial.hpp"

namespace laika_hardware_interface
{

class LaikaUsbHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  struct Joint
  {
    std::string name;
    size_t usb_index = 0;  // slot in the CMD_JOINTS array (firmware joint index)
    double min_position = -std::numeric_limits<double>::infinity();
    double max_position = std::numeric_limits<double>::infinity();

    // command interfaces (joint side: rad, rad/s, N*m)
    double position_command = kNaN;  // NaN until a controller writes a target
    double velocity_command = 0.0;
    double effort_command = 0.0;

    // state interfaces (echo of the last command until the protocol has feedback)
    double position_state = 0.0;
    double velocity_state = 0.0;
  };

  // parameters
  std::string serial_port_;
  double send_period_s_ = 0.01;  // from send_rate_hz

  std::vector<Joint> joints_;
  UsbSerial serial_;

  // runtime
  bool active_ = false;
  uint16_t seq_ = 0;
  rclcpp::Time last_send_time_;
  bool have_sent_ = false;
  uint8_t frame_[quads_usb::kMaxFrameSize] = {};
  uint64_t frames_sent_ = 0;
  uint64_t frames_dropped_ = 0;
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};  // for rate-limited error logs
};

}  // namespace laika_hardware_interface

#endif  // LAIKA_HARDWARE_INTERFACE__LAIKA_USB_HARDWARE_INTERFACE_HPP_
