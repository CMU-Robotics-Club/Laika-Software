#include "laika_hardware_interface/laika_usb_hardware_interface.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <random>
#include <set>

#include "rclcpp/rclcpp.hpp"

namespace laika_hardware_interface
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("LaikaUsbHardwareInterface");

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
}  // namespace

////////////////////// on_init: read and check parameters /////////////////////////
CallbackReturn LaikaUsbHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & hw = info_.hardware_parameters;
  if (!hw.count("serial_port")) {
    RCLCPP_FATAL(kLogger, "[PARAM] Hardware parameter 'serial_port' is required");
    return CallbackReturn::ERROR;
  }
  serial_port_ = hw.at("serial_port");

  double send_rate_hz = 100.0;
  if (hw.count("send_rate_hz")) {
    send_rate_hz = std::stod(hw.at("send_rate_hz"));
  }
  // Protocol: CMD_JOINTS at 50-200 Hz
  if (!(send_rate_hz >= 50.0 && send_rate_hz <= 200.0)) {
    RCLCPP_FATAL(kLogger, "[PARAM] send_rate_hz = %.1f, must be 50-200", send_rate_hz);
    return CallbackReturn::ERROR;
  }
  send_period_s_ = 1.0 / send_rate_hz;
  RCLCPP_INFO(kLogger, "[PARAM] serial_port: %s, send_rate_hz: %.1f",
    serial_port_.c_str(), send_rate_hz);

  std::set<size_t> used_indices;
  for (const auto & joint_info : info_.joints) {
    Joint joint;
    joint.name = joint_info.name;

    // usb_index decides the slot in CMD_JOINTS, independent of the joint order here.
    // It must match the firmware's joint index (joint_interface.h).
    if (!joint_info.parameters.count("usb_index")) {
      RCLCPP_FATAL(kLogger, "[PARAM] Joint '%s': parameter 'usb_index' is required",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    const int idx = std::stoi(joint_info.parameters.at("usb_index"));
    if (idx < 0 || static_cast<size_t>(idx) >= quads_usb::kMaxJoints ||
      !used_indices.insert(static_cast<size_t>(idx)).second)
    {
      RCLCPP_FATAL(kLogger, "[PARAM] Joint '%s': usb_index %d is out of range 0-11 or used twice",
        joint.name.c_str(), idx);
      return CallbackReturn::ERROR;
    }
    joint.usb_index = static_cast<size_t>(idx);

    // Optional position clamp from <command_interface name="position"><param name="min"/max>
    for (const auto & ci : joint_info.command_interfaces) {
      if (ci.name == "position") {
        if (!ci.min.empty()) {joint.min_position = std::stod(ci.min);}
        if (!ci.max.empty()) {joint.max_position = std::stod(ci.max);}
      }
    }

    RCLCPP_INFO(kLogger, "[PARAM] Joint '%s': usb_index %zu, position limits [%.3f, %.3f] rad",
      joint.name.c_str(), joint.usb_index, joint.min_position, joint.max_position);
    joints_.push_back(joint);
  }
  return CallbackReturn::SUCCESS;
}

////////////////////// lifecycle: open / close the port /////////////////////////
CallbackReturn LaikaUsbHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!serial_.open(serial_port_, error)) {
    RCLCPP_FATAL(kLogger, "Cannot open serial port: %s", error.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Opened %s", serial_port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn LaikaUsbHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Random start seq, like quads_host.py, so a restart does not reuse old numbers
  std::random_device rd;
  seq_ = static_cast<uint16_t>(rd() & 0xFFFF);

  // Nothing is sent until a controller writes a real position target
  for (auto & joint : joints_) {
    joint.position_command = kNaN;
    joint.velocity_command = 0.0;
    joint.effort_command = 0.0;
  }
  have_sent_ = false;
  frames_sent_ = 0;
  frames_dropped_ = 0;
  active_ = true;
  RCLCPP_INFO(kLogger, "Activated, start seq %u. Waiting for position targets.", seq_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn LaikaUsbHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop streaming. Nothing is sent to tell the STM32 to stop: it relies on its own
  // command timeout (quads_firmware branch feat/usb-cmd-timeout: no CMD_JOINTS for
  // 100 ms -> FAULT, motors IDLE). Older firmware has no such timeout.
  active_ = false;
  RCLCPP_INFO(kLogger, "Deactivated. Frames sent: %lu, dropped: %lu",
    static_cast<unsigned long>(frames_sent_), static_cast<unsigned long>(frames_dropped_));
  return CallbackReturn::SUCCESS;
}

CallbackReturn LaikaUsbHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  serial_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LaikaUsbHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  active_ = false;
  serial_.close();
  return CallbackReturn::SUCCESS;
}

////////////////////// interfaces /////////////////////////
std::vector<hardware_interface::StateInterface>
LaikaUsbHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : joints_) {
    state_interfaces.emplace_back(joint.name, "position", &joint.position_state);
    state_interfaces.emplace_back(joint.name, "velocity", &joint.velocity_state);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LaikaUsbHardwareInterface::export_command_interfaces()
{
  // position -> q_des, velocity -> qd_des, effort -> tau_ff
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : joints_) {
    command_interfaces.emplace_back(joint.name, "position", &joint.position_command);
    command_interfaces.emplace_back(joint.name, "velocity", &joint.velocity_command);
    command_interfaces.emplace_back(joint.name, "effort", &joint.effort_command);
  }
  return command_interfaces;
}

////////////////////// read: echo commands (no feedback in the protocol yet) /////////////////////////
return_type LaikaUsbHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : joints_) {
    if (std::isfinite(joint.position_command)) {
      joint.position_state = std::clamp(joint.position_command, joint.min_position, joint.max_position);
      joint.velocity_state = std::isfinite(joint.velocity_command) ? joint.velocity_command : 0.0;
    }
  }
  return return_type::OK;
}

////////////////////// write: one CMD_JOINTS frame per send period /////////////////////////
return_type LaikaUsbHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!active_) {
    return return_type::OK;
  }

  // The controller manager may run faster (e.g. 1 kHz); send at send_rate_hz only
  if (have_sent_ && (time - last_send_time_).seconds() < send_period_s_) {
    return return_type::OK;
  }

  // Unused slots stay 0. Every slot must be finite: the firmware drops the whole
  // frame (silently, no ACK for CMD_JOINTS) if any value is NaN/inf.
  quads_usb::JointsCommand cmd{};
  for (const auto & joint : joints_) {
    if (!std::isfinite(joint.position_command)) {
      return return_type::OK;  // no target yet for this joint: send nothing
    }
    auto & sp = cmd[joint.usb_index];
    sp.q_des = static_cast<float>(
      std::clamp(joint.position_command, joint.min_position, joint.max_position));
    sp.qd_des = std::isfinite(joint.velocity_command) ? static_cast<float>(joint.velocity_command) : 0.0f;
    sp.tau_ff = std::isfinite(joint.effort_command) ? static_cast<float>(joint.effort_command) : 0.0f;
  }

  uint8_t payload[quads_usb::kJointsPayloadSize];
  quads_usb::pack_joints(cmd, payload);
  const size_t n = quads_usb::build_frame(
    quads_usb::kCmdJoints, seq_, payload, sizeof(payload), frame_, sizeof(frame_));

  const long w = serial_.write_some(frame_, n);
  // Step the send time by exactly one period instead of using `time`: with
  // `last_send_time_ = time`, a call that comes a little late moves the whole
  // schedule later, the next call then looks too early and a cycle is skipped
  // (measured ~78 Hz instead of 100 Hz). After a long gap (first frame, or no
  // target for a while) restart from now, so missed frames are not sent in a burst.
  const auto period = rclcpp::Duration::from_seconds(send_period_s_);
  if (!have_sent_ || time - last_send_time_ > period * 2) {
    last_send_time_ = time;
  } else {
    last_send_time_ = last_send_time_ + period;
  }
  have_sent_ = true;
  seq_ = static_cast<uint16_t>(seq_ + 1);  // the firmware counts gaps, so bump even on a drop

  if (w == static_cast<long>(n)) {
    ++frames_sent_;
    return return_type::OK;
  }

  ++frames_dropped_;
  if (w < 0) {
    // e.g. the board was unplugged. Keep the controller manager running; the
    // firmware's command timeout must stop the motors (feat/usb-cmd-timeout, 100 ms).
    RCLCPP_ERROR_THROTTLE(kLogger, throttle_clock_, 1000,
      "Serial write failed: %s (dropped %lu frames)", std::strerror(errno),
      static_cast<unsigned long>(frames_dropped_));
  } else if (w > 0) {
    // Partial frame on the wire: end it with 0x00 so the next frame starts clean.
    // The STM32 rejects the broken piece by its CRC.
    const uint8_t delim = 0x00;
    serial_.write_some(&delim, 1);
  }
  return return_type::OK;
}

}  // namespace laika_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  laika_hardware_interface::LaikaUsbHardwareInterface, hardware_interface::SystemInterface)
