#include "laika_hardware_interface/laika_hardware_interface.hpp"
#include <cmath>
#include <string>
#include <vector>
#include "socket_can.hpp"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "odrive_enums.h"

namespace laika_hardware_interface
{
  ////////////////////// on_init /////////////////////////
  hardware_interface::CallbackReturn LaikaHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Load hardware parameters
    can_interface_name_ = info_.hardware_parameters["can_interface_name"];
    RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM] CAN inteface: %s", can_interface_name_.c_str());

    // load joints 
    for (const hardware_interface::ComponentInfo & joint_info : info_.joints)
    {
      // Check for correct size of command and state interfaces
      if (joint_info.command_interfaces.size() != 1) {
        RCLCPP_FATAL(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM] Joint '%s' has %zu command interfaces. 1 expected.", joint_info.name.c_str(), joint_info.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint_info.state_interfaces.size() != 3) {
        RCLCPP_FATAL(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM] Joint '%s' has %zu state interfaces. 3 expected.", joint_info.name.c_str(), joint_info.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // create joint object and loading parameters
      LaikaHardwareInterface::Joint joint;
      joint.name = joint_info.name;
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM] Creating Joint '%s': ", joint.name.c_str());
      if (!joint_info.parameters.count("can_id")) {
        RCLCPP_FATAL(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM] Parameter can_id has to be specified!");
        return CallbackReturn::FAILURE;
      }
      joint.can_id = (uint8_t)std::stoi(joint_info.parameters.at("can_id"));
      if (joint_info.parameters.count("motor_velocity_limit")) {
        joint.motor_velocity_limit = std::stod(joint_info.parameters.at("motor_velocity_limit"));
      }
      if (joint_info.parameters.count("motor_current_limit")) {
        joint.motor_current_limit = std::stod(joint_info.parameters.at("motor_current_limit"));
      }

      // set initial mode
      joint.mode = Modes::TORQUE_CONTROL;

      // print info
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM]   can-id: %d", joint.can_id);
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM]   mode: TORQUE_CONTROL");
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM]   velocity-limit: %.1f", joint.motor_velocity_limit);
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[PARAM]   current-limit: %.1f", joint.motor_current_limit);
      joints.push_back(joint);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  hardware_interface::CallbackReturn LaikaHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    // connect to CAN
    if (!can_intf_.init(can_interface_name_, &event_loop_, std::bind(&LaikaHardwareInterface::on_can_msg, this, _1))) {
      RCLCPP_ERROR(rclcpp::get_logger("LaikaHardwareInterface"), "[CONFIG] Failed to initialize SocketCAN on %s", can_interface_name_.c_str());
      RCLCPP_ERROR(rclcpp::get_logger("LaikaHardwareInterface"), "[CONFIG] Have you run 'sudo ip link set up can0 type can bitrate 1000000'?");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[CONFIG] Succesfully initialized SocketCAN on %s", can_interface_name_.c_str());
    for (auto& joint : joints) {
      joint.can_intf = &can_intf_;
    }

    // Check if Motor connected and if version is correct
    for (auto& joint : joints) {
      Get_Version_msg_t msg;
      joint.send(msg, true);
    }
    for (int i = 0; i <= 30; i++) {
      bool finish = true;
      while (can_intf_.read_nonblocking()); // repeat until CAN interface has no more messages
      for (auto& joint : joints) {
        if (joint.hw_version.length() != 0 && joint.fw_version.length() != 0) {
          RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[CONFIG] Found: Joint '%s' can-id '%d' with fw-version '%s' and hw-version '%s'", joint.name.c_str(), joint.can_id, joint.fw_version.c_str(), joint.hw_version.c_str());
          if (joint.fw_version != "0:6:11" || joint.hw_version != "5:2:0") {
            RCLCPP_ERROR(rclcpp::get_logger("LaikaHardwareInterface"), "[CONFIG] Joint '%s' with can-id '%d' is running the wrong version! Please update to 0:6:11 and 5:2:0.", joint.name.c_str(), joint.can_id);
            return CallbackReturn::FAILURE;
          }
        } else {
          finish = false;
        }
      }
      if (finish) {
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      if (i == 30) {
        RCLCPP_ERROR(rclcpp::get_logger("LaikaHardwareInterface"), "[CONFIG] Could not reach all motors! Check CAN connection!");
        return CallbackReturn::FAILURE;
      }
    }

    // clear all errors
    clear_all_errors();
    RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[CONFIG] Cleared all errors");

    // write parameters
    for (auto& joint : joints) {
      joint.set_motor_limits();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  hardware_interface::CallbackReturn LaikaHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    // Arm all axis
    for (auto& joint : joints) {
      joint.set_mode();
      joint.previous_mode = joint.mode;
    }

    // Check if all joints are ready
    while (1) {
      // Request Torques and Encoder Estimates
      for (auto &joint : joints) {
        Get_Torques_msg_t get_torques_msg;
        Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
        joint.send(get_torques_msg, true);
        joint.send(get_encoder_estimages_msg, true);
      }
      while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
      }
      bool finish = true;
      for (auto &joint : joints) {
        if (joint.odrive_error == 0 && joint.position_state != 0) {
          joint.effort_command = 0.0;
          joint.ready = true;
          RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[ACTIVATION] Motor with can-id '%d' for joint '%s' is ready!", joint.can_id, joint.name.c_str());
        } else {
          // keep interface from activating if any motor has an error
          finish = false;
        }
      }
      if (finish) {
        RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[ACTIVATION] Hardware succesfully activated!");
        break;
        // return hardware_interface::CallbackReturn::SUCCESS;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[ACTIVATION] Hardware not ready yet...");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }


  ////////////////////// read /////////////////////////
  hardware_interface::return_type LaikaHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    while (can_intf_.read_nonblocking()) {
      // repeat until CAN interface has no more messages
    }

    // CONTINUE ONLY IF READY
    for (auto &joint : joints) {
      if (!joint.ready) {
        return hardware_interface::return_type::OK;
      }
    }
    
    // Request Torques and Encoder Estimates
    for (auto &joint : joints) {
      Get_Torques_msg_t get_torques_msg;
      Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
      joint.send(get_torques_msg, true);
      joint.send(get_encoder_estimages_msg, true);
    }

    return hardware_interface::return_type::OK;
  }

  ////////////////////// write /////////////////////////
  hardware_interface::return_type LaikaHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // CONTINUE ONLY IF READY
    for (auto &joint : joints) {
      if (!joint.ready) {
        return hardware_interface::return_type::OK;
      }
    }

    // set command position to all joints
    for (auto &joint : joints) {
      if (joint.previous_mode != joint.mode) {
        joint.set_mode();
      }
      joint.previous_mode = joint.mode;
      if ((int)joint.mode == Modes::TORQUE_CONTROL) {
        Set_Input_Torque_msg_t msg;
        msg.Input_Torque = joint.effort_command;
        joint.send(msg);
      }
    }
    return hardware_interface::return_type::OK;
  }

  ////////////////////// on_deactivate /////////////////////////
  hardware_interface::CallbackReturn LaikaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    // Configure all axis
    for (auto& joint : joints) {
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_IDLE;
      joint.send(set_axis_state_msg);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_cleanup /////////////////////////
  hardware_interface::CallbackReturn LaikaHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_shutdown /////////////////////////
  hardware_interface::CallbackReturn LaikaHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// export_state_interfaces /////////////////////////
  std::vector<hardware_interface::StateInterface> LaikaHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto &joint : joints) {
      state_interfaces.emplace_back(joint.name, "position", &joint.position_state);
      state_interfaces.emplace_back(joint.name, "velocity", &joint.velocity_state);
      state_interfaces.emplace_back(joint.name, "effort", &joint.effort_state);
    }
    return state_interfaces;
  }

  ////////////////////// export_command_interfaces /////////////////////////
  std::vector<hardware_interface::CommandInterface> LaikaHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto &joint : joints) {
      command_interfaces.emplace_back(joint.name, "effort", &joint.effort_command);
    }
    return command_interfaces;
  }

  ////////////////////// handle messages /////////////////////////
  void LaikaHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& joint : joints) {
      if ((frame.can_id >> 5) == joint.can_id) {
        joint.on_can_msg(frame);
      }
    }
  }
  void LaikaHardwareInterface::Joint::on_can_msg(const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    switch (cmd) {
      case Get_Encoder_Estimates_msg_t::cmd_id: {
        on_encoder_feedback(frame);
      } break;
      case Get_Torques_msg_t::cmd_id: {
        on_torque_feedback(frame);
      } break;
      case Heartbeat_msg_t::cmd_id: {
        on_heartbeat(frame);
      } break;
      case Get_Version_msg_t::cmd_id: {
        on_version_msg(frame);
      } break;
    }
  }

  ////////////////////// joint response functions /////////////////////////
  void LaikaHardwareInterface::Joint::on_encoder_feedback(const can_frame& frame) {
    if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("LaikaHardwareInterface"), "message %d too short", Get_Encoder_Estimates_msg_t::cmd_id);
      return;
    }
    Get_Encoder_Estimates_msg_t msg;
    msg.decode_buf(frame.data);
    if (name.find("hip") != std::string::npos) {
      position_state = msg.Pos_Estimate * (2 * M_PI) / 14;
      velocity_state = msg.Vel_Estimate * (2 * M_PI) / 14;
    } else {
      position_state = msg.Pos_Estimate * (2 * M_PI);
      velocity_state = msg.Vel_Estimate * (2 * M_PI);
    }
  }
  void LaikaHardwareInterface::Joint::on_torque_feedback(const can_frame& frame) {
    if (frame.can_dlc < Get_Torques_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("LaikaHardwareInterface"), "message %d too short", Get_Torques_msg_t::cmd_id);
      return;
    }
    Get_Torques_msg_t msg;
    msg.decode_buf(frame.data);
    effort_target = msg.Torque_Target;
    effort_state = msg.Torque_Estimate;
  }
  void LaikaHardwareInterface::Joint::on_heartbeat(const can_frame& frame) {
    if (frame.can_dlc < Heartbeat_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("LaikaHardwareInterface"), "message %d too short", Heartbeat_msg_t::cmd_id);
      return;
    }
    Heartbeat_msg_t msg;
    msg.decode_buf(frame.data);
    odrive_error = msg.Axis_Error;
    odrive_state = msg.Axis_State;
    odrive_procedure_result = msg.Procedure_Result;
    odrive_trajectory_done = msg.Trajectory_Done_Flag;
    if (odrive_error != 0) {
      ready = false;
      RCLCPP_ERROR(rclcpp::get_logger("LaikaHardwareInterface"), "Joint '%s' with can-id '%d' has error '%d' and state '%d'", name.c_str(), can_id, odrive_error, odrive_state);
    }
  }
  void LaikaHardwareInterface::Joint::on_version_msg(const can_frame& frame) {
    Get_Version_msg_t msg;
    msg.decode_buf(frame.data);
    hw_version = std::to_string(msg.Hw_Version_Major) + ":" + std::to_string(msg.Hw_Version_Minor) + ":" + std::to_string(msg.Hw_Version_Variant);
    fw_version = std::to_string(msg.Fw_Version_Major) + ":" + std::to_string(msg.Fw_Version_Minor) + ":" + std::to_string(msg.Fw_Version_Revision);
  }

  ////////////////////// joint helper functions /////////////////////////
  void LaikaHardwareInterface::Joint::clear_error() {
    Clear_Errors_msg_t msg;
    msg.Identify = 0;
    send(msg);
  }
  void LaikaHardwareInterface::Joint::request_encoder_feedback() {
    Get_Encoder_Estimates_msg_t get_encoder_estimages_msg;
    send(get_encoder_estimages_msg, true);
  }
  void LaikaHardwareInterface::Joint::request_torques_feedback() {
    Get_Torques_msg_t get_torques_msg;
    send(get_torques_msg, true);
  }
  template <typename V>
  void LaikaHardwareInterface::Joint::write_parameter(uint16_t endpoint_id, V value) {
    struct can_frame frame;
    frame.can_id = can_id << 5 | 0x04;
    frame.can_dlc = 8;
    can_set_signal_raw<uint8_t>(frame.data, 1, 0, 8, true);
    can_set_signal_raw<uint16_t>(frame.data, endpoint_id, 8, 24, true);
    can_set_signal_raw<uint8_t>(frame.data, 0, 24, 32, true);
    if constexpr (std::is_same_v<V, uint8_t>) {
      can_set_signal_raw<uint8_t>(frame.data, value, 32, 40, true);
    } else if constexpr (std::is_same_v<V, uint16_t>) {
      can_set_signal_raw<uint16_t>(frame.data, value, 32, 48, true);
    } else if constexpr (std::is_same_v<V, uint32_t>) {
      can_set_signal_raw<uint32_t>(frame.data, value, 32, 56, true);
    } else if constexpr (std::is_same_v<V, int8_t>) {
      can_set_signal_raw<int8_t>(frame.data, value, 32, 40, true);
    } else if constexpr (std::is_same_v<V, int16_t>) {
      can_set_signal_raw<int16_t>(frame.data, value, 32, 48, true);
    } else if constexpr (std::is_same_v<V, int32_t>) {
      can_set_signal_raw<int32_t>(frame.data, value, 32, 56, true);
    } else if constexpr (std::is_same_v<V, bool>) {
      can_set_signal_raw<bool>(frame.data, value, 32, 40, true);
    } else if constexpr (std::is_same_v<V, float>) {
      can_set_signal_raw<float>(frame.data, value, 32, 64, true);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("LaikaHardwareInterface"), "Unsupported type in write_parameter function");
      return;
    }
    can_intf->send_can_frame(frame);
  }
  void LaikaHardwareInterface::Joint::set_motor_limits() {
    if (std::isnan(motor_velocity_limit) || std::isnan(motor_current_limit)) {
      return;
    }
    // Set Velocity Limit
    Set_Limits_msg_t msg;
    if (motor_velocity_limit != 0) {
      msg.Velocity_Limit = (float)motor_velocity_limit;
    } else {
      msg.Velocity_Limit = std::numeric_limits<float>::infinity();
    }
    // Set Soft Current Limit
    if (motor_current_limit != 0) {
      msg.Current_Limit = (float)motor_current_limit;
    } else {
      msg.Current_Limit = std::numeric_limits<float>::infinity();
    }
    send(msg);
  }
  void LaikaHardwareInterface::Joint::set_mode() {
    // control mode
    if ((int)mode == Modes::IDLE) {
      // idle
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_IDLE;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[UPDATE] Changed Mode to: IDLE");
    } else if ((int)mode == Modes::POSITION_FILTERED) {
      // position filtered 
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_POS_FILTER;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[UPDATE] Changed Mode to: POSITION_FILTERED");
    } else if ((int)mode == Modes::POSITION_TRAJECTORY) {
      // position trajectory 
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_TRAP_TRAJ;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[UPDATE] Changed Mode to: POSITION_TRAJECTORY");
    } else if ((int)mode == Modes::VELOCITY_RAMPED) {
      // velocity ramped 
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_VEL_RAMP;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[UPDATE] Changed Mode to: VELOCITY_RAMPED");
    } else if ((int)mode == Modes::TORQUE_CONTROL) {
      // torque control
      Set_Controller_Mode_msg_t set_controller_mode_msg;
      set_controller_mode_msg.Control_Mode = ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL;
      set_controller_mode_msg.Input_Mode = ODriveInputMode::INPUT_MODE_PASSTHROUGH;
      send(set_controller_mode_msg);
      Set_Axis_State_msg_t set_axis_state_msg;
      set_axis_state_msg.Axis_Requested_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
      send(set_axis_state_msg);
      RCLCPP_INFO(rclcpp::get_logger("LaikaHardwareInterface"), "[UPDATE] Changed Mode to: TORQUE_CONTROL");
    } else if ((int)mode != 0) {
      RCLCPP_FATAL(rclcpp::get_logger("LaikaHardwareInterface"), "Unrecognized mode: %d", (int)mode);
    }
  }

  ////////////////////// general helper functions /////////////////////////
  void LaikaHardwareInterface::clear_all_errors() {
    for (auto& joint : joints) {
      joint.clear_error();
    }
  }


}  // namespace laika_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(laika_hardware_interface::LaikaHardwareInterface, hardware_interface::SystemInterface)
