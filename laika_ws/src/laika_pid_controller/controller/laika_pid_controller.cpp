#include "laika_pid_controller/laika_pid_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace laika_pid_controller
{
LaikaPidController::LaikaPidController() : controller_interface::ControllerInterface() {}

////////////////////// on_init /////////////////////////
controller_interface::CallbackReturn LaikaPidController::on_init() {
  try {
    param_listener = std::make_shared<ParamListener>(get_node());
    params = param_listener->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("LaikaPidController"), "[INIT] Exception thrown when reading parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

////////////////////// on_configure /////////////////////////
controller_interface::CallbackReturn LaikaPidController::on_configure(const rclcpp_lifecycle::State &) {
  for (size_t i = 0; i < params.joints.size(); ++i)
  {
    LaikaPidController::Joint joint;
    joint.name = params.joints[i];
    joint.id = i;
    control_toolbox::AntiWindupStrategy anti_windup_strategy_;
    anti_windup_strategy_.set_type("none");
    try {
      anti_windup_strategy_.validate();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("LaikaPidController"), "[CONFIGURE] Invalid antiwindup strategy:: %s", e.what());
      return CallbackReturn::ERROR;
    }
    joint.position_pid = std::make_shared<control_toolbox::Pid>(
        params.gains.joints_map[joint.name].position_p / (2 * M_PI),
        params.gains.joints_map[joint.name].position_i / (2 * M_PI),
        params.gains.joints_map[joint.name].position_d / (2 * M_PI),
        params.gains.joints_map[joint.name].position_output_max / (2 * M_PI),
        params.gains.joints_map[joint.name].position_output_min / (2 * M_PI),
        anti_windup_strategy_
        );
    joint.velocity_pid = std::make_shared<control_toolbox::Pid>(
        params.gains.joints_map[joint.name].velocity_p / (2 * M_PI),
        params.gains.joints_map[joint.name].velocity_i / (2 * M_PI),
        params.gains.joints_map[joint.name].velocity_d / (2 * M_PI),
        params.gains.joints_map[joint.name].velocity_output_max / (2 * M_PI),
        params.gains.joints_map[joint.name].velocity_output_min / (2 * M_PI),
        anti_windup_strategy_
        );
    joints.push_back(joint);
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM] Creating Joint '%s': ", joint.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM]   id: %d", joint.id);
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM]   position-p-gain: %.3f", params.gains.joints_map[joint.name].position_p);
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM]   position-i-gain: %.3f", params.gains.joints_map[joint.name].position_i);
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM]   position-d-gain: %.3f", params.gains.joints_map[joint.name].position_d);
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM]   velocity-p-gain: %.3f", params.gains.joints_map[joint.name].velocity_p);
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM]   velocity-i-gain: %.3f", params.gains.joints_map[joint.name].velocity_i);
    RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "[PARAM]   velocity-d-gain: %.3f", params.gains.joints_map[joint.name].velocity_d);
  }

  joints_cmd_sub = this->get_node()->create_subscription<CommandType>("laika_pid_controller/command", rclcpp::SystemDefaultsQoS(),
    [this](const CommandType::SharedPtr msg) {
      rt_buffer.set(*msg);
    });

  return CallbackReturn::SUCCESS;
}


////////////////////// on_activate /////////////////////////
controller_interface::CallbackReturn LaikaPidController::on_activate(const rclcpp_lifecycle::State &) {
  // get joint states
  get_joint_states();
  for (auto& joint : joints) {
    joint.position_reference = joint.position_state;
  }
  return CallbackReturn::SUCCESS;
}

////////////////////// update /////////////////////////
controller_interface::return_type LaikaPidController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
  // get joint states
  get_joint_states();

  // get joint reference
  get_joint_references();

  // update pid loop
  for (auto& joint : joints) {
    // position pid
    if (std::isnan(joint.position_reference) || std::isnan(joint.position_state)) {continue;}
    double position_error = (joint.position_reference * 14) - (joint.position_state * 14);
    double temp_output = joint.position_pid->compute_command(position_error, period);
    // velocity pid
    if (std::isnan(joint.velocity_state)) {continue;}
    double velocity_error = temp_output - (joint.velocity_state * 14);
    joint.effort_command = joint.velocity_pid->compute_command(velocity_error, period);
    // write effort value
    if (!command_interfaces_[joint.id].set_value(joint.effort_command)) {
      RCLCPP_WARN(rclcpp::get_logger("LaikaPidController"), "Failed to set effort command interface value for joint '%s'", joint.name.c_str());
    }
  }


  return controller_interface::return_type::OK;
}

////////////////////// on_deactivate /////////////////////////
controller_interface::CallbackReturn LaikaPidController::on_deactivate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

////////////////////// command_interface_configuration /////////////////////////
controller_interface::InterfaceConfiguration LaikaPidController::command_interface_configuration() const {

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::vector<std::string> names_;
  for (const auto& joint : joints) {
    names_.push_back(joint.name + "/effort");
  }
  command_interfaces_config.names = names_;
  return command_interfaces_config;
}

////////////////////// state_interface_configuration /////////////////////////
controller_interface::InterfaceConfiguration LaikaPidController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::vector<std::string> names_;
  for (const auto& joint : joints) {
    names_.push_back(joint.name + "/position");
    names_.push_back(joint.name + "/velocity");
    names_.push_back(joint.name + "/effort");
  }
  state_interfaces_config.names = names_;
  return state_interfaces_config;
}

////////////////////// helper functions /////////////////////////
void LaikaPidController::get_joint_states() {
  for (auto& joint : joints) {
    const auto position_op = state_interfaces_[joint.id * 3].get_optional();
    if (position_op.has_value()) {joint.position_state = position_op.value();}
    const auto velocity_op = state_interfaces_[(joint.id * 3) + 1].get_optional();
    if (velocity_op.has_value()) {joint.velocity_state = velocity_op.value();}
    const auto effort_op = state_interfaces_[(joint.id * 3) + 2].get_optional();
    if (effort_op.has_value()) {joint.effort_state = effort_op.value();}
    // RCLCPP_INFO(rclcpp::get_logger("LaikaPidController"), "Joint '%s' State: pos: %f vel: %f effort: %f Command: pos: %f", joint.name.c_str(), joint.position_state, joint.velocity_state, joint.effort_state, joint.position_command);
  }
}
void LaikaPidController::get_joint_references() {
  auto reference_op = rt_buffer.try_get();
  if (reference_op.has_value())
  {
    reference_msg = reference_op.value();
    int i = 0;
    for (std::string joint_name : reference_msg.joint_names) {
      // search for joint with same name
      auto it = std::find_if(joints.begin(), joints.end(), [&](const Joint& joint) {
          return joint.name == joint_name;
          });

      // Check if the joint was found
      if (it != joints.end()) {
        // loop through interfaces
        int a = 0;
        for (std::string interface_name : reference_msg.interface_values[i].interface_names) {
          // check if interface name is recognized
          it->position_reference = std::numeric_limits<double>::quiet_NaN();
          if (reference_msg.interface_values[i].interface_names[a] == "position") {
            it->position_reference = reference_msg.interface_values[i].values[a];
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("LaikaPidController"), "Wrong type of command interface! Expected 'position' Got '%s'", reference_msg.interface_values[i].interface_names[a].c_str());
          }
          a++;
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("LaikaPidController"), "Joint of name '%s' not recognized!", joint_name.c_str());
      }
      i++;
    }
  }
}


}  // namespace laika_pid_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(laika_pid_controller::LaikaPidController, controller_interface::ControllerInterface)
