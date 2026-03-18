#include "laika_cartesian_impedance_controller/laika_cartesian_impedance_controller.hpp"

#include <cmath>
#include <stddef.h>
#include <algorithm>
#include "rclcpp/time.hpp"

namespace laika_cartesian_impedance_controller
{
LaikaCartesianImpedanceController::LaikaCartesianImpedanceController(): controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn LaikaCartesianImpedanceController::on_init() {
  try {
    param_listener_ = std::make_shared<laika_cartesian_impedance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("LaikaCartesianImpedanceController"), "[INIT] Exception thrown when reading parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LaikaCartesianImpedanceController::on_configure(const rclcpp_lifecycle::State &) {
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    JointData joint;
    joint.name = params_.joints[i];
    joint.id = i;
    joints_.push_back(joint);
    
    RCLCPP_INFO(rclcpp::get_logger("LaikaCartesianImpedanceController"), "[PARAM] Configuring Joint '%s' with id: %d", joint.name.c_str(), joint.id);
  }

  cartesian_cmd_sub_ = this->get_node()->create_subscription<CommandType>(
    "laika_cartesian_impedance_controller/command", rclcpp::SystemDefaultsQoS(),
    [this](const CommandType::SharedPtr msg) {
      rt_buffer_.writeFromNonRT(*msg);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LaikaCartesianImpedanceController::on_activate(const rclcpp_lifecycle::State &) {
  get_joint_states();

  // Initialize reference to current position to prevent violent snapping
  if (joints_.size() >= 2 && !std::isnan(joints_[0].position_state) && !std::isnan(joints_[1].position_state)) {
    double th_h = joints_[0].position_state;
    double th_k = joints_[1].position_state;
    
    double th_k_eff = th_k + KNEE_OFFSET; 

    // Initialize position to current, explicitly zero out velocity and feed-forward
    x_ref_ = LINK_LENGTH_UPPER * std::cos(th_h) - LINK_LENGTH_LOWER * std::cos(th_h - th_k_eff);
    y_ref_ = -LINK_LENGTH_UPPER * std::sin(th_h) + LINK_LENGTH_LOWER * std::sin(th_h - th_k_eff);
    dx_ref_ = 0.0;
    dy_ref_ = 0.0;
    fx_ff_ = 0.0;
    fy_ff_ = 0.0;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LaikaCartesianImpedanceController::update(const rclcpp::Time &, const rclcpp::Duration &) {
  get_joint_states();
  get_cartesian_reference();

  if (joints_.size() < 2) {
    return controller_interface::return_type::OK;
  }

  JointData& hip = joints_[0];
  JointData& knee = joints_[1];

  if (std::isnan(hip.position_state) || std::isnan(knee.position_state) ||
      std::isnan(hip.velocity_state) || std::isnan(knee.velocity_state)) {
    return controller_interface::return_type::OK;
  }

  double th_h = hip.position_state;
  double th_k = knee.position_state;
  double dth_h = hip.velocity_state;
  double dth_k = knee.velocity_state;

  double th_k_eff = th_k + KNEE_OFFSET;

  // 1. Forward Kinematics 
  double x = LINK_LENGTH_UPPER * std::cos(th_h) - LINK_LENGTH_LOWER * std::cos(th_h - th_k_eff);
  double y = -LINK_LENGTH_UPPER * std::sin(th_h) + LINK_LENGTH_LOWER * std::sin(th_h - th_k_eff);

  // 2. Jacobian Calculation
  double j00 = -LINK_LENGTH_UPPER * std::sin(th_h) + LINK_LENGTH_LOWER * std::sin(th_h - th_k_eff);
  double j01 = -LINK_LENGTH_LOWER * std::sin(th_h - th_k_eff);
  double j10 = -LINK_LENGTH_UPPER * std::cos(th_h) + LINK_LENGTH_LOWER * std::cos(th_h - th_k_eff);
  double j11 = -LINK_LENGTH_LOWER * std::cos(th_h - th_k_eff);

  double dx = j00 * dth_h + j01 * dth_k;
  double dy = j10 * dth_h + j11 * dth_k;

  // 3. Full State Impedance Control Law + Feed Forward
  double fx = params_.stiffness[0] * (x_ref_ - x) + params_.damping[0] * (dx_ref_ - dx) + fx_ff_;
  double fy = params_.stiffness[1] * (y_ref_ - y) + params_.damping[1] * (dy_ref_ - dy) + fy_ff_;

  // 4. Transform forces to joint torques
  double tau_h = j00 * fx + j10 * fy;
  double tau_k = j01 * fx + j11 * fy;

  static int print_counter = 0;
  if (++print_counter % 100 == 0) {
      RCLCPP_INFO(rclcpp::get_logger("LaikaCartesianImpedanceController"), 
        "Foot Pos -> X: %.4f, Y: %.4f | Tgt -> X: %.4f, Y: %.4f | F_ff -> X: %.2f, Y: %.2f", 
        x, y, x_ref_, y_ref_, fx_ff_, fy_ff_);
  }

  hip.effort_command = tau_h / GEAR_RATIO;
  knee.effort_command = tau_k / GEAR_RATIO;

  if (!command_interfaces_[hip.id].set_value(hip.effort_command)) {
    RCLCPP_WARN(rclcpp::get_logger("LaikaCartesianImpedanceController"), "Failed to set effort for %s", hip.name.c_str());
  }
  if (!command_interfaces_[knee.id].set_value(knee.effort_command)) {
    RCLCPP_WARN(rclcpp::get_logger("LaikaCartesianImpedanceController"), "Failed to set effort for %s", knee.name.c_str());
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn LaikaCartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LaikaCartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint.name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration LaikaCartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint.name + "/position");
    config.names.push_back(joint.name + "/velocity");
    config.names.push_back(joint.name + "/effort");
  }
  return config;
}

void LaikaCartesianImpedanceController::get_joint_states() {
  for (auto& joint : joints_) {
    const auto position_op = state_interfaces_[joint.id * 3].get_optional();
    if (position_op.has_value()) { joint.position_state = position_op.value(); }
    
    const auto velocity_op = state_interfaces_[(joint.id * 3) + 1].get_optional();
    if (velocity_op.has_value()) { joint.velocity_state = velocity_op.value(); }
    
    const auto effort_op = state_interfaces_[(joint.id * 3) + 2].get_optional();
    if (effort_op.has_value()) { joint.effort_state = effort_op.value(); }
  }
}

void LaikaCartesianImpedanceController::get_cartesian_reference() {
  auto reference_ptr = rt_buffer_.readFromRT();
  if (reference_ptr && reference_ptr->data.size() >= 6) {
    reference_msg_ = *reference_ptr;
    
    // Explicit 6-value unpack: [x, y, dx, dy, Fx, Fy]
    x_ref_  = reference_msg_.data[0];
    y_ref_  = reference_msg_.data[1];
    dx_ref_ = reference_msg_.data[2];
    dy_ref_ = reference_msg_.data[3];
    fx_ff_  = reference_msg_.data[4];
    fy_ff_  = reference_msg_.data[5];
  }
}

}  // namespace laika_cartesian_impedance_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(laika_cartesian_impedance_controller::LaikaCartesianImpedanceController, controller_interface::ControllerInterface)
