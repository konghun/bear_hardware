#include "bear_hardware/bear_hardware.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bear_hardware {
constexpr const char* kBearHardware = "BearHardware";

CallbackReturn BearHardware::on_init(const hardware_interface::HardwareInfo& info) {
  RCLCPP_DEBUG(rclcpp::get_logger(kBearHardware), "Initializing Bear Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);


  // 이미 생성자에서 bear_ 객체를 초기화했으므로, 여기서 추가적으로 bear_를 초기화할 필요가 없습니다.
  // bear_->connect(); 는 필요한 초기화 작업을 수행합니다.
  
   bear_->connect(); 

  // Initialize each joint based on your configuration
  for (uint i = 0; i < info_.joints.size(); ++i) {
    // Assign joint IDs from the hardware info parameters
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));

    // Initialize joint states and commands with NaN to indicate they are not set yet
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(rclcpp::get_logger(kBearHardware), "Bear Hardware Interface initialized successfully.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BearHardware::export_state_interfaces() {
  RCLCPP_DEBUG(rclcpp::get_logger(kBearHardware), "Exporting state interfaces.");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Loop through each joint and register state interfaces for position, velocity, and effort
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BearHardware::export_command_interfaces() {
  RCLCPP_DEBUG(rclcpp::get_logger(kBearHardware), "Exporting command interfaces.");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Loop through each joint and register command interfaces for position, velocity, and effort
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command.effort));
  }

  return command_interfaces;
}

CallbackReturn BearHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_DEBUG(rclcpp::get_logger(kBearHardware), "Activating Bear Hardware Interface.");

  // Example: Enable torque for all connected BEAR motors
  for (auto joint_id : joint_ids_) {
    bear_->SetTorqueEnable(joint_id, true);
  }

  RCLCPP_INFO(rclcpp::get_logger(kBearHardware), "Bear Hardware Interface activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BearHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_DEBUG(rclcpp::get_logger(kBearHardware), "Deactivating Bear Hardware Interface.");

  // Example: Disable torque for all connected BEAR motors
  for (auto joint_id : joint_ids_) {
    bear_->SetTorqueEnable(joint_id, false);
  }

  RCLCPP_INFO(rclcpp::get_logger(kBearHardware), "Bear Hardware Interface deactivated.");
  return CallbackReturn::SUCCESS;
}

return_type BearHardware::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  // Implement your logic to read from the BEAR motors
  // Example: read position, velocity, and effort for each joint
  for (uint i = 0; i < joints_.size(); ++i) {
    auto& joint = joints_[i];
    joint.state.position = bear_->GetGoalPosition(joint_ids_[i]);
    joint.state.velocity = bear_->GetGoalVelocity(joint_ids_[i]);
    // Effort (current) reading example
    joint.state.effort = bear_->GetPresentIq(joint_ids_[i]);
  }

  return return_type::OK;
}

return_type BearHardware::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  // Implement your logic to write to the BEAR motors
  // Example: write position or velocity commands for each joint
  for (uint i = 0; i < joints_.size(); ++i) {
    auto& joint = joints_[i];
    // Set position or velocity based on control mode
    if (control_mode_ == ControlMode::Position) {
      bear_->SetGoalPosition(joint_ids_[i], joint.command.position);
    } else if (control_mode_ == ControlMode::Velocity) {
      bear_->SetGoalVelocity(joint_ids_[i], joint.command.velocity);
    }
    // Add effort (torque) command example if applicable
  }

  return return_type::OK;
}

}  // namespace bear_hardware

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(bear_hardware::BearHardware, hardware_interface::SystemInterface)
