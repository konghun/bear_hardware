#include "bear_hardware/bear_hardware.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>


#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "bear_macro.h" 
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

// 토크넣기,위치모드로 설정
CallbackReturn BearHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_DEBUG(rclcpp::get_logger(kBearHardware), "Activating Bear Hardware Interface.");


  // 모터 모드 설정: 모든 조인트를 위치 제어 모드로 설정
  for (auto joint_id : joint_ids_) {
      if (!bear_->SetMode(joint_id, POSITION_CONTROL_MODE)) {  // POSITION_CONTROL_MODE은 위치 제어 모드를 가정
          RCLCPP_ERROR(rclcpp::get_logger(kBearHardware), "Failed to set mode for joint %d", joint_id);
          return CallbackReturn::ERROR;
      }
  }

  // Example: Enable torque for all connected BEAR motors
  for (auto joint_id : joint_ids_) {
    bear_->SetTorqueEnable(joint_id, true);
  }

  RCLCPP_INFO(rclcpp::get_logger(kBearHardware), "Bear Hardware Interface activated.");
  return CallbackReturn::SUCCESS;
}

// 토크 풀기
CallbackReturn BearHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_DEBUG(rclcpp::get_logger(kBearHardware), "Deactivating Bear Hardware Interface.");

  // Example: Disable torque for all connected BEAR motors
  for (auto joint_id : joint_ids_) {
    bear_->SetTorqueEnable(joint_id, false);
  }

  RCLCPP_INFO(rclcpp::get_logger(kBearHardware), "Bear Hardware Interface deactivated.");
  return CallbackReturn::SUCCESS;
}

//pid게인 넣기
CallbackReturn BearHardware::set_joint_params()
{
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        uint8_t joint_id = joint_ids_[i];
        float pGain = 5.0f; // 예시 PID 게인 값, 실제 값은 구성에 따라 다를 수 있음
        float iGain = 0.0f;
        float dGain = 0.1f;

        if (!bear_->SetPGainDirectForce(joint_id, pGain) ||
            !bear_->SetIGainDirectForce(joint_id, iGain) ||
            !bear_->SetDGainDirectForce(joint_id, dGain)) {
            RCLCPP_ERROR(rclcpp::get_logger("BearHardware"), "Failed to set PID gains for joint %d", joint_id);
            return CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("BearHardware"), "All PID gains are set successfully.");
    return CallbackReturn::SUCCESS;
}

// 값읽기
return_type BearHardware::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    // 모터 상태 값을 읽기 위한 주소 목록
    std::vector<uint8_t> read_addresses = {
        bear_macro::PRESENT_POSITION,  // 현재 위치
        bear_macro::PRESENT_VELOCITY,  // 현재 속도
        bear_macro::PRESENT_IQ         // 현재 전류 (토크)
    };

    // 모든 조인트에 대해 BulkRead를 수행
    auto read_results = bear_->BulkRead(joint_ids_, read_addresses);

    // 결과의 크기가 조인트의 수와 일치하는지 확인
    if (read_results.size() != joints_.size()) {
        RCLCPP_ERROR(rclcpp::get_logger(kBearHardware), "Bulk read failed or returned incomplete data.");
        return return_type::ERROR;
    }

    // 읽은 결과를 각 조인트의 상태에 할당
    for (size_t i = 0; i < joints_.size(); ++i) {
        joints_[i].state.position = read_results[i][0]; // 위치
        joints_[i].state.velocity = read_results[i][1]; // 속도
        joints_[i].state.effort = read_results[i][2];   // 전류 (토크)
    }

    return return_type::OK;
}


// 값주기
return_type BearHardware::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    // 명령어 주소 및 데이터 준비
    std::vector<uint8_t> write_addresses = {
        bear_macro::GOAL_POSITION,  // 목표 위치
        bear_macro::GOAL_VELOCITY,  // 목표 속도
        bear_macro::GOAL_IQ         // 목표 전류 (토크)
    };

    std::vector<std::vector<float>> data;

    // 모든 조인트에 대한 데이터를 준비
    for (auto& joint : joints_) {
        std::vector<float> joint_data;
        joint_data.push_back(joint.command.position);  // 조인트별 목표 위치 설정
        joint_data.push_back(joint.command.velocity);  // 조인트별 목표 속도 설정
        joint_data.push_back(joint.command.effort);    // 조인트별 목표 전류 (토크) 설정
        data.push_back(joint_data);
    }

    // BulkWrite 연산 수행
    bool success = bear_->BulkWrite(joint_ids_, write_addresses, data);
    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("BearHardware"), "BulkWrite operation failed.");
        return return_type::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("BearHardware"), "BulkWrite operation successful.");
    return return_type::OK;
}


}  // namespace bear_hardware

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(bear_hardware::BearHardware, hardware_interface::SystemInterface)
