#ifndef BEAR_HARDWARE__BEAR_HARDWARE_HPP_
#define BEAR_HARDWARE__BEAR_HARDWARE_HPP_

#include <yaml-cpp/yaml.h>
#include <bear_sdk.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace bear_hardware {
struct JointValue {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};

struct Joint {
    JointValue state{};
    JointValue command{};
    JointValue prev_command{};
};

enum class ControlMode {
    Position,
    Velocity,
    Effort, // Adjust based on BEAR motors' capabilities
};

class BearHardware : public hardware_interface::SystemInterface {
public:

    RCLCPP_SHARED_PTR_DEFINITIONS(BearHardware)

    BearHardware()
    : bear_(std::make_unique<bear::BEAR>("/dev/ttyUSB0", 7500000)) {
        // 추가 초기화 작업이 필요하면 여기서 수행
    }

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::unique_ptr<bear::BEAR> bear_;
    std::vector<Joint> joints_;
    std::vector<uint8_t> joint_ids_;
    std::vector<double> mechanical_reductions_;
    std::vector<double> mechanical_offsets_;
    bool torque_enabled_{false};
    ControlMode control_mode_{ControlMode::Position};
    bool mode_changed_{false};


    
};

}  // namespace bear_hardware

#endif  // BEAR_HARDWARE__BEAR_HARDWARE_HPP_
