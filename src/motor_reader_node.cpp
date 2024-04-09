#include "rclcpp/rclcpp.hpp"
#include "bear_hardware/bear_hardware.hpp"

class SimpleActivationNode : public rclcpp::Node {
public:
    SimpleActivationNode() : Node("simple_activation_node") {
        // BearHardware 인스턴스 생성
        auto bear_hardware = std::make_shared<bear_hardware::BearHardware>();

        // BearHardware 인스턴스 활성화
        auto result = bear_hardware->on_activate(rclcpp_lifecycle::State());

        // 활성화 상태 로깅
        if (result == hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "BearHardware successfully activated.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate BearHardware.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleActivationNode>();
    rclcpp::spin(node); // 노드 실행
    rclcpp::shutdown();
    return 0;
}
