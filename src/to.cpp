#include "rclcpp/rclcpp.hpp"
#include "bear_sdk.h"

using namespace bear;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("motor_torque_enable_node");

    const char* portName = "/dev/ttyUSB0";
    int baudrate = 7500000;

    BEAR bear(portName, baudrate);

    // 모터의 ID 설정
    uint8_t motorID = 1; // 예시로 1번 모터를 사용

    // 토크 활성화 값 설정 (1로 설정)
    uint32_t torqueEnable = 1;

    // 모터의 토크 활성화
    bool success = bear.SetTorqueEnable(motorID, torqueEnable);

    if (success) {
        RCLCPP_INFO(logger, "Motor %d: Torque enabled successfully.", motorID);
    } else {
        uint8_t errorCode = bear.GetErrorCode();
        RCLCPP_ERROR(logger, "Motor %d: Failed to enable torque. Error code: %d", motorID, errorCode);
    }

    rclcpp::shutdown();
    return 0;
}
