#include "rclcpp/rclcpp.hpp"
#include "bear_sdk.h"
#include "bear_macro.h"

using namespace bear;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger = rclcpp::get_logger("motor_set_mode_and_pid_gain_node");

    const char* portName = "/dev/ttyUSB0";
    int baudrate = 7500000; // 샘플 코드와 일치하는 baudrate 설정

    BEAR bear(portName, baudrate);

    uint8_t motorID = 1; // 설정할 모터의 ID
    uint32_t mode = 3; // 모드 값 (예시: 3은 위치 제어 모드일 수 있음. 실제 값은 모터 문서 참조)
    float pGain = 2.0; // P 게인 값
    float iGain = 0.0; // I 게인 값
    float dGain = 0.1; // D 게인 값

    // 모드 설정
    bool modeSuccess = bear.SetMode(motorID, mode);
    if (modeSuccess) {
        RCLCPP_INFO(logger, "Mode setting successful. Mode: %d", mode);
    } else {
        RCLCPP_ERROR(logger, "Mode setting failed.");
    }

    // P, I, D 게인 값 설정
    bool pSuccess = bear.SetPGainDirectForce(motorID, pGain);
    bool iSuccess = bear.SetIGainDirectForce(motorID, iGain);
    bool dSuccess = bear.SetDGainDirectForce(motorID, dGain);

    if (pSuccess && iSuccess && dSuccess) {
        RCLCPP_INFO(logger, "PID Gain setting successful.");
    } else {
        RCLCPP_ERROR(logger, "PID Gain setting failed.");
    }

    rclcpp::shutdown();
    return 0;
}
