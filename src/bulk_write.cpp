#include "rclcpp/rclcpp.hpp"
#include "bear_sdk.h"
#include "bear_macro.h"

using namespace bear;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger = rclcpp::get_logger("motor_bulk_write_node");

    const char* portName = "/dev/ttyUSB0";
    int baudrate = 7500000; // 샘플 코드와 일치하는 baudrate 설정

    BEAR bear(portName, baudrate);

    // BulkWrite를 위한 설정
    std::vector<uint8_t> mIDs = {1}; // 1번 모터의 ID
    std::vector<uint8_t> write_add = {
        bear_macro::GOAL_POSITION, 
        bear_macro::GOAL_VELOCITY
    };

    // 목표 위치와 속도 설정 (예: 위치 1.0, 속도 0.5)
    std::vector<std::vector<float>> data = {{0.7, 0.3}};

    // BulkWrite 연산 수행
    bool success = bear.BulkWrite(mIDs, write_add, data);

    if (success) {
        RCLCPP_INFO(logger, "BulkWrite operation successful.");
    } else {
        RCLCPP_ERROR(logger, "BulkWrite operation failed.");
    }

    rclcpp::shutdown();
    return 0;
}
