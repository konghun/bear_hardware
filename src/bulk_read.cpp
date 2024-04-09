#include "rclcpp/rclcpp.hpp"
#include "bear_sdk.h"
#include "bear_macro.h"


using namespace bear;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger = rclcpp::get_logger("motor_bulk_read_node");

    const char* portName = "/dev/ttyUSB0";
    int baudrate = 7500000; // 샘플 코드에 맞춰서 baudrate 설정

    BEAR bear(portName, baudrate);

    // BulkReadWrite를 위한 설정
    std::vector<uint8_t> mIDs = {1}; // 1번 모터의 ID
    std::vector<uint8_t> read_add = {
        bear_macro::PRESENT_POSITION, 
        bear_macro::PRESENT_VELOCITY, 
        bear_macro::PRESENT_IQ
    };
    std::vector<uint8_t> write_add; // 쓰기 연산이 필요 없으면 비워둡니다.
    std::vector<std::vector<float>> data; // 쓰기 데이터가 필요 없으면 비워둡니다.

    // BulkReadWrite 연산 수행
    auto ret_vec_rw = bear.BulkReadWrite(mIDs, read_add, write_add, data);

    // 결과 출력
    for (const auto& motor_results : ret_vec_rw) {
        for (const auto& value : motor_results) {
            RCLCPP_INFO(logger, "Read Value: %f", value);
        }
    }

    rclcpp::shutdown();
    return 0;
}
