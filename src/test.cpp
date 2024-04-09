#include "rclcpp/rclcpp.hpp"
#include "bear_sdk.h"  // bear 네임스페이스와 BEAR 클래스 포함

class MotorPingNode : public rclcpp::Node {
public:
  MotorPingNode() : Node("motor_ping_node") {
    // 포트 이름과 보레이트 설정
    const char* portName = "/dev/ttyUSB0"; 
    int baudrate = 7500000; 

    // bear 네임스페이스 내의 BEAR 클래스 사용
    bear = std::make_unique<bear::BEAR>(portName, baudrate);

    // 모터 ID 1에 ping 보내기
    auto success = bear->ping(1);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Ping successful.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to ping motor.");
    }
  }

private:
  // std::unique_ptr로 BEAR 객체 관리
  std::unique_ptr<bear::BEAR> bear;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorPingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
