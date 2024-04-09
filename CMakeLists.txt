cmake_minimum_required(VERSION 3.8)
project(bear_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED) # hardware_interface 패키지를 찾도록 추가
find_package(pluginlib REQUIRED)


include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/src/CBEAR/include)


# Add executable
add_executable(ping src/test.cpp 
src/CBEAR/src/bear_sdk.cpp src/CBEAR/src/packet_manager.cpp src/CBEAR/src/port_manager.cpp)

add_executable(bulk_read src/bulk_read.cpp
src/CBEAR/src/bear_sdk.cpp src/CBEAR/src/packet_manager.cpp src/CBEAR/src/port_manager.cpp)

add_executable(bulk_write src/bulk_write.cpp
src/CBEAR/src/bear_sdk.cpp src/CBEAR/src/packet_manager.cpp src/CBEAR/src/port_manager.cpp)

add_executable(to src/to.cpp
src/CBEAR/src/bear_sdk.cpp src/CBEAR/src/packet_manager.cpp src/CBEAR/src/port_manager.cpp)

add_executable(pid src/pid.cpp
src/CBEAR/src/bear_sdk.cpp src/CBEAR/src/packet_manager.cpp src/CBEAR/src/port_manager.cpp)

add_executable(motor_reader_node src/motor_reader_node.cpp)
ament_target_dependencies(motor_reader_node rclcpp hardware_interface pluginlib)
target_link_libraries(motor_reader_node bear_hardware)


add_library(bear_hardware SHARED
  src/bear_hardware.cpp
  src/CBEAR/src/bear_sdk.cpp
  src/CBEAR/src/packet_manager.cpp
  src/CBEAR/src/port_manager.cpp
)


# Include directories and link dependencies for ping and bulk_read
foreach(target ping bulk_read bulk_write to pid bear_hardware )
  target_include_directories(${target} PUBLIC
    include
    src/CBEAR/include
    include/bear_hardware


  )

  ament_target_dependencies(${target}
    rclcpp
    hardware_interface
    pluginlib

  )
endforeach()



# Install executables
install(TARGETS
  ping
  bulk_read
  bulk_write
  to
  pid
  bear_hardware
  motor_reader_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

