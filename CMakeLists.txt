cmake_minimum_required(VERSION 3.8)
project(ros2_pi_sense_hat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPixel.srv"
  "srv/ReadJoystick.srv"
  "action/I2CTransaction.action"
)

include_directories(include)

add_executable(i2c_bridge_node
  src/i2c_bridge_node.cpp
  src/i2c_device.cpp
)

add_executable(led_matrix_node 
  src/led_matrix_node.cpp 
  src/attiny88_driver.cpp
)

add_executable(joystick_node 
  src/joystick_node.cpp 
  src/attiny88_driver.cpp
)

add_executable(imu_node
  src/imu_node.cpp
  src/lsm9ds1_driver.cpp
)

target_link_libraries(i2c_bridge_node PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

target_link_libraries(led_matrix_node PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  gpiod
)

target_link_libraries(joystick_node PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  gpiod
)

target_link_libraries(imu_node PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(i2c_bridge_node PUBLIC
  "${cpp_typesupport_target}"
)

target_link_libraries(led_matrix_node PUBLIC
  "${cpp_typesupport_target}"
)

target_link_libraries(joystick_node PUBLIC
  "${cpp_typesupport_target}"
)

target_link_libraries(imu_node PUBLIC
  "${cpp_typesupport_target}"
)

install(TARGETS
  i2c_bridge_node
  led_matrix_node
  joystick_node
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
