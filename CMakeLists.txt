cmake_minimum_required(VERSION 3.8)
project(ros2_pi_sense_hat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPixel.srv"
  "srv/ReadJoystick.srv"
)

include_directories(include)

add_executable(led_matrix_node 
  src/led_matrix_node.cpp 
  src/attiny88_driver.cpp 
  src/i2c_device.cpp
)

add_executable(joystick_node 
  src/joystick_node.cpp 
  src/attiny88_driver.cpp 
  src/i2c_device.cpp
)

add_executable(imu_node
  src/imu_node.cpp
  src/lsm9ds1_driver.cpp
  src/imu_calibration.cpp
  src/i2c_device.cpp
)

add_executable(environmental_node
  src/environmental_node.cpp
  src/hts221_driver.cpp
  src/lps25h_driver.cpp
  src/i2c_device.cpp
)

add_executable(color_node
  src/color_node.cpp
  src/tcs3400_driver.cpp
  src/i2c_device.cpp
)

target_link_libraries(led_matrix_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  gpiod
)

target_link_libraries(joystick_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  gpiod
)

target_link_libraries(imu_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

target_link_libraries(environmental_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

target_link_libraries(color_node PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(led_matrix_node PUBLIC
  "${cpp_typesupport_target}"
)

target_link_libraries(joystick_node PUBLIC
  "${cpp_typesupport_target}"
)

target_link_libraries(imu_node PUBLIC
  "${cpp_typesupport_target}"
)

target_link_libraries(environmental_node PUBLIC
  "${cpp_typesupport_target}"
)

target_link_libraries(color_node PUBLIC
  "${cpp_typesupport_target}"
)

install(TARGETS
  led_matrix_node
  joystick_node
  imu_node
  environmental_node
  color_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
