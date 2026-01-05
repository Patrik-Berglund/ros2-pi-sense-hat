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

include_directories(include)

add_executable(led_matrix_node 
  src/led_matrix_node.cpp 
  src/attiny88_driver.cpp 
  src/i2c_device.cpp
)

ament_target_dependencies(led_matrix_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  std_srvs
)

install(TARGETS
  led_matrix_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
