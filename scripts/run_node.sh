#!/bin/bash
cd /home/patrik/ros2-pi-sense-hat
source /opt/ros/kilted/setup.bash
source install/setup.bash

echo "Starting camera node..."
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -p output_encoding:=rgb8 &
sleep 2  # Wait for camera to initialize

echo "Starting LED matrix node..."
ros2 run ros2_pi_sense_hat led_matrix_node &
sleep 2  # Wait for LED matrix to finish I2C initialization

echo "Starting joystick node..."
ros2 run ros2_pi_sense_hat joystick_node &
sleep 2  # Wait for joystick to finish I2C initialization

echo "Starting IMU node..."
ros2 run ros2_pi_sense_hat imu_node &
sleep 2  # Wait for IMU to start publishing

echo "Starting environmental sensors node..."
ros2 run ros2_pi_sense_hat environmental_node &
sleep 2  # Wait for environmental sensors to initialize

echo "Starting color sensor node..."
ros2 run ros2_pi_sense_hat color_node &
sleep 2  # Wait for color sensor to initialize

echo "Starting IMU orientation filter (Madgwick)..."
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args --params-file config/madgwick.yaml &
sleep 2  # Wait for filter to start

echo "Starting sensor fusion (EKF)..."
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf_minimal.yaml &

echo "All nodes started. Press Ctrl+C to stop all nodes."
wait
