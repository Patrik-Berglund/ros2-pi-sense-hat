#!/bin/bash
cd /home/patrik/ros2-pi-sense-hat
source /opt/ros/kilted/setup.bash
source install/setup.bash

echo "Starting LED matrix node..."
ros2 run ros2_pi_sense_hat led_matrix_node &
sleep 3  # Wait for LED matrix to finish I2C initialization

echo "Starting joystick node..."
ros2 run ros2_pi_sense_hat joystick_node &
sleep 2  # Wait for joystick to finish I2C initialization

echo "Starting IMU node..."
ros2 run ros2_pi_sense_hat imu_node &

wait
