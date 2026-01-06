#!/bin/bash
cd /home/patrik/ros2-pi-sense-hat
source /opt/ros/kilted/setup.bash
source install/setup.bash

echo "Starting I2C Bridge node..."
ros2 run ros2_pi_sense_hat i2c_bridge_node &
sleep 1  # Wait for I2C bridge to start

echo "Starting LED matrix node..."
ros2 run ros2_pi_sense_hat led_matrix_node &
sleep 1

echo "Starting joystick node..."
ros2 run ros2_pi_sense_hat joystick_node &
sleep 1

echo "Starting IMU node..."
ros2 run ros2_pi_sense_hat imu_node &

wait
