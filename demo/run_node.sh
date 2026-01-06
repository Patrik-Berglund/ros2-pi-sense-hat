#!/bin/bash
cd /home/patrik/ros2-pi-sense-hat
source /opt/ros/kilted/setup.bash
source install/setup.bash

echo "Starting LED matrix node..."
ros2 run ros2_pi_sense_hat led_matrix_node
