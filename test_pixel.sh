#!/bin/bash
source /opt/ros/kilted/setup.bash

echo "Setting pixel at (3,3) to red (31,0,0)..."
ros2 param set /led_matrix_node pixel_x 3
ros2 param set /led_matrix_node pixel_y 3
ros2 param set /led_matrix_node pixel_r 31
ros2 param set /led_matrix_node pixel_g 0
ros2 param set /led_matrix_node pixel_b 0

echo "Done! Check the display."
