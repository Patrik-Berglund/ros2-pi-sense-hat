#!/bin/bash
source /opt/ros/kilted/setup.bash

echo "Clearing display..."
ros2 service call /sense_hat/led_matrix/clear std_srvs/srv/Trigger

echo "Setting pixel at (3,3) to red..."
ros2 service call /sense_hat/led_matrix/set_pixel ros2_pi_sense_hat/srv/SetPixel "{x: 3, y: 3, r: 31, g: 0, b: 0}"

echo "Done!"
