#!/bin/bash
source /opt/ros/kilted/setup.bash
source install/setup.bash

echo "Clearing display..."
ros2 service call /sense_hat/led_matrix/clear std_srvs/srv/Trigger

echo "Setting corner pixels..."
ros2 service call /sense_hat/led_matrix/set_pixel ros2_pi_sense_hat/srv/SetPixel "{x: 0, y: 0, r: 31, g: 0, b: 0}"    # Red top-left
ros2 service call /sense_hat/led_matrix/set_pixel ros2_pi_sense_hat/srv/SetPixel "{x: 7, y: 0, r: 0, g: 31, b: 0}"    # Green top-right
ros2 service call /sense_hat/led_matrix/set_pixel ros2_pi_sense_hat/srv/SetPixel "{x: 0, y: 7, r: 0, g: 0, b: 31}"    # Blue bottom-left
ros2 service call /sense_hat/led_matrix/set_pixel ros2_pi_sense_hat/srv/SetPixel "{x: 7, y: 7, r: 31, g: 31, b: 0}"   # Yellow bottom-right

echo "Done!"
