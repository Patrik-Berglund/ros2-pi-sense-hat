#!/bin/bash

# Test sensor fusion with robot_localization
cd /home/patrik/ros2-pi-sense-hat

# Source ROS2 environment
source /opt/ros/kilted/setup.bash
source install/setup.bash

echo "🚀 Starting sensor fusion test..."
echo "📊 This will fuse your calibrated IMU data into odometry"
echo ""

# Start EKF node with our config
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf_simple.yaml
