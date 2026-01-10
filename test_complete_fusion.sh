#!/bin/bash

# Complete sensor fusion test
cd /home/patrik/ros2-pi-sense-hat

# Source ROS2 environment
source /opt/ros/kilted/setup.bash
source install/setup.bash

echo "🚀 Starting complete sensor fusion test..."
echo "📊 Starting IMU node and EKF fusion"
echo ""

# Start IMU node in background
echo "Starting IMU node..."
ros2 run ros2_pi_sense_hat imu_node &
IMU_PID=$!

# Wait for IMU to start
sleep 3

# Start EKF node
echo "Starting EKF sensor fusion..."
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf_simple.yaml &
EKF_PID=$!

# Wait a bit then show topics
sleep 3
echo ""
echo "📡 Available topics:"
ros2 topic list | grep -E "(imu|odom|filter)"

echo ""
echo "🔄 Monitoring fused odometry for 10 seconds..."
timeout 10s ros2 topic echo /odometry/filtered --once

# Cleanup
kill $IMU_PID $EKF_PID 2>/dev/null
