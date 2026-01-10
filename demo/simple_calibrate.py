#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys
import time
import numpy as np

class SimpleIMUCalibration(Node):
    def __init__(self):
        super().__init__('imu_calibration')
        self.imu_sub = self.create_subscription(Imu, '/sense_hat/imu/data_raw', self.imu_callback, 10)
        self.samples = []
        self.get_logger().info('IMU Calibration started - collecting data...')

    def imu_callback(self, msg):
        self.samples.append({
            'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'gyro': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        })

    def calibrate_gyro(self):
        print("\n🎯 Gyroscope Calibration")
        print("📍 Keep IMU stationary for 10 seconds...")
        input("Press Enter to start...")
        
        self.samples.clear()
        start_time = time.time()
        
        while time.time() - start_time < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
            print(f"\r⏳ Samples: {len(self.samples)}", end="", flush=True)
        
        if len(self.samples) < 50:
            print("\n❌ Not enough samples")
            return None
        
        gyro_data = np.array([s['gyro'] for s in self.samples])
        bias = np.mean(gyro_data, axis=0)
        
        print(f"\n✅ Gyro bias: [{bias[0]:.6f}, {bias[1]:.6f}, {bias[2]:.6f}] rad/s")
        
        # Save calibration
        with open('imu_calibration.yaml', 'w') as f:
            f.write("# IMU Calibration\n")
            f.write(f"gyro_bias_x: {bias[0]}\n")
            f.write(f"gyro_bias_y: {bias[1]}\n")
            f.write(f"gyro_bias_z: {bias[2]}\n")
            f.write("gyro_calibrated: true\n")
        
        print("✅ Calibration saved to imu_calibration.yaml")
        return True

def main():
    rclpy.init()
    
    if len(sys.argv) < 2:
        print("Usage: python3 simple_calibrate.py gyro")
        return
    
    node = SimpleIMUCalibration()
    
    if sys.argv[1] == 'gyro':
        node.calibrate_gyro()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
