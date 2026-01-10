#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class CalibrationTester(Node):
    def __init__(self):
        super().__init__('calibration_tester')
        self.imu_sub = self.create_subscription(Imu, '/sense_hat/imu/data_raw', self.imu_callback, 10)
        self.samples = []
        self.get_logger().info('Collecting IMU samples to check calibration...')

    def imu_callback(self, msg):
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        
        self.samples.append({'accel': accel, 'gyro': gyro})
        
        if len(self.samples) >= 50:
            self.analyze_calibration()
            rclpy.shutdown()

    def analyze_calibration(self):
        print("\n=== Calibration Analysis ===")
        
        # Calculate averages
        accel_avg = [0, 0, 0]
        gyro_avg = [0, 0, 0]
        
        for sample in self.samples:
            for i in range(3):
                accel_avg[i] += sample['accel'][i]
                gyro_avg[i] += sample['gyro'][i]
        
        for i in range(3):
            accel_avg[i] /= len(self.samples)
            gyro_avg[i] /= len(self.samples)
        
        print(f"Gyro averages (should be ~0 if calibrated): {gyro_avg}")
        print(f"Accel averages: {accel_avg}")
        print(f"Accel magnitude: {(accel_avg[0]**2 + accel_avg[1]**2 + accel_avg[2]**2)**0.5:.3f} m/s²")
        print("Expected: ~9.807 m/s² if properly calibrated")
        
        # Check if calibration is working
        gyro_bias = max(abs(g) for g in gyro_avg)
        if gyro_bias < 0.1:
            print("✅ Gyro calibration appears to be working")
        else:
            print("❌ Gyro calibration may not be applied")
            
        accel_mag = (accel_avg[0]**2 + accel_avg[1]**2 + accel_avg[2]**2)**0.5
        if 9.5 < accel_mag < 10.1:
            print("✅ Accel calibration appears to be working")
        else:
            print("❌ Accel calibration may not be applied")

def main():
    rclpy.init()
    tester = CalibrationTester()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
