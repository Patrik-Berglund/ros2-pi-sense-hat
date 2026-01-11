#!/usr/bin/env python3
"""
Verify IMU calibration quality by checking measurement consistency.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np

class CalibrationVerifier(Node):
    def __init__(self):
        super().__init__('calibration_verifier')
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, '/imu/mag', self.mag_callback, 10)
        
        self.accel_errors = []
        self.accel_batch = []  # Last 10 samples for batch stats
        self.mag_samples = []
        
    def imu_callback(self, msg):
        acc = msg.linear_acceleration
        norm = np.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
        error = abs(norm - 9.80665)
        
        self.accel_errors.append(error)
        self.accel_batch.append(error)
        
        if len(self.accel_batch) > 10:
            self.accel_batch.pop(0)
        
        if len(self.accel_errors) % 10 == 0:
            # Overall stats
            rms_all = np.sqrt(np.mean(np.array(self.accel_errors)**2))
            max_all = np.max(self.accel_errors)
            
            # Batch stats (last 10 samples)
            rms_batch = np.sqrt(np.mean(np.array(self.accel_batch)**2))
            max_batch = np.max(self.accel_batch)
            
            print(f"Accel - Samples: {len(self.accel_errors):4d} | "
                  f"Batch RMS: {rms_batch:.4f} m/s² ({rms_batch/9.80665*100:.2f}%) | "
                  f"Batch Max: {max_batch:.4f} m/s² | "
                  f"Overall RMS: {rms_all:.4f} m/s² ({rms_all/9.80665*100:.2f}%)")
    
    def mag_callback(self, msg):
        mag = msg.magnetic_field
        self.mag_samples.append([mag.x, mag.y, mag.z])
        
        if len(self.mag_samples) >= 100:
            mag_data = np.array(self.mag_samples)
            norms = np.linalg.norm(mag_data, axis=1)
            mean_norm = np.mean(norms)
            std_norm = np.std(norms)
            
            print(f"Mag - Samples: {len(self.mag_samples):4d} | "
                  f"Mean: {mean_norm:.6f} T | "
                  f"Std: {std_norm:.6f} T ({std_norm/mean_norm*100:.2f}%)")
            
            self.mag_samples = []

def main():
    rclpy.init()
    node = CalibrationVerifier()
    print("Verifying IMU calibration...")
    print("Rotate IMU through different orientations\n")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
