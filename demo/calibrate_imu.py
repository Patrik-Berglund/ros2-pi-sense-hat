#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys
import time
import numpy as np
from collections import deque
import yaml

class IMUCalibrationService(Node):
    def __init__(self):
        super().__init__('imu_calibration_service')
        
        # Subscribe to raw IMU data
        self.imu_sub = self.create_subscription(
            Imu, '/sense_hat/imu/data_raw', self.imu_callback, 10)
        
        self.imu_data = deque(maxlen=1000)
        self.get_logger().info('IMU Calibration Service started')
        self.get_logger().info('Collecting IMU data...')

    def imu_callback(self, msg):
        """Store incoming IMU data"""
        data = {
            'timestamp': time.time(),
            'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'gyro': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        }
        self.imu_data.append(data)

    def wait_for_data(self, min_samples=10):
        """Wait for sufficient IMU data"""
        print(f"⏳ Waiting for IMU data...")
        while len(self.imu_data) < min_samples:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print(f"✅ Got {len(self.imu_data)} samples")

    def collect_samples(self, duration_sec, sample_rate=10):
        """Collect samples for specified duration"""
        samples = []
        start_time = time.time()
        
        print(f"📊 Collecting samples for {duration_sec} seconds...")
        
        while time.time() - start_time < duration_sec:
            if len(self.imu_data) > 0:
                samples.append(dict(self.imu_data[-1]))  # Get latest sample
            
            # Progress indicator
            elapsed = time.time() - start_time
            progress = int((elapsed / duration_sec) * 20)
            bar = "█" * progress + "░" * (20 - progress)
            print(f"\r⏳ [{bar}] {elapsed:.1f}s/{duration_sec}s", end="", flush=True)
            
            rclpy.spin_once(self, timeout_sec=1.0/sample_rate)
            time.sleep(1.0/sample_rate)
        
        print(f"\n✅ Collected {len(samples)} samples")
        return samples

    def calibrate_gyroscope(self, sample_count=1000):
        """Calibrate gyroscope bias"""
        print("\n🎯 Starting gyroscope calibration...")
        print("📍 Keep IMU completely stationary!")
        
        input("Press Enter when ready...")
        
        # Collect stationary samples
        samples = self.collect_samples(30)  # 30 seconds
        
        if len(samples) < 50:  # Just need reasonable number of samples
            print("❌ Insufficient samples collected")
            return None
        
        # Calculate bias (average)
        gyro_data = np.array([s['gyro'] for s in samples])
        bias = np.mean(gyro_data, axis=0)
        
        print(f"✅ Gyroscope calibration complete!")
        print(f"   Bias: X={bias[0]:.6f} Y={bias[1]:.6f} Z={bias[2]:.6f} rad/s")
        
        return {
            'gyro_bias_x': float(bias[0]),
            'gyro_bias_y': float(bias[1]),
            'gyro_bias_z': float(bias[2]),
            'gyro_calibrated': True
        }

    def calibrate_accelerometer(self):
        """Calibrate accelerometer using 6-point method"""
        print("\n🎯 Starting accelerometer calibration (6-point method)...")
        print("📋 You will position the IMU in 6 orientations:")
        
        orientations = [
            ("+X up", "X-axis pointing up"),
            ("-X up", "X-axis pointing down"), 
            ("+Y up", "Y-axis pointing up"),
            ("-Y up", "Y-axis pointing down"),
            ("+Z up", "Z-axis pointing up"),
            ("-Z up", "Z-axis pointing down")
        ]
        
        measurements = []
        
        for i, (name, desc) in enumerate(orientations):
            print(f"\n📐 Position {i+1}/6: {name} ({desc})")
            input("   Press Enter when positioned...")
            
            samples = self.collect_samples(5)  # 5 seconds per orientation
            
            if len(samples) < 10:
                print(f"❌ Failed to collect samples for {name}")
                return None
            
            # Average the samples
            accel_data = np.array([s['accel'] for s in samples])
            avg = np.mean(accel_data, axis=0)
            measurements.append(avg)
            
            print(f"✅ {name}: [{avg[0]:.3f}, {avg[1]:.3f}, {avg[2]:.3f}] m/s²")
        
        # Calculate calibration using 6-point method
        g = 9.80665  # Standard gravity
        
        # X-axis: measurements[0] = +X up, measurements[1] = -X up
        x_offset = (measurements[0][0] + measurements[1][0]) / 2.0
        x_scale = (2.0 * g) / (measurements[0][0] - measurements[1][0])
        
        # Y-axis: measurements[2] = +Y up, measurements[3] = -Y up  
        y_offset = (measurements[2][1] + measurements[3][1]) / 2.0
        y_scale = (2.0 * g) / (measurements[2][1] - measurements[3][1])
        
        # Z-axis: measurements[4] = +Z up, measurements[5] = -Z up
        z_offset = (measurements[4][2] + measurements[5][2]) / 2.0
        z_scale = (2.0 * g) / (measurements[4][2] - measurements[5][2])
        
        print(f"\n✅ Accelerometer calibration complete!")
        print(f"   X: offset={x_offset:.6f} scale={x_scale:.6f}")
        print(f"   Y: offset={y_offset:.6f} scale={y_scale:.6f}")
        print(f"   Z: offset={z_offset:.6f} scale={z_scale:.6f}")
        
        return {
            'accel_offset_x': float(x_offset),
            'accel_offset_y': float(y_offset),
            'accel_offset_z': float(z_offset),
            'accel_scale_x': float(x_scale),
            'accel_scale_y': float(y_scale),
            'accel_scale_z': float(z_scale),
            'accel_calibrated': True
        }

    def save_calibration(self, cal_data, filename='imu_calibration.yaml'):
        """Save calibration data to YAML file, merging with existing data"""
        # Load existing calibration data first
        existing_data = {}
        try:
            with open(filename, 'r') as f:
                for line in f:
                    if ':' in line and not line.strip().startswith('#'):
                        key, value = line.strip().split(':', 1)
                        key = key.strip()
                        value = value.strip()
                        if value in ['true', 'false']:
                            existing_data[key] = (value == 'true')
                        else:
                            try:
                                existing_data[key] = float(value)
                            except ValueError:
                                existing_data[key] = value
        except FileNotFoundError:
            pass  # No existing file, start fresh
        
        # Merge new data with existing data
        existing_data.update(cal_data)
        
        # Save merged data
        try:
            with open(filename, 'w') as f:
                f.write("# IMU Calibration Data\n")
                for key, value in existing_data.items():
                    if isinstance(value, bool):
                        f.write(f"{key}: {'true' if value else 'false'}\n")
                    else:
                        f.write(f"{key}: {value}\n")
            
            print(f"✅ Calibration saved to {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to save calibration: {e}")
            return False

def main():
    rclpy.init()
    
    service = IMUCalibrationService()
    
    # Wait for IMU data
    service.wait_for_data()
    
    if len(sys.argv) < 2:
        print("\nIMU Calibration Service")
        print("Usage: python3 calibrate_imu.py [gyro|accel|all]")
        print("\nAvailable calibrations:")
        print("  gyro  - Gyroscope bias calibration")
        print("  accel - Accelerometer 6-point calibration") 
        print("  all   - Full calibration (gyro + accel)")
        return
    
    mode = sys.argv[1].lower()
    cal_data = {}
    
    if mode in ['gyro', 'all']:
        gyro_cal = service.calibrate_gyroscope()
        if gyro_cal:
            cal_data.update(gyro_cal)
    
    if mode in ['accel', 'all']:
        accel_cal = service.calibrate_accelerometer()
        if accel_cal:
            cal_data.update(accel_cal)
    
    if cal_data:
        service.save_calibration(cal_data)
        print(f"\n🎉 Calibration complete! Restart IMU node to load new calibration.")
    else:
        print("\n❌ No calibration data to save")
    
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
