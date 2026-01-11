#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import sys
import time
import numpy as np
from collections import deque
from ellipsoid_fit import fit_ellipsoid, validate_calibration

class IMUCalibrationService(Node):
    def __init__(self):
        super().__init__('imu_calibration_service')
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, '/imu/mag', self.mag_callback, 10)
        
        self.imu_data = deque(maxlen=1000)
        self.mag_data = deque(maxlen=1000)
        self.get_logger().info('IMU Calibration Service started')

    def imu_callback(self, msg):
        data = {
            'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'gyro': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        }
        self.imu_data.append(data)

    def mag_callback(self, msg):
        data = {'mag': [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]}
        self.mag_data.append(data)

    def wait_for_data(self, min_samples=10):
        print(f"⏳ Waiting for IMU data...")
        while len(self.imu_data) < min_samples:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print(f"✅ Got {len(self.imu_data)} samples")

    def calibrate_gyroscope(self):
        print("\n🎯 Gyroscope Calibration")
        print("📍 Keep IMU completely stationary for 30 seconds")
        input("Press Enter to start...")
        
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < 30:
            if len(self.imu_data) > 0:
                samples.append(self.imu_data[-1]['gyro'])
            
            elapsed = time.time() - start_time
            progress = int((elapsed / 30) * 20)
            bar = "█" * progress + "░" * (20 - progress)
            print(f"\r⏳ [{bar}] {elapsed:.1f}s/30s", end="", flush=True)
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        print(f"\n✅ Collected {len(samples)} samples")
        
        if len(samples) < 50:
            print("❌ Insufficient samples")
            return None
        
        gyro_data = np.array(samples)
        bias = np.mean(gyro_data, axis=0)
        
        print(f"✅ Gyroscope calibration complete!")
        print(f"   Bias: [{bias[0]:.6f}, {bias[1]:.6f}, {bias[2]:.6f}] rad/s")
        
        return {
            'gyro_bias_x': float(bias[0]),
            'gyro_bias_y': float(bias[1]),
            'gyro_bias_z': float(bias[2]),
            'gyro_calibrated': True
        }

    def calibrate_accelerometer(self):
        print("\n🎯 Accelerometer Calibration")
        print("🔄 Slowly rotate IMU in ALL directions for 60 seconds")
        input("Press Enter to start...")
        
        samples = []
        start_time = time.time()
        
        print("🔄 ROTATE NOW!")
        
        while time.time() - start_time < 60:
            if len(self.imu_data) > 0:
                samples.append(self.imu_data[-1]['accel'])
            
            elapsed = time.time() - start_time
            progress = int((elapsed / 60) * 20)
            bar = "█" * progress + "░" * (20 - progress)
            print(f"\r🔄 [{bar}] {elapsed:.1f}s/60s", end="", flush=True)
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        print(f"\n✅ Collected {len(samples)} samples")
        
        if len(samples) < 200:
            print(f"❌ Need 200+ samples, got {len(samples)}")
            return None
        
        data = np.array(samples)
        g = 9.80665
        
        # Save raw data
        self.save_raw_data('accel', samples)
        
        print("🔄 Fitting ellipsoid...")
        offset, matrix, radii, residual = fit_ellipsoid(data)
        
        rms_error, max_error, mean_radius = validate_calibration(data, offset, matrix, g)
        
        print(f"\n✅ Accelerometer calibration complete!")
        print(f"   Offset: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}] m/s²")
        print(f"   Radii: [{radii[0]:.3f}, {radii[1]:.3f}, {radii[2]:.3f}] m/s²")
        print(f"   Expected: {g:.5f} m/s²")
        print(f"   Mean: {mean_radius:.5f} m/s²")
        print(f"   RMS error: {rms_error:.4f} m/s² ({rms_error/g*100:.2f}%)")
        print(f"   Max error: {max_error:.4f} m/s²")
        
        if rms_error > 0.1:
            print(f"⚠️  WARNING: High RMS error - consider recalibrating")
        
        return {
            'accel_offset_x': float(offset[0]),
            'accel_offset_y': float(offset[1]),
            'accel_offset_z': float(offset[2]),
            'accel_matrix': matrix.flatten().tolist(),
            'accel_calibrated': True
        }

    def calibrate_magnetometer(self):
        print("\n🧭 Magnetometer Calibration")
        print("🔄 Slowly rotate IMU in ALL directions for 60 seconds")
        input("Press Enter to start...")
        
        samples = []
        start_time = time.time()
        
        print("🔄 ROTATE NOW!")
        
        while time.time() - start_time < 60:
            if len(self.mag_data) > 0:
                samples.append(self.mag_data[-1]['mag'])
            
            elapsed = time.time() - start_time
            progress = int((elapsed / 60) * 20)
            bar = "█" * progress + "░" * (20 - progress)
            print(f"\r🔄 [{bar}] {elapsed:.1f}s/60s", end="", flush=True)
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        print(f"\n✅ Collected {len(samples)} samples")
        
        if len(samples) < 200:
            print(f"❌ Need 200+ samples, got {len(samples)}")
            return None
        
        data = np.array(samples)
        
        # Save raw data
        self.save_raw_data('mag', samples)
        
        print("🔄 Fitting ellipsoid...")
        offset, matrix, radii, residual = fit_ellipsoid(data)
        
        expected_radius = np.mean(radii)
        rms_error, max_error, mean_radius = validate_calibration(data, offset, matrix, expected_radius)
        
        print(f"\n✅ Magnetometer calibration complete!")
        print(f"   Offset: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}] T")
        print(f"   Radii: [{radii[0]:.6f}, {radii[1]:.6f}, {radii[2]:.6f}] T")
        print(f"   Field strength: {expected_radius:.6f} T")
        print(f"   RMS error: {rms_error:.6f} T ({rms_error/expected_radius*100:.2f}%)")
        
        if rms_error/expected_radius > 0.02:
            print(f"⚠️  WARNING: High RMS error - consider recalibrating")
        
        return {
            'mag_offset_x': float(offset[0]),
            'mag_offset_y': float(offset[1]),
            'mag_offset_z': float(offset[2]),
            'mag_matrix': matrix.flatten().tolist(),
            'mag_calibrated': True,
            'mag_field_strength': float(expected_radius)
        }

    def save_calibration(self, cal_data, filename='imu_calibration.yaml'):
        try:
            with open(filename, 'w') as f:
                f.write("# IMU Calibration Data\n")
                for key, value in cal_data.items():
                    if isinstance(value, bool):
                        f.write(f"{key}: {'true' if value else 'false'}\n")
                    elif isinstance(value, list):
                        f.write(f"{key}: {value}\n")
                    else:
                        f.write(f"{key}: {value}\n")
            
            print(f"✅ Calibration saved to {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to save: {e}")
            return False
    
    def save_raw_data(self, sensor_type, samples, filename=None):
        """Save raw calibration data for debugging"""
        if filename is None:
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"raw_{sensor_type}_data_{timestamp}.npy"
        
        try:
            np.save(filename, np.array(samples))
            print(f"📊 Raw data saved to {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to save raw data: {e}")
            return False

def main():
    rclpy.init()
    
    service = IMUCalibrationService()
    service.wait_for_data()
    
    if len(sys.argv) < 2:
        print("\nIMU Calibration Service")
        print("Usage: python3 scripts/calibrate_imu.py [gyro|accel|mag|all]")
        print("\nCalibrations:")
        print("  gyro  - Gyroscope bias (30s stationary)")
        print("  accel - Accelerometer ellipsoid fit (60s rotation)")
        print("  mag   - Magnetometer ellipsoid fit (60s rotation)")
        print("  all   - All three in sequence")
        return
    
    mode = sys.argv[1].lower()
    cal_data = {}
    
    if mode in ['gyro', 'all']:
        result = service.calibrate_gyroscope()
        if result:
            cal_data.update(result)
    
    if mode in ['accel', 'all']:
        result = service.calibrate_accelerometer()
        if result:
            cal_data.update(result)
    
    if mode in ['mag', 'all']:
        result = service.calibrate_magnetometer()
        if result:
            cal_data.update(result)
    
    if cal_data:
        service.save_calibration(cal_data)
        print(f"\n🎉 Calibration complete! Restart IMU node to apply.")
    else:
        print("\n❌ No calibration data")
    
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
