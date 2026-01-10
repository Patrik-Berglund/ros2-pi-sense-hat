#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
import time

class IMUConfigTest(Node):
    def __init__(self):
        super().__init__('imu_config_test')
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/sense_hat/imu/data_raw', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/sense_hat/imu/mag', self.mag_callback, 10)
        self.temp_sub = self.create_subscription(Temperature, '/sense_hat/temperature/imu', self.temp_callback, 10)
        
        # Data counters
        self.imu_count = 0
        self.mag_count = 0
        self.temp_count = 0
        self.start_time = time.time()
        
        # Create timer for periodic reporting
        self.timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info("IMU Configuration Test started - monitoring data rates...")
    
    def imu_callback(self, msg):
        self.imu_count += 1
        if self.imu_count == 1:
            self.get_logger().info(f"First IMU message received:")
            self.get_logger().info(f"  Frame: {msg.header.frame_id}")
            self.get_logger().info(f"  Accel: [{msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}] m/s²")
            self.get_logger().info(f"  Gyro:  [{msg.angular_velocity.x:.3f}, {msg.angular_velocity.y:.3f}, {msg.angular_velocity.z:.3f}] rad/s")
    
    def mag_callback(self, msg):
        self.mag_count += 1
        if self.mag_count == 1:
            self.get_logger().info(f"First Magnetometer message received:")
            self.get_logger().info(f"  Frame: {msg.header.frame_id}")
            self.get_logger().info(f"  Mag:   [{msg.magnetic_field.x:.6f}, {msg.magnetic_field.y:.6f}, {msg.magnetic_field.z:.6f}] T")
    
    def temp_callback(self, msg):
        self.temp_count += 1
        if self.temp_count == 1:
            self.get_logger().info(f"First Temperature message received:")
            self.get_logger().info(f"  Frame: {msg.header.frame_id}")
            self.get_logger().info(f"  Temp:  {msg.temperature:.2f} °C")
    
    def report_status(self):
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            imu_rate = self.imu_count / elapsed
            mag_rate = self.mag_count / elapsed
            temp_rate = self.temp_count / elapsed
            
            self.get_logger().info(f"Data rates after {elapsed:.1f}s:")
            self.get_logger().info(f"  IMU:  {imu_rate:.1f} Hz ({self.imu_count} messages)")
            self.get_logger().info(f"  Mag:  {mag_rate:.1f} Hz ({self.mag_count} messages)")
            self.get_logger().info(f"  Temp: {temp_rate:.1f} Hz ({self.temp_count} messages)")

def main(args=None):
    rclpy.init(args=args)
    node = IMUConfigTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
