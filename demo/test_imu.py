#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature, MagneticField

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_test')
        
        self.imu_sub = self.create_subscription(Imu, '/sense_hat/imu/data_raw', self.imu_callback, 10)
        self.temp_sub = self.create_subscription(Temperature, '/sense_hat/temperature/imu', self.temp_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/sense_hat/imu/mag', self.mag_callback, 10)
        
        self.get_logger().info('IMU test subscriber started')

    def imu_callback(self, msg):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self.get_logger().info(f'Accel: [{a.x:.2f}, {a.y:.2f}, {a.z:.2f}] m/s² | Gyro: [{g.x:.2f}, {g.y:.2f}, {g.z:.2f}] rad/s')

    def temp_callback(self, msg):
        self.get_logger().info(f'Temperature: {msg.temperature:.1f}°C')

    def mag_callback(self, msg):
        m = msg.magnetic_field
        self.get_logger().info(f'Magnetometer: [{m.x:.6f}, {m.y:.6f}, {m.z:.6f}] T')

def main():
    rclpy.init()
    node = IMUSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
