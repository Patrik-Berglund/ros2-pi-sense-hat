#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class SensorFusionTest(Node):
    def __init__(self):
        super().__init__('sensor_fusion_test')
        
        # Subscribe to fused odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Subscribe to Madgwick-filtered IMU
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.get_logger().info('Sensor fusion test started')
        self.get_logger().info('Monitoring /odometry/filtered and /imu/data')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles for display
        roll, pitch, yaw = self.quaternion_to_euler(orient.x, orient.y, orient.z, orient.w)
        
        self.get_logger().info(
            f'🎯 FUSED POSE: '
            f'Pos[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] '
            f'Orient[R:{math.degrees(roll):.1f}° P:{math.degrees(pitch):.1f}° Y:{math.degrees(yaw):.1f}°]'
        )

    def imu_callback(self, msg):
        orient = msg.orientation
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(orient.x, orient.y, orient.z, orient.w)
        
        self.get_logger().info(
            f'🧭 MADGWICK IMU: '
            f'Orient[R:{math.degrees(roll):.1f}° P:{math.degrees(pitch):.1f}° Y:{math.degrees(yaw):.1f}°] '
            f'Valid: {orient.w != 0.0}'
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to roll, pitch, yaw in radians"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main():
    rclpy.init()
    
    test_node = SensorFusionTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
