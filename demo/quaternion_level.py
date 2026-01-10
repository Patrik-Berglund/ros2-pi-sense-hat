#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import numpy as np
import math

class QuaternionLevelDisplay(Node):
    def __init__(self):
        super().__init__('quaternion_level_display')
        
        # Subscribe to fused odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Publisher for LED matrix
        self.image_pub = self.create_publisher(Image, '/sense_hat/led_matrix/image', 10)
        
        self.get_logger().info('🎨 Quaternion Level Display - no Euler angle conversion!')

    def odom_callback(self, msg):
        orient = msg.pose.pose.orientation
        
        # Work directly with quaternion - no Euler conversion!
        quat = np.array([orient.w, orient.x, orient.y, orient.z])  # [w, x, y, z]
        
        # Create level display using quaternion
        image = self.create_quaternion_level_display(quat)
        
        # Publish to LED matrix
        self.publish_image(image)

    def create_quaternion_level_display(self, quat):
        """Create level display directly from quaternion"""
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        
        # Center position
        center_x = 3.5
        center_y = 3.5
        
        # Extract tilt directly from quaternion components
        # For small angles: roll ≈ 2*quat[1], pitch ≈ 2*quat[2]
        roll_component = quat[1] * 8   # X component (roll)
        pitch_component = quat[2] * 8  # Y component (pitch)
        
        # Calculate bubble position
        bubble_x = int(center_x + pitch_component)
        bubble_y = int(center_y + roll_component)
        
        # Ensure bubble stays on display
        bubble_x = max(0, min(7, bubble_x))
        bubble_y = max(0, min(7, bubble_y))
        
        # Draw green bubble
        image[bubble_y, bubble_x] = [0, 255, 0]
        
        # Log quaternion components for debugging
        self.get_logger().info(f'Quat: w={quat[0]:.3f} x={quat[1]:.3f} y={quat[2]:.3f} z={quat[3]:.3f}')
        
        return image

    def publish_image(self, image_array):
        """Publish numpy array as ROS2 Image message"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "led_matrix"
        msg.height = 8
        msg.width = 8
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 24
        
        msg.data = image_array.flatten().tobytes()
        self.image_pub.publish(msg)

def main():
    rclpy.init()
    
    display = QuaternionLevelDisplay()
    
    try:
        rclpy.spin(display)
    except KeyboardInterrupt:
        pass
    
    display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
