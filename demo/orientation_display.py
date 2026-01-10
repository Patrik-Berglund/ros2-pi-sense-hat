#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import numpy as np
import math

class OrientationDisplay(Node):
    def __init__(self):
        super().__init__('orientation_display')
        
        # Subscribe to fused odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Publisher for LED matrix
        self.image_pub = self.create_publisher(Image, '/sense_hat/led_matrix/image', 10)
        
        self.get_logger().info('🎨 Orientation Display started - showing attitude on LED matrix')

    def odom_callback(self, msg):
        orient = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(orient.x, orient.y, orient.z, orient.w)
        
        # Create 8x8 RGB image showing orientation
        image = self.create_orientation_image(roll, pitch, yaw)
        
        # Publish to LED matrix
        self.publish_image(image)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to roll, pitch, yaw in radians"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def create_orientation_image(self, roll, pitch, yaw):
        """Create 8x8 RGB image showing orientation"""
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        
        # Center of display
        cx, cy = 3.5, 3.5
        
        # Roll indicator (red line)
        roll_deg = math.degrees(roll)
        roll_x = int(cx + 2 * math.cos(roll))
        roll_y = int(cy + 2 * math.sin(roll))
        if 0 <= roll_x < 8 and 0 <= roll_y < 8:
            image[roll_y, roll_x] = [255, 0, 0]  # Red
        
        # Pitch indicator (green line)  
        pitch_deg = math.degrees(pitch)
        pitch_x = int(cx + 2 * math.cos(pitch + math.pi/2))
        pitch_y = int(cy + 2 * math.sin(pitch + math.pi/2))
        if 0 <= pitch_x < 8 and 0 <= pitch_y < 8:
            image[pitch_y, pitch_x] = [0, 255, 0]  # Green
        
        # Yaw indicator (blue center dot intensity)
        yaw_intensity = int(128 + 127 * math.sin(yaw))
        image[3:5, 3:5] = [0, 0, yaw_intensity]  # Blue center
        
        # Add corner reference dots
        image[0, 0] = [50, 50, 50]  # Gray corner markers
        image[0, 7] = [50, 50, 50]
        image[7, 0] = [50, 50, 50]
        image[7, 7] = [50, 50, 50]
        
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
        msg.step = 24  # 8 pixels * 3 bytes per pixel
        
        # Flatten and convert to bytes
        msg.data = image_array.flatten().tobytes()
        
        self.image_pub.publish(msg)

def main():
    rclpy.init()
    
    display = OrientationDisplay()
    
    try:
        rclpy.spin(display)
    except KeyboardInterrupt:
        pass
    
    display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
