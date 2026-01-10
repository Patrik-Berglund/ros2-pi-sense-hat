#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import numpy as np
import math

class SimpleOrientationDisplay(Node):
    def __init__(self):
        super().__init__('simple_orientation_display')
        
        # Subscribe to fused odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Publisher for LED matrix
        self.image_pub = self.create_publisher(Image, '/sense_hat/led_matrix/image', 10)
        
        # Store previous angles for unwrapping
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        
        self.get_logger().info('🎨 Simple Level Display - tilt to see the bubble move!')

    def odom_callback(self, msg):
        orient = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(orient.x, orient.y, orient.z, orient.w)
        
        # Unwrap angles to prevent jumps
        roll = self.unwrap_angle(roll, self.prev_roll)
        pitch = self.unwrap_angle(pitch, self.prev_pitch)
        
        # Store for next iteration
        self.prev_roll = roll
        self.prev_pitch = pitch
        
        # Create simple level bubble display
        image = self.create_level_display(roll, pitch)
        
        # Publish to LED matrix
        self.publish_image(image)
        
        # Log the angles for debugging
        self.get_logger().info(f'Roll: {math.degrees(roll):.1f}° Pitch: {math.degrees(pitch):.1f}°')

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

    def unwrap_angle(self, angle, prev_angle):
        """Unwrap angle to prevent jumps at ±180°"""
        diff = angle - prev_angle
        
        # If difference is > 180°, we wrapped around
        if diff > math.pi:
            angle -= 2 * math.pi
        elif diff < -math.pi:
            angle += 2 * math.pi
        
        # Reset if angle gets too extreme (accumulated error)
        if abs(angle) > 4 * math.pi:  # More than 2 full rotations
            angle = angle % (2 * math.pi)
            if angle > math.pi:
                angle -= 2 * math.pi
            
        return angle

    def create_level_display(self, roll, pitch):
        """Create 8x8 level bubble display"""
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        
        # Center position (4,4 is center of 8x8)
        center_x = 3.5
        center_y = 3.5
        
        # Convert angles to pixel offsets (simple scaling)
        roll_offset = max(-3, min(3, roll * 2))    # Roll moves up/down (Y)
        pitch_offset = max(-3, min(3, pitch * 2))  # Pitch moves left/right (X)
        
        # Calculate bubble position (swap X/Y to match display orientation)
        bubble_x = int(center_x + pitch_offset)  # Pitch controls X movement
        bubble_y = int(center_y + roll_offset)   # Roll controls Y movement
        
        # Ensure bubble stays on display
        bubble_x = max(0, min(7, bubble_x))
        bubble_y = max(0, min(7, bubble_y))
        
        # Draw green bubble (level indicator)
        image[bubble_y, bubble_x] = [0, 255, 0]  # Bright green bubble
        
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
    
    display = SimpleOrientationDisplay()
    
    try:
        rclpy.spin(display)
    except KeyboardInterrupt:
        pass
    
    display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
