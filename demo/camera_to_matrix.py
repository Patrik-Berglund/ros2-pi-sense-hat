#!/usr/bin/env python3
"""Capture camera image and display on LED matrix."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import numpy as np

class CameraToMatrix(Node):
    def __init__(self):
        super().__init__('camera_to_matrix')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10)
        
        # Publish to LED matrix
        self.matrix_pub = self.create_publisher(
            Image,
            '/sense_hat/led_matrix/image',
            10)
        
        self.get_logger().info('Camera to matrix node started')
    
    def camera_callback(self, msg):
        try:
            # Let cv_bridge handle the conversion to RGB
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            self.get_logger().info(f'Received image: {cv_image.shape}, encoding: {msg.encoding}, min: {cv_image.min()}, max: {cv_image.max()}')
            
            # Resize to 8x8
            small = cv2.resize(cv_image, (8, 8), interpolation=cv2.INTER_AREA)
            
            self.get_logger().info(f'Resized: {small.shape}, sample pixel: {small[0,0]}')
            
            # Create ROS Image message
            matrix_msg = self.bridge.cv2_to_imgmsg(small, encoding='rgb8')
            
            # Publish to matrix
            self.matrix_pub.publish(matrix_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main():
    rclpy.init()
    node = CameraToMatrix()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
