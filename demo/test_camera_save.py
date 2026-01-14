#!/usr/bin/env python3
"""Test camera by saving a frame."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        self.bridge = CvBridge()
        self.saved = False
        
        self.camera_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10)
        
        self.get_logger().info('Waiting for camera image...')
    
    def camera_callback(self, msg):
        if not self.saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.imwrite('/tmp/camera_test.jpg', cv_image)
                self.get_logger().info(f'Saved image: {cv_image.shape}, encoding: {msg.encoding}')
                self.saved = True
            except Exception as e:
                self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    node = CameraTest()
    
    while not node.saved:
        rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()
    print('Image saved to /tmp/camera_test.jpg')

if __name__ == '__main__':
    main()
