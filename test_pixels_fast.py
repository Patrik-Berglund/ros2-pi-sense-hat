#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/home/patrik/ros2-pi-sense-hat/install/ros2_pi_sense_hat/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from ros2_pi_sense_hat.srv import SetPixel
import time

class PixelTester(Node):
    def __init__(self):
        super().__init__('pixel_tester')
        self.client = self.create_client(SetPixel, '/sense_hat/led_matrix/set_pixel')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def set_pixel(self, x, y, r, g, b):
        request = SetPixel.Request()
        request.x = x
        request.y = y
        request.r = r
        request.g = g
        request.b = b
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = PixelTester()
    
    print("Setting corner pixels...")
    start = time.time()
    
    # Corner pixels
    node.set_pixel(0, 0, 31, 0, 0)    # Red top-left
    node.set_pixel(7, 0, 0, 31, 0)    # Green top-right
    node.set_pixel(0, 7, 0, 0, 31)    # Blue bottom-left
    node.set_pixel(7, 7, 31, 31, 0)   # Yellow bottom-right
    
    # Center cross
    node.set_pixel(3, 3, 31, 31, 31)  # White center
    node.set_pixel(4, 3, 31, 31, 31)
    node.set_pixel(3, 4, 31, 31, 31)
    node.set_pixel(4, 4, 31, 31, 31)
    
    elapsed = time.time() - start
    print(f"Set 8 pixels in {elapsed:.3f} seconds")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
