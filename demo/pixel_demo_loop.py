#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/home/patrik/ros2-pi-sense-hat/install/ros2_pi_sense_hat/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from ros2_pi_sense_hat.srv import SetPixel
import time
import math
import random

class PixelDemo(Node):
    def __init__(self):
        super().__init__('pixel_demo')
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

    def clear(self):
        for y in range(8):
            for x in range(8):
                self.set_pixel(x, y, 0, 0, 0)

    def spinning_wheel(self, frame):
        self.clear()
        center_x, center_y = 3.5, 3.5
        for i in range(8):
            angle = (frame * 0.3 + i * 0.8) % (2 * math.pi)
            x = int(center_x + 2.5 * math.cos(angle))
            y = int(center_y + 2.5 * math.sin(angle))
            if 0 <= x < 8 and 0 <= y < 8:
                intensity = int(31 * (i / 8))
                self.set_pixel(x, y, intensity, 0, 31 - intensity)

    def bouncing_ball(self, frame):
        self.clear()
        x = int(3.5 + 3 * math.sin(frame * 0.1))
        y = int(3.5 + 3 * math.cos(frame * 0.15))
        self.set_pixel(x, y, 31, 31, 0)
        # Trail
        for i in range(1, 4):
            trail_x = int(3.5 + 3 * math.sin(frame * 0.1 - i * 0.3))
            trail_y = int(3.5 + 3 * math.cos(frame * 0.15 - i * 0.3))
            if 0 <= trail_x < 8 and 0 <= trail_y < 8:
                fade = max(0, 31 - i * 8)
                self.set_pixel(trail_x, trail_y, fade, fade, 0)

    def random_sparkle(self):
        # Keep some pixels, add new ones
        if random.random() < 0.3:
            x, y = random.randint(0, 7), random.randint(0, 7)
            r = random.randint(15, 31)
            g = random.randint(15, 31)
            b = random.randint(15, 31)
            self.set_pixel(x, y, r, g, b)
        
        # Fade some pixels
        if random.random() < 0.2:
            x, y = random.randint(0, 7), random.randint(0, 7)
            self.set_pixel(x, y, 0, 0, 0)

def main():
    rclpy.init()
    node = PixelDemo()
    
    try:
        frame = 0
        mode = 0
        mode_frames = 0
        
        print("Starting pixel demo loop (Ctrl+C to stop)...")
        
        while rclpy.ok():
            if mode == 0:
                node.spinning_wheel(frame)
            elif mode == 1:
                node.bouncing_ball(frame)
            elif mode == 2:
                node.random_sparkle()
            
            mode_frames += 1
            if mode_frames > 100:  # Switch modes every 100 frames
                mode = (mode + 1) % 3
                mode_frames = 0
                node.clear()
                time.sleep(0.5)
            
            frame += 1
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping demo...")
        node.clear()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
