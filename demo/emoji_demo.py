#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/home/patrik/ros2-pi-sense-hat/install/ros2_pi_sense_hat/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from ros2_pi_sense_hat.srv import SetPixel
import time

class EmojiDemo(Node):
    def __init__(self):
        super().__init__('emoji_demo')
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

    def clear(self):
        for y in range(8):
            for x in range(8):
                self.set_pixel(x, y, 0, 0, 0)

    def draw_emoji(self, emoji_name):
        self.clear()
        
        if emoji_name == "smiley":
            # Yellow face
            for x in range(2, 6):
                for y in range(1, 7):
                    self.set_pixel(x, y, 31, 31, 0)
            # Eyes
            self.set_pixel(3, 3, 0, 0, 0)
            self.set_pixel(4, 3, 0, 0, 0)
            # Smile
            self.set_pixel(2, 5, 0, 0, 0)
            self.set_pixel(3, 6, 0, 0, 0)
            self.set_pixel(4, 6, 0, 0, 0)
            self.set_pixel(5, 5, 0, 0, 0)
            
        elif emoji_name == "heart":
            # Red heart
            pattern = [
                "  ##  ##",
                " ###### ",
                "########",
                "########",
                " ###### ",
                "  ####  ",
                "   ##   ",
                "    #   "
            ]
            for y, row in enumerate(pattern):
                for x, char in enumerate(row):
                    if char == '#':
                        self.set_pixel(x, y, 31, 0, 10)
                        
        elif emoji_name == "star":
            # Yellow star
            pattern = [
                "   ##   ",
                "   ##   ",
                " ###### ",
                "########",
                " ###### ",
                " # ## # ",
                "#  ##  #",
                "   ##   "
            ]
            for y, row in enumerate(pattern):
                for x, char in enumerate(row):
                    if char == '#':
                        self.set_pixel(x, y, 31, 31, 0)
                        
        elif emoji_name == "fire":
            # Orange/red fire
            pattern = [
                "   ##   ",
                "  ####  ",
                " ###### ",
                "########",
                "########",
                " ###### ",
                "  ####  ",
                "   ##   "
            ]
            for y, row in enumerate(pattern):
                for x, char in enumerate(row):
                    if char == '#':
                        # Gradient from yellow to red
                        r = 31
                        g = max(0, 31 - y * 4)
                        b = 0
                        self.set_pixel(x, y, r, g, b)
                        
        elif emoji_name == "rainbow":
            # Rainbow stripes
            colors = [
                (31, 0, 0),   # Red
                (31, 15, 0),  # Orange
                (31, 31, 0),  # Yellow
                (0, 31, 0),   # Green
                (0, 0, 31),   # Blue
                (15, 0, 31),  # Purple
                (31, 0, 31),  # Magenta
                (31, 31, 31)  # White
            ]
            for y in range(8):
                r, g, b = colors[y]
                for x in range(8):
                    self.set_pixel(x, y, r, g, b)
                    
        elif emoji_name == "sun":
            # Yellow sun with rays
            # Center
            for x in range(3, 5):
                for y in range(3, 5):
                    self.set_pixel(x, y, 31, 31, 0)
            # Rays
            self.set_pixel(1, 1, 31, 31, 0)
            self.set_pixel(3, 0, 31, 31, 0)
            self.set_pixel(6, 1, 31, 31, 0)
            self.set_pixel(7, 3, 31, 31, 0)
            self.set_pixel(6, 6, 31, 31, 0)
            self.set_pixel(3, 7, 31, 31, 0)
            self.set_pixel(1, 6, 31, 31, 0)
            self.set_pixel(0, 3, 31, 31, 0)

def main():
    rclpy.init()
    node = EmojiDemo()
    
    emojis = ["smiley", "heart", "star", "fire", "rainbow", "sun"]
    
    try:
        print("Starting emoji demo (Ctrl+C to stop)...")
        
        while rclpy.ok():
            for emoji in emojis:
                print(f"Showing: {emoji}")
                node.draw_emoji(emoji)
                time.sleep(2.0)  # Show each emoji for 2 seconds
            
    except KeyboardInterrupt:
        print("\nGood night! 😴")
        node.clear()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
