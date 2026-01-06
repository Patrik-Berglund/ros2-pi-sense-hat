#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/home/patrik/ros2-pi-sense-hat/install/ros2_pi_sense_hat/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from ros2_pi_sense_hat.srv import SetPixel
import time

class CountdownDemo(Node):
    def __init__(self):
        super().__init__('countdown_demo')
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

    def draw_digit(self, digit):
        # 5x7 digit patterns (centered on 8x8 display)
        patterns = {
            0: ["  ###  ",
                " #   # ",
                " #   # ",
                " #   # ",
                " #   # ",
                " #   # ",
                "  ###  "],
            1: ["   #   ",
                "  ##   ",
                "   #   ",
                "   #   ",
                "   #   ",
                "   #   ",
                " ##### "],
            2: [" ##### ",
                "#     #",
                "      #",
                " ##### ",
                "#      ",
                "#      ",
                "#######"],
            3: [" ##### ",
                "#     #",
                "      #",
                " ##### ",
                "      #",
                "#     #",
                " ##### "],
            4: ["#     #",
                "#     #",
                "#     #",
                "#######",
                "      #",
                "      #",
                "      #"],
            5: ["#######",
                "#      ",
                "#      ",
                "###### ",
                "      #",
                "#     #",
                " ##### "],
            6: [" ##### ",
                "#     #",
                "#      ",
                "###### ",
                "#     #",
                "#     #",
                " ##### "],
            7: ["#######",
                "      #",
                "     # ",
                "    #  ",
                "   #   ",
                "  #    ",
                " #     "],
            8: [" ##### ",
                "#     #",
                "#     #",
                " ##### ",
                "#     #",
                "#     #",
                " ##### "],
            9: [" ##### ",
                "#     #",
                "#     #",
                " ######",
                "      #",
                "#     #",
                " ##### "]
        }
        
        self.clear()
        pattern = patterns[digit]
        
        # Draw pattern centered on 8x8 display
        for y, row in enumerate(pattern):
            for x, char in enumerate(row):
                if char == '#':
                    # Color based on digit (red for low numbers, green for high)
                    r = max(0, 31 - digit * 3)
                    g = min(31, digit * 3)
                    self.set_pixel(x, y, r, g, 0)

def main():
    rclpy.init()
    node = CountdownDemo()
    
    try:
        print("Starting countdown demo (Ctrl+C to stop)...")
        
        while rclpy.ok():
            for digit in range(9, -1, -1):
                print(f"Showing: {digit}")
                node.draw_digit(digit)
                time.sleep(1.0)  # Show each digit for 1 second
            
    except KeyboardInterrupt:
        print("\nStopping countdown...")
        node.clear()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
