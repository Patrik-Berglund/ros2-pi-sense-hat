#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class PatternPublisher(Node):
    def __init__(self):
        super().__init__('pattern_publisher')
        self.publisher = self.create_publisher(Image, '/sense_hat/led_matrix/image', 10)
        
    def publish_pattern(self, pattern):
        img = Image()
        img.width = 8
        img.height = 8
        img.encoding = "rgb8"
        img.step = 24  # 8 pixels * 3 bytes
        
        data = []
        for y in range(8):
            for x in range(8):
                if pattern == "red":
                    data.extend([255, 0, 0])
                elif pattern == "green":
                    data.extend([0, 255, 0])
                elif pattern == "blue":
                    data.extend([0, 0, 255])
                elif pattern == "white":
                    data.extend([255, 255, 255])
                elif pattern == "cross":
                    if x == y or x == 7-y:
                        data.extend([255, 0, 0])
                    else:
                        data.extend([0, 0, 0])
                else:
                    data.extend([0, 0, 0])
        
        img.data = data
        self.publisher.publish(img)
        self.get_logger().info(f'Published {pattern} pattern')

def main():
    rclpy.init()
    node = PatternPublisher()
    pattern = sys.argv[1] if len(sys.argv) > 1 else "red"
    node.publish_pattern(pattern)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
