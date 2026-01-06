#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import math

class DemoPatterns(Node):
    def __init__(self):
        super().__init__('demo_patterns')
        self.publisher = self.create_publisher(Image, '/sense_hat/led_matrix/image', 10)
        
    def publish_pattern(self, data):
        img = Image()
        img.width = 8
        img.height = 8
        img.encoding = "rgb8"
        img.step = 24
        img.data = data
        self.publisher.publish(img)
        
    def rainbow_spiral(self, frame):
        data = []
        for y in range(8):
            for x in range(8):
                angle = math.atan2(y-3.5, x-3.5) + frame * 0.2
                hue = (angle + math.pi) / (2 * math.pi)
                r = int(255 * (0.5 + 0.5 * math.sin(hue * 6.28)))
                g = int(255 * (0.5 + 0.5 * math.sin(hue * 6.28 + 2.09)))
                b = int(255 * (0.5 + 0.5 * math.sin(hue * 6.28 + 4.18)))
                data.extend([r, g, b])
        return data
        
    def pulsing_heart(self, frame):
        heart = [
            [0,1,1,0,0,1,1,0],
            [1,1,1,1,1,1,1,1],
            [1,1,1,1,1,1,1,1],
            [1,1,1,1,1,1,1,1],
            [0,1,1,1,1,1,1,0],
            [0,0,1,1,1,1,0,0],
            [0,0,0,1,1,0,0,0],
            [0,0,0,0,0,0,0,0]
        ]
        pulse = int(128 + 127 * math.sin(frame * 0.3))
        data = []
        for y in range(8):
            for x in range(8):
                if heart[y][x]:
                    data.extend([pulse, 20, 60])
                else:
                    data.extend([10, 0, 20])
        return data
        
    def fire_effect(self, frame):
        data = []
        for y in range(8):
            for x in range(8):
                heat = max(0, 7-y + math.sin(frame * 0.1 + x) * 2)
                r = min(255, int(heat * 40))
                g = min(255, max(0, int((heat-2) * 30)))
                b = max(0, int((heat-4) * 20))
                data.extend([r, g, b])
        return data
        
    def matrix_rain(self, frame):
        data = []
        for y in range(8):
            for x in range(8):
                drop = (frame + x * 3) % 16
                if drop < 8 and drop == y:
                    data.extend([0, 255, 100])
                elif drop < 8 and drop-1 == y:
                    data.extend([0, 150, 50])
                elif drop < 8 and drop-2 == y:
                    data.extend([0, 80, 20])
                else:
                    data.extend([0, 10, 5])
        return data

def main():
    rclpy.init()
    node = DemoPatterns()
    
    pattern = sys.argv[1] if len(sys.argv) > 1 else "rainbow"
    
    try:
        frame = 0
        while rclpy.ok():
            if pattern == "rainbow":
                data = node.rainbow_spiral(frame)
            elif pattern == "heart":
                data = node.pulsing_heart(frame)
            elif pattern == "fire":
                data = node.fire_effect(frame)
            elif pattern == "matrix":
                data = node.matrix_rain(frame)
            else:
                data = node.rainbow_spiral(frame)
                
            node.publish_pattern(data)
            time.sleep(0.1)
            frame += 1
            
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
