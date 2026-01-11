#!/usr/bin/env python3
"""Test color sensor (RGB and illuminance)"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
from std_msgs.msg import ColorRGBA

class ColorTest(Node):
    def __init__(self):
        super().__init__('color_test')
        
        self.create_subscription(Illuminance, 'sense_hat/color/illuminance', 
                                self.illuminance_callback, 10)
        self.create_subscription(ColorRGBA, 'sense_hat/color/rgb', 
                                self.rgb_callback, 10)
        
        self.get_logger().info('Color sensor test started')

    def illuminance_callback(self, msg):
        self.get_logger().info(f'Illuminance: {msg.illuminance:.0f}')

    def rgb_callback(self, msg):
        self.get_logger().info(
            f'RGB: R={msg.r:.3f} G={msg.g:.3f} B={msg.b:.3f}'
        )

def main():
    rclpy.init()
    node = ColorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
