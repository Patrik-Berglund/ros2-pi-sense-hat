#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/home/patrik/ros2-pi-sense-hat/install/ros2_pi_sense_hat/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_subscriber')
        self.subscription = self.create_subscription(
            Joy, '/sense_hat/joystick', self.joystick_callback, 10)
        self.get_logger().info('Listening for joystick events...')
        self.get_logger().info('Press joystick buttons to see events!')

    def joystick_callback(self, msg):
        # Show which buttons are pressed
        pressed_buttons = []
        for i, pressed in enumerate(msg.buttons):
            if pressed:
                pressed_buttons.append(f"Button{i}")
        
        if pressed_buttons:
            buttons_str = ", ".join(pressed_buttons)
            self.get_logger().info(f'Joystick: {buttons_str} pressed')
        else:
            self.get_logger().info('Joystick: All buttons released')

def main():
    rclpy.init()
    node = JoystickSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nJoystick subscriber stopped")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
