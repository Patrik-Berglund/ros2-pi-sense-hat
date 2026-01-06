#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/home/patrik/ros2-pi-sense-hat/install/ros2_pi_sense_hat/lib/python3.12/site-packages')
import rclpy
from rclpy.node import Node
from ros2_pi_sense_hat.srv import ReadJoystick
import time

class JoystickTest(Node):
    def __init__(self):
        super().__init__('joystick_test')
        self.client = self.create_client(ReadJoystick, '/sense_hat/joystick/read')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for joystick service...')

    def read_joystick(self):
        request = ReadJoystick.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().buttons

def main():
    rclpy.init()
    node = JoystickTest()
    
    print("Joystick test - press buttons (Ctrl+C to exit)")
    print("Button mapping: bit4 bit3 bit2 bit1 bit0")
    
    last_buttons = 0
    
    try:
        while rclpy.ok():
            buttons = node.read_joystick()
            
            if buttons != last_buttons:
                # Show binary representation
                binary = format(buttons, '05b')
                print(f"Buttons: 0x{buttons:02X} ({binary}) - Changed!")
                
                # Show individual button states
                for i in range(5):
                    if (buttons >> i) & 1:
                        print(f"  Button {i}: PRESSED")
                
                last_buttons = buttons
            
            time.sleep(0.1)  # 100ms polling
            
    except KeyboardInterrupt:
        print("\nJoystick test stopped")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
