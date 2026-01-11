#!/usr/bin/env python3
"""Test environmental sensors (temperature, humidity, pressure)"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure

class EnvironmentalTest(Node):
    def __init__(self):
        super().__init__('environmental_test')
        
        self.create_subscription(Temperature, 'sense_hat/temperature/humidity_sensor', 
                                self.temp_hts_callback, 10)
        self.create_subscription(Temperature, 'sense_hat/temperature/pressure_sensor', 
                                self.temp_lps_callback, 10)
        self.create_subscription(RelativeHumidity, 'sense_hat/humidity', 
                                self.humidity_callback, 10)
        self.create_subscription(FluidPressure, 'sense_hat/pressure', 
                                self.pressure_callback, 10)
        
        self.get_logger().info('Environmental test started')

    def temp_hts_callback(self, msg):
        self.get_logger().info(f'HTS221 Temperature: {msg.temperature:.2f}°C')

    def temp_lps_callback(self, msg):
        self.get_logger().info(f'LPS25H Temperature: {msg.temperature:.2f}°C')

    def humidity_callback(self, msg):
        self.get_logger().info(f'Humidity: {msg.relative_humidity * 100:.1f}%')

    def pressure_callback(self, msg):
        hpa = msg.fluid_pressure / 100.0
        self.get_logger().info(f'Pressure: {hpa:.2f} hPa')

def main():
    rclpy.init()
    node = EnvironmentalTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
