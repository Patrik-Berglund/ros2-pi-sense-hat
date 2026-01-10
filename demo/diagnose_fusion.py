#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')
        self.get_logger().info('🔍 Sensor Fusion Diagnostics')
        
        # Check what topics exist
        self.check_topics()
        
        # Check what nodes are running
        self.check_nodes()
        
        # Try to get one message from each topic
        self.test_topics()

    def check_topics(self):
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')
            
            self.get_logger().info('📡 Available topics:')
            for topic in topics:
                if 'imu' in topic or 'odom' in topic:
                    self.get_logger().info(f'  ✅ {topic}')
        except Exception as e:
            self.get_logger().error(f'Failed to list topics: {e}')

    def check_nodes(self):
        try:
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
            nodes = result.stdout.strip().split('\n')
            
            self.get_logger().info('🤖 Running nodes:')
            for node in nodes:
                if any(x in node for x in ['imu', 'ekf', 'madgwick']):
                    self.get_logger().info(f'  ✅ {node}')
        except Exception as e:
            self.get_logger().error(f'Failed to list nodes: {e}')

    def test_topics(self):
        # Test raw IMU (should work)
        self.get_logger().info('🧪 Testing topic publishing rates...')
        
        topics_to_test = [
            '/sense_hat/imu/data_raw',
            '/imu/data', 
            '/odometry/filtered'
        ]
        
        for topic in topics_to_test:
            try:
                result = subprocess.run(['ros2', 'topic', 'hz', topic], 
                                      capture_output=True, text=True, timeout=3)
                if 'average rate' in result.stdout:
                    self.get_logger().info(f'  ✅ {topic} - Publishing')
                else:
                    self.get_logger().warn(f'  ❌ {topic} - Not publishing')
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f'  ❌ {topic} - Timeout (not publishing)')
            except Exception as e:
                self.get_logger().error(f'  ❌ {topic} - Error: {e}')

def main():
    rclpy.init()
    
    diagnostic = DiagnosticNode()
    
    # Keep alive for a moment
    time.sleep(1)
    
    diagnostic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
