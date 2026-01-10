#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys

class CalibrationTester(Node):
    def __init__(self):
        super().__init__('calibration_tester')
        
        # Create service clients
        self.calibrate_gyro_client = self.create_client(Trigger, '/sense_hat/imu/calibrate_gyro')
        self.calibrate_accel_client = self.create_client(Trigger, '/sense_hat/imu/calibrate_accel')
        self.calibrate_mag_client = self.create_client(Trigger, '/sense_hat/imu/calibrate_mag')
        self.calibrate_all_client = self.create_client(Trigger, '/sense_hat/imu/calibrate')
        self.save_calibration_client = self.create_client(Trigger, '/sense_hat/imu/save_calibration')
        
        # Wait for services
        self.get_logger().info('Waiting for calibration services...')
        self.calibrate_gyro_client.wait_for_service(timeout_sec=5.0)
        self.calibrate_accel_client.wait_for_service(timeout_sec=5.0)
        self.calibrate_mag_client.wait_for_service(timeout_sec=5.0)
        self.calibrate_all_client.wait_for_service(timeout_sec=5.0)
        self.save_calibration_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('All calibration services available!')

    def test_service(self, client, service_name):
        """Test a calibration service"""
        self.get_logger().info(f'Testing {service_name}...')
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'{service_name}: {"SUCCESS" if response.success else "FAILED"}')
            self.get_logger().info(f'  Message: {response.message}')
            return response.success
        else:
            self.get_logger().error(f'{service_name}: Service call failed')
            return False

def main():
    rclpy.init()
    
    tester = CalibrationTester()
    
    if len(sys.argv) > 1:
        # Test specific service
        service_name = sys.argv[1]
        if service_name == 'gyro':
            tester.test_service(tester.calibrate_gyro_client, 'Gyroscope Calibration')
        elif service_name == 'accel':
            tester.test_service(tester.calibrate_accel_client, 'Accelerometer Calibration')
        elif service_name == 'mag':
            tester.test_service(tester.calibrate_mag_client, 'Magnetometer Calibration')
        elif service_name == 'all':
            tester.test_service(tester.calibrate_all_client, 'Full IMU Calibration')
        elif service_name == 'save':
            tester.test_service(tester.save_calibration_client, 'Save Calibration')
        else:
            print("Usage: python3 test_calibration_services.py [gyro|accel|mag|all|save]")
    else:
        # Test all services
        print("\nTesting all calibration services:")
        print("=" * 40)
        
        tester.test_service(tester.calibrate_gyro_client, 'Gyroscope Calibration')
        print()
        tester.test_service(tester.calibrate_accel_client, 'Accelerometer Calibration')
        print()
        tester.test_service(tester.calibrate_mag_client, 'Magnetometer Calibration')
        print()
        tester.test_service(tester.calibrate_all_client, 'Full IMU Calibration')
        print()
        tester.test_service(tester.save_calibration_client, 'Save Calibration')
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
