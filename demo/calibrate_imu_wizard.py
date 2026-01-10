#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class IMUCalibrationDemo(Node):
    def __init__(self):
        super().__init__('imu_calibration_demo')
        
        # Service clients
        self.gyro_cal = self.create_client(Trigger, '/sense_hat/imu/calibrate_gyro')
        self.accel_cal = self.create_client(Trigger, '/sense_hat/imu/calibrate_accel')
        self.mag_cal = self.create_client(Trigger, '/sense_hat/imu/calibrate_mag')
        self.save_cal = self.create_client(Trigger, '/sense_hat/imu/save_calibration')
        
        self.get_logger().info("🎯 IMU Calibration Demo Ready")
    
    def call_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"❌ Service {service_name} not available")
            return False
        
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(f"✅ {service_name}: {result.message}")
                return True
            else:
                self.get_logger().error(f"❌ {service_name} failed: {result.message}")
                return False
        else:
            self.get_logger().error(f"❌ Service call failed for {service_name}")
            return False
    
    def run_calibration_wizard(self):
        print("\n" + "="*50)
        print("🎯 LSM9DS1 IMU Calibration Wizard")
        print("="*50)
        
        # Step 1: Gyroscope calibration
        print("\n📍 Step 1: Gyroscope Bias Calibration")
        print("   - Place IMU on a stable, level surface")
        print("   - Do NOT move or vibrate the IMU")
        print("   - Calibration will take ~30 seconds")
        input("   Press Enter when ready...")
        
        if self.call_service(self.gyro_cal, "Gyroscope Calibration"):
            print("✅ Gyroscope calibration completed!")
        else:
            print("❌ Gyroscope calibration failed!")
            return
        
        # Step 2: Accelerometer calibration
        print("\n📍 Step 2: Accelerometer Offset Calibration")
        print("   - Keep IMU level (Z-axis pointing up)")
        print("   - Keep stationary during calibration")
        print("   - Calibration will take ~10 seconds")
        input("   Press Enter when ready...")
        
        if self.call_service(self.accel_cal, "Accelerometer Calibration"):
            print("✅ Accelerometer calibration completed!")
        else:
            print("❌ Accelerometer calibration failed!")
            return
        
        # Step 3: Magnetometer calibration
        print("\n📍 Step 3: Magnetometer Hard-Iron Calibration")
        print("   - Rotate IMU slowly in ALL directions")
        print("   - Make figure-8 motions in 3D space")
        print("   - Continue rotating for full 60 seconds")
        print("   - Stay away from metal objects!")
        input("   Press Enter when ready to start rotating...")
        
        if self.call_service(self.mag_cal, "Magnetometer Calibration"):
            print("✅ Magnetometer calibration completed!")
        else:
            print("❌ Magnetometer calibration failed!")
            return
        
        # Step 4: Save calibration
        print("\n💾 Saving calibration data...")
        if self.call_service(self.save_cal, "Save Calibration"):
            print("✅ Calibration saved successfully!")
        else:
            print("❌ Failed to save calibration!")
            return
        
        print("\n" + "="*50)
        print("🎉 IMU Calibration Complete!")
        print("   Your IMU is now calibrated and ready to use.")
        print("   Restart the IMU node to apply calibration.")
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    
    demo = IMUCalibrationDemo()
    
    try:
        demo.run_calibration_wizard()
    except KeyboardInterrupt:
        print("\n❌ Calibration cancelled by user")
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
