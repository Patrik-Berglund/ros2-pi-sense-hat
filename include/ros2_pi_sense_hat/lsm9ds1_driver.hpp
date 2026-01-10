#pragma once

#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <cstdint>

struct IMUData {
  float accel_x, accel_y, accel_z;  // m/s²
  float gyro_x, gyro_y, gyro_z;     // rad/s
  float mag_x, mag_y, mag_z;        // Tesla (ROS2 standard)
  float temperature;                // °C
};

class LSM9DS1Driver {
public:
  LSM9DS1Driver();
  ~LSM9DS1Driver();

  bool init();
  bool readAllSensors(IMUData& data);
  
  // Configuration methods
  bool setIMUConfig(int odr_hz, int accel_range_g, int gyro_range_dps, 
                    bool accel_bw_auto = true, int accel_bw = 1);
  bool setMagConfig(int odr_hz, int range_gauss, int performance_mode = 2, 
                    bool temp_comp = true);
  
  // Legacy methods for compatibility
  bool setAccelRange(int range_g);    // 2, 4, 8, 16
  bool setGyroRange(int range_dps);   // 245, 500, 2000
  bool setMagRange(int range_gauss);  // 4, 8, 12, 16

private:
  I2CDevice accel_gyro_;
  I2CDevice magnetometer_;
  
  // Current configuration
  int imu_odr_;
  int accel_range_;
  int gyro_range_;
  int mag_odr_;
  int mag_range_;
  bool accel_bw_auto_;
  int accel_bw_;
  int mag_performance_mode_;
  bool mag_temp_comp_;
  
  // Helper methods
  uint8_t getODRBits(int odr_hz) const;
  uint8_t getAccelFSBits(int range_g) const;
  uint8_t getGyroFSBits(int range_dps) const;
  uint8_t getMagFSBits(int range_gauss) const;
  uint8_t getMagODRBits(int odr_hz) const;
  uint8_t getMagOMBits(int performance_mode) const;
  
  float getAccelSensitivity() const;
  float getGyroSensitivity() const;
  float getMagSensitivity() const;
};
