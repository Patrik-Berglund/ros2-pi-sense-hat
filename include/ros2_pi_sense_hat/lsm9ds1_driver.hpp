#pragma once

#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <cstdint>

struct IMUData {
  float accel_x, accel_y, accel_z;  // m/s²
  float gyro_x, gyro_y, gyro_z;     // rad/s
  float mag_x, mag_y, mag_z;        // gauss
  float temperature;                // °C
};

class LSM9DS1Driver {
public:
  LSM9DS1Driver();
  ~LSM9DS1Driver();

  bool init();
  bool readAllSensors(IMUData& data);
  
  // Configuration
  bool setAccelRange(int range_g);    // 2, 4, 8, 16
  bool setGyroRange(int range_dps);   // 245, 500, 2000
  bool setMagRange(int range_gauss);  // 4, 8, 12, 16

private:
  I2CDevice accel_gyro_;
  I2CDevice magnetometer_;
  
  int accel_range_;
  int gyro_range_;
  int mag_range_;
  
  float getAccelSensitivity() const;
  float getGyroSensitivity() const;
  float getMagSensitivity() const;
};
