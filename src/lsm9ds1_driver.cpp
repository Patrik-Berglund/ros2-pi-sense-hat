#include "ros2_pi_sense_hat/lsm9ds1_driver.hpp"
#include <cmath>
#include <unistd.h>

// I2C addresses
static const uint8_t LSM9DS1_AG_ADDR = 0x6A;  // Accel/Gyro
static const uint8_t LSM9DS1_M_ADDR = 0x1C;   // Magnetometer

// Register addresses
static const uint8_t WHO_AM_I_AG = 0x0F;
static const uint8_t WHO_AM_I_M = 0x0F;
static const uint8_t CTRL_REG1_G = 0x10;
static const uint8_t CTRL_REG6_XL = 0x20;
static const uint8_t CTRL_REG1_M = 0x20;
static const uint8_t CTRL_REG2_M = 0x21;
static const uint8_t CTRL_REG3_M = 0x22;
static const uint8_t STATUS_REG_AG = 0x27;
static const uint8_t STATUS_REG_M = 0x27;
static const uint8_t OUT_X_G = 0x18;

// Expected WHO_AM_I values
static const uint8_t WHO_AM_I_AG_VAL = 0x68;
static const uint8_t WHO_AM_I_M_VAL = 0x3D;

LSM9DS1Driver::LSM9DS1Driver(rclcpp::Node* node) 
  : accel_gyro_(node, LSM9DS1_AG_ADDR),
    magnetometer_(node, LSM9DS1_M_ADDR),
    accel_range_(2), gyro_range_(245), mag_range_(4) {
}

LSM9DS1Driver::~LSM9DS1Driver() {
}

bool LSM9DS1Driver::init() {
  // Verify accelerometer/gyroscope device ID
  uint8_t who_am_i;
  if (!accel_gyro_.readReg(WHO_AM_I_AG, who_am_i) || who_am_i != WHO_AM_I_AG_VAL) {
    return false;
  }

  // Try to read magnetometer (optional)
  uint8_t who_am_i_m;
  if (magnetometer_.readReg(WHO_AM_I_M, who_am_i_m) && who_am_i_m == WHO_AM_I_M_VAL) {
    // Configure magnetometer
    // CTRL_REG1_M: TEMP_COMP=1, OM=10 (high-perf), DO=101 (20Hz)
    magnetometer_.writeReg(CTRL_REG1_M, 0x9C);
    
    // CTRL_REG2_M: FS=00 (±4 gauss)
    magnetometer_.writeReg(CTRL_REG2_M, 0x00);
    
    // CTRL_REG3_M: MD=00 (continuous mode)
    magnetometer_.writeReg(CTRL_REG3_M, 0x00);
  }

  // Configure both accel+gyro (activates both at same ODR)
  // CTRL_REG1_G: ODR=119Hz (011), FS=245dps (00), BW=default (00)
  if (!accel_gyro_.writeReg(CTRL_REG1_G, 0x60)) return false;
  
  // Enable accelerometer: CTRL_REG6_XL: ODR=119Hz (011), FS=2g (00)
  if (!accel_gyro_.writeReg(CTRL_REG6_XL, 0x60)) return false;
  
  return true;
}

bool LSM9DS1Driver::readAllSensors(IMUData& data) {
  // Read gyroscope (6 bytes from 0x18) and accelerometer (6 bytes from 0x28) separately
  // Temperature is at 0x15-0x16, not included in burst reads
  uint8_t gyro_data[6];
  if (!accel_gyro_.readMultiReg(0x18, gyro_data, 6)) {
    return false;
  }
  
  uint8_t accel_data[6];
  if (!accel_gyro_.readMultiReg(0x28, accel_data, 6)) {
    return false;
  }
  
  uint8_t temp_data[2];
  if (!accel_gyro_.readMultiReg(0x15, temp_data, 2)) {
    return false;
  }
  
  // Parse data with ST endianness (high byte first, then low byte)
  int16_t raw_gyro_x = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
  int16_t raw_gyro_y = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
  int16_t raw_gyro_z = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);
  int16_t raw_accel_x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
  int16_t raw_accel_y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
  int16_t raw_accel_z = (int16_t)((accel_data[5] << 8) | accel_data[4]);
  int16_t raw_temp = (int16_t)((temp_data[1] << 8) | temp_data[0]);
  
  // Convert to physical units
  float gyro_sens = getGyroSensitivity();
  data.gyro_x = raw_gyro_x * gyro_sens * M_PI / 180.0f / 1000.0f;
  data.gyro_y = raw_gyro_y * gyro_sens * M_PI / 180.0f / 1000.0f;
  data.gyro_z = raw_gyro_z * gyro_sens * M_PI / 180.0f / 1000.0f;
  
  float accel_sens = getAccelSensitivity();
  data.accel_x = raw_accel_x * accel_sens * 9.81f / 1000.0f;
  data.accel_y = raw_accel_y * accel_sens * 9.81f / 1000.0f;
  data.accel_z = raw_accel_z * accel_sens * 9.81f / 1000.0f;
  
  data.temperature = (raw_temp / 16.0f) + 25.0f;
  
  // Read magnetometer (separate I2C device) - optional
  data.mag_x = data.mag_y = data.mag_z = 0.0f;
  // Try to read magnetometer - if it fails, just skip it
  uint8_t mag_data[6];
  if (magnetometer_.readMultiReg(0x28, mag_data, 6)) {
    // Parse little endian data like ST driver
    int16_t raw_mag_x = (int16_t)((mag_data[1] << 8) | mag_data[0]);
    int16_t raw_mag_y = (int16_t)((mag_data[3] << 8) | mag_data[2]);
    int16_t raw_mag_z = (int16_t)((mag_data[5] << 8) | mag_data[4]);
    
    float mag_sens = getMagSensitivity();
    data.mag_x = raw_mag_x * mag_sens / 1000.0f;
    data.mag_y = raw_mag_y * mag_sens / 1000.0f;
    data.mag_z = raw_mag_z * mag_sens / 1000.0f;
  }
  
  return true;
}

bool LSM9DS1Driver::setAccelRange(int range_g) {
  uint8_t fs_xl;
  switch(range_g) {
    case 2:  fs_xl = 0b00; break;  // ±2g
    case 4:  fs_xl = 0b10; break;  // ±4g
    case 8:  fs_xl = 0b11; break;  // ±8g
    case 16: fs_xl = 0b01; break;  // ±16g
    default: return false;
  }
  
  // CTRL_REG6_XL: ODR=119Hz (011), FS_XL=range
  uint8_t reg_val = (0b011 << 5) | (fs_xl << 3);
  if (accel_gyro_.writeReg(CTRL_REG6_XL, reg_val)) {
    accel_range_ = range_g;
    return true;
  }
  return false;
}

bool LSM9DS1Driver::setGyroRange(int range_dps) {
  uint8_t fs_g;
  switch(range_dps) {
    case 245:  fs_g = 0b00; break;  // ±245 dps
    case 500:  fs_g = 0b01; break;  // ±500 dps
    case 2000: fs_g = 0b11; break;  // ±2000 dps
    default: return false;
  }
  
  // CTRL_REG1_G: ODR=119Hz (011), FS_G=range, BW=default (00)
  uint8_t reg_val = (0b011 << 5) | (fs_g << 3) | 0b00;
  if (accel_gyro_.writeReg(CTRL_REG1_G, reg_val)) {
    gyro_range_ = range_dps;
    return true;
  }
  return false;
}

bool LSM9DS1Driver::setMagRange(int range_gauss) {
  uint8_t fs_m;
  switch(range_gauss) {
    case 4:  fs_m = 0b00; break;  // ±4 gauss
    case 8:  fs_m = 0b01; break;  // ±8 gauss
    case 12: fs_m = 0b10; break;  // ±12 gauss
    case 16: fs_m = 0b11; break;  // ±16 gauss
    default: return false;
  }
  
  // CTRL_REG2_M: FS=range
  uint8_t reg_val = (fs_m << 5);
  if (magnetometer_.writeReg(CTRL_REG2_M, reg_val)) {
    mag_range_ = range_gauss;
    return true;
  }
  return false;
}

float LSM9DS1Driver::getAccelSensitivity() const {
  switch(accel_range_) {
    case 2:  return 0.061f;   // mg/LSB
    case 4:  return 0.122f;
    case 8:  return 0.244f;
    case 16: return 0.732f;
    default: return 0.061f;
  }
}

float LSM9DS1Driver::getGyroSensitivity() const {
  switch(gyro_range_) {
    case 245:  return 8.75f;   // mdps/LSB
    case 500:  return 17.50f;
    case 2000: return 70.0f;
    default: return 8.75f;
  }
}

float LSM9DS1Driver::getMagSensitivity() const {
  switch(mag_range_) {
    case 4:  return 0.14f;    // mgauss/LSB
    case 8:  return 0.29f;
    case 12: return 0.43f;
    case 16: return 0.58f;
    default: return 0.14f;
  }
}
