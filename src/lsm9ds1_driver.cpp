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

LSM9DS1Driver::LSM9DS1Driver() 
  : accel_gyro_("/dev/i2c-1", LSM9DS1_AG_ADDR),
    magnetometer_("/dev/i2c-1", LSM9DS1_M_ADDR),
    imu_odr_(119), accel_range_(2), gyro_range_(245),
    mag_odr_(10), mag_range_(4), accel_bw_auto_(true), accel_bw_(1),
    mag_performance_mode_(2), mag_temp_comp_(true) {
}

LSM9DS1Driver::~LSM9DS1Driver() {
  accel_gyro_.close();
  magnetometer_.close();
}

bool LSM9DS1Driver::init() {
  if (!accel_gyro_.open()) {
    return false;
  }

  // Verify accelerometer/gyroscope device ID
  uint8_t who_am_i;
  if (!accel_gyro_.readReg(WHO_AM_I_AG, who_am_i) || who_am_i != WHO_AM_I_AG_VAL) {
    return false;
  }

  // Try to open magnetometer (optional)
  if (magnetometer_.open()) {
    uint8_t who_am_i_m;
    if (magnetometer_.readReg(WHO_AM_I_M, who_am_i_m) && who_am_i_m == WHO_AM_I_M_VAL) {
      // Configure magnetometer with current settings
      if (!setMagConfig(mag_odr_, mag_range_, mag_performance_mode_, mag_temp_comp_)) {
        magnetometer_.close();
      }
    } else {
      magnetometer_.close();
    }
  }

  // Configure IMU with current settings (both accel+gyro)
  return setIMUConfig(imu_odr_, accel_range_, gyro_range_, accel_bw_auto_, accel_bw_);
}

bool LSM9DS1Driver::readAllSensors(IMUData& data) {
  // Use 12-byte burst read from OUT_X_G (0x18) - hardware auto-wraps to accel registers
  // Based on datasheet section 3.3: "starting from OUT_X_G (18h - 19h) multiple reads can be performed"
  uint8_t burst_data[12];
  if (!accel_gyro_.readMultiReg(0x18, burst_data, 12, false)) {
    return false;
  }
  
  // Read temperature separately (not part of burst sequence)
  uint8_t temp_data[2];
  if (!accel_gyro_.readMultiReg(0x15, temp_data, 2, false)) {
    return false;
  }
  
  // Parse burst data: gyro (0-5), then accel (6-11) due to hardware auto-wrap
  int16_t raw_gyro_x = (int16_t)((burst_data[1] << 8) | burst_data[0]);
  int16_t raw_gyro_y = (int16_t)((burst_data[3] << 8) | burst_data[2]);
  int16_t raw_gyro_z = (int16_t)((burst_data[5] << 8) | burst_data[4]);
  int16_t raw_accel_x = (int16_t)((burst_data[7] << 8) | burst_data[6]);
  int16_t raw_accel_y = (int16_t)((burst_data[9] << 8) | burst_data[8]);
  int16_t raw_accel_z = (int16_t)((burst_data[11] << 8) | burst_data[10]);
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
  if (magnetometer_.isOpen()) {
    // Use 6-byte burst read like ST driver
    uint8_t mag_data[6];
    if (magnetometer_.readMultiReg(0x28, mag_data, 6, true)) {
      // Parse little endian data like ST driver
      int16_t raw_mag_x = (int16_t)((mag_data[1] << 8) | mag_data[0]);
      int16_t raw_mag_y = (int16_t)((mag_data[3] << 8) | mag_data[2]);
      int16_t raw_mag_z = (int16_t)((mag_data[5] << 8) | mag_data[4]);
      
      float mag_sens = getMagSensitivity();
      // Convert to Tesla (ROS2 standard): 1 gauss = 0.0001 Tesla
      data.mag_x = raw_mag_x * mag_sens / 1000.0f * 0.0001f;  // mgauss -> gauss -> Tesla
      data.mag_y = raw_mag_y * mag_sens / 1000.0f * 0.0001f;  // mgauss -> gauss -> Tesla
      data.mag_z = raw_mag_z * mag_sens / 1000.0f * 0.0001f;  // mgauss -> gauss -> Tesla
    }
  }
  
  return true;
}

bool LSM9DS1Driver::setIMUConfig(int odr_hz, int accel_range_g, int gyro_range_dps, 
                                 bool accel_bw_auto, int accel_bw) {
  // Validate parameters
  uint8_t odr_bits = getODRBits(odr_hz);
  uint8_t accel_fs_bits = getAccelFSBits(accel_range_g);
  uint8_t gyro_fs_bits = getGyroFSBits(gyro_range_dps);
  
  if (odr_bits == 0xFF || accel_fs_bits == 0xFF || gyro_fs_bits == 0xFF) {
    return false;
  }
  
  if (!accel_bw_auto && (accel_bw < 0 || accel_bw > 3)) {
    return false;
  }
  
  // CRITICAL: Configure CTRL_REG1_G first - this controls ODR for both sensors when both are active
  // CTRL_REG1_G: ODR_G[7:5], FS_G[4:3], 0, BW_G[1:0]
  uint8_t ctrl_reg1_g = (odr_bits << 5) | (gyro_fs_bits << 3) | 0x00; // BW_G=00 (default)
  if (!accel_gyro_.writeReg(CTRL_REG1_G, ctrl_reg1_g)) {
    return false;
  }
  
  // Configure CTRL_REG6_XL - ODR bits are ignored when gyro is active, only FS and BW matter
  // CTRL_REG6_XL: ODR_XL[7:5] (ignored), FS_XL[4:3], BW_SCAL_ODR[2], BW_XL[1:0]
  uint8_t bw_scal_odr = accel_bw_auto ? 0 : 1;
  uint8_t bw_xl = accel_bw_auto ? 0 : (accel_bw & 0x03);
  uint8_t ctrl_reg6_xl = (0b011 << 5) | (accel_fs_bits << 3) | (bw_scal_odr << 2) | bw_xl;
  if (!accel_gyro_.writeReg(CTRL_REG6_XL, ctrl_reg6_xl)) {
    return false;
  }
  
  // Update stored configuration
  imu_odr_ = odr_hz;
  accel_range_ = accel_range_g;
  gyro_range_ = gyro_range_dps;
  accel_bw_auto_ = accel_bw_auto;
  accel_bw_ = accel_bw;
  
  return true;
}

bool LSM9DS1Driver::setMagConfig(int odr_hz, int range_gauss, int performance_mode, bool temp_comp) {
  if (!magnetometer_.isOpen()) {
    return false;
  }
  
  // Validate parameters
  uint8_t odr_bits = getMagODRBits(odr_hz);
  uint8_t fs_bits = getMagFSBits(range_gauss);
  uint8_t om_bits = getMagOMBits(performance_mode);
  
  if (odr_bits == 0xFF || fs_bits == 0xFF || om_bits == 0xFF) {
    return false;
  }
  
  // CTRL_REG1_M: TEMP_COMP[7], OM[6:5], DO[4:2], 0, ST[0]
  uint8_t ctrl_reg1_m = (temp_comp ? 0x80 : 0x00) | (om_bits << 5) | (odr_bits << 2) | 0x00;
  if (!magnetometer_.writeReg(CTRL_REG1_M, ctrl_reg1_m)) {
    return false;
  }
  
  // CTRL_REG2_M: 0, FS[6:5], 0[4:0]
  uint8_t ctrl_reg2_m = (fs_bits << 5);
  if (!magnetometer_.writeReg(CTRL_REG2_M, ctrl_reg2_m)) {
    return false;
  }
  
  // CTRL_REG3_M: I2C_DISABLE[7], 0[6:3], LP[2], 0[1], MD[1:0] - continuous mode (00)
  if (!magnetometer_.writeReg(CTRL_REG3_M, 0x00)) {
    return false;
  }
  
  // Update stored configuration
  mag_odr_ = odr_hz;
  mag_range_ = range_gauss;
  mag_performance_mode_ = performance_mode;
  mag_temp_comp_ = temp_comp;
  
  return true;
}

// Legacy methods for compatibility
bool LSM9DS1Driver::setAccelRange(int range_g) {
  return setIMUConfig(imu_odr_, range_g, gyro_range_, accel_bw_auto_, accel_bw_);
}

bool LSM9DS1Driver::setGyroRange(int range_dps) {
  return setIMUConfig(imu_odr_, accel_range_, range_dps, accel_bw_auto_, accel_bw_);
}

bool LSM9DS1Driver::setMagRange(int range_gauss) {
  return setMagConfig(mag_odr_, range_gauss, mag_performance_mode_, mag_temp_comp_);
}

// Helper methods for bit mapping
uint8_t LSM9DS1Driver::getODRBits(int odr_hz) const {
  switch(odr_hz) {
    case 0:   return 0b000;  // Power-down
    case 14:  return 0b001;  // 14.9 Hz (closest to 14)
    case 15:  return 0b001;  // 14.9 Hz
    case 59:  return 0b010;  // 59.5 Hz (closest to 59)
    case 60:  return 0b010;  // 59.5 Hz (closest to 60)
    case 119: return 0b011;  // 119 Hz
    case 238: return 0b100;  // 238 Hz
    case 476: return 0b101;  // 476 Hz
    case 952: return 0b110;  // 952 Hz
    default:  return 0xFF;   // Invalid
  }
}

uint8_t LSM9DS1Driver::getAccelFSBits(int range_g) const {
  switch(range_g) {
    case 2:  return 0b00;  // ±2g
    case 4:  return 0b10;  // ±4g
    case 8:  return 0b11;  // ±8g
    case 16: return 0b01;  // ±16g
    default: return 0xFF;  // Invalid
  }
}

uint8_t LSM9DS1Driver::getGyroFSBits(int range_dps) const {
  switch(range_dps) {
    case 245:  return 0b00;  // ±245 dps
    case 500:  return 0b01;  // ±500 dps
    case 2000: return 0b11;  // ±2000 dps
    default:   return 0xFF;  // Invalid
  }
}

uint8_t LSM9DS1Driver::getMagFSBits(int range_gauss) const {
  switch(range_gauss) {
    case 4:  return 0b00;  // ±4 gauss
    case 8:  return 0b01;  // ±8 gauss
    case 12: return 0b10;  // ±12 gauss
    case 16: return 0b11;  // ±16 gauss
    default: return 0xFF;  // Invalid
  }
}

uint8_t LSM9DS1Driver::getMagODRBits(int odr_hz) const {
  // Handle fractional ODRs by checking closest match
  if (odr_hz == 0) return 0xFF;  // Power-down not supported in our config
  if (odr_hz <= 1) return 0b000;  // 0.625 Hz (closest to 1)
  if (odr_hz <= 2) return 0b001;  // 1.25 Hz (closest to 1-2)
  if (odr_hz <= 3) return 0b010;  // 2.5 Hz (closest to 3)
  if (odr_hz <= 5) return 0b011;  // 5 Hz
  if (odr_hz <= 10) return 0b100; // 10 Hz
  if (odr_hz <= 20) return 0b101; // 20 Hz
  if (odr_hz <= 40) return 0b110; // 40 Hz
  if (odr_hz <= 80) return 0b111; // 80 Hz
  return 0xFF;  // Invalid
}

uint8_t LSM9DS1Driver::getMagOMBits(int performance_mode) const {
  switch(performance_mode) {
    case 0: return 0b00;  // Low-power
    case 1: return 0b01;  // Medium-performance
    case 2: return 0b10;  // High-performance
    case 3: return 0b11;  // Ultra-high performance
    default: return 0xFF; // Invalid
  }
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
