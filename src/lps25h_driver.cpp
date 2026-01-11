#include "ros2_pi_sense_hat/lps25h_driver.hpp"
#include <unistd.h>

static const uint8_t LPS25H_ADDR = 0x5C;
static const uint8_t WHO_AM_I = 0x0F;
static const uint8_t WHO_AM_I_VAL = 0xBD;
static const uint8_t CTRL_REG1 = 0x20;
static const uint8_t STATUS_REG = 0x27;
static const uint8_t PRESS_OUT_XL = 0x28;
static const uint8_t TEMP_OUT_L = 0x2B;

LPS25HDriver::LPS25HDriver() 
  : device_("/dev/i2c-1", LPS25H_ADDR) {}

LPS25HDriver::~LPS25HDriver() {
  device_.close();
}

bool LPS25HDriver::init() {
  if (!device_.open()) return false;

  uint8_t who_am_i;
  if (!device_.readReg(WHO_AM_I, who_am_i) || who_am_i != WHO_AM_I_VAL) return false;

  // Power on, BDU enabled, 1 Hz ODR
  device_.writeReg(CTRL_REG1, 0x84);
  usleep(10000);

  return true;
}

bool LPS25HDriver::read_pressure(float& pressure) {
  uint8_t buf[3];
  if (!device_.readMultiReg(PRESS_OUT_XL, buf, 3)) return false;
  
  int32_t p_raw = buf[0] | (buf[1] << 8) | (buf[2] << 16);
  if (p_raw & 0x800000) p_raw |= 0xFF000000;  // Sign extend
  
  pressure = p_raw / 4096.0f;
  return true;
}

bool LPS25HDriver::read_temperature(float& temperature) {
  uint8_t buf[2];
  if (!device_.readMultiReg(TEMP_OUT_L, buf, 2)) return false;
  
  int16_t t_raw = static_cast<int16_t>(buf[0] | (buf[1] << 8));
  temperature = 42.5f + (t_raw / 480.0f);
  return true;
}

void LPS25HDriver::set_odr(uint8_t odr) {
  uint8_t ctrl;
  device_.readReg(CTRL_REG1, ctrl);
  ctrl = (ctrl & 0x8F) | ((odr & 0x07) << 4);
  device_.writeReg(CTRL_REG1, ctrl);
}
