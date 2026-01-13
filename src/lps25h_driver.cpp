#include "ros2_pi_sense_hat/lps25h_driver.hpp"
#include <unistd.h>

static const uint8_t LPS25H_ADDR = 0x5C;
static const uint8_t WHO_AM_I = 0x0F;
static const uint8_t WHO_AM_I_VAL = 0xBD;
static const uint8_t CTRL_REG1 = 0x20;
static const uint8_t CTRL_REG2 = 0x21;
static const uint8_t RES_CONF = 0x10;
static const uint8_t STATUS_REG = 0x27;
static const uint8_t PRESS_OUT_XL = 0x28;
static const uint8_t TEMP_OUT_L = 0x2B;
static const uint8_t FIFO_CTRL = 0x2E;

LPS25HDriver::LPS25HDriver() 
  : device_("/dev/i2c-1", LPS25H_ADDR) {}

LPS25HDriver::~LPS25HDriver() {
  device_.close();
}

bool LPS25HDriver::init() {
  if (!device_.open()) return false;

  uint8_t who_am_i;
  if (!device_.readReg(WHO_AM_I, who_am_i) || who_am_i != WHO_AM_I_VAL) return false;

  // Software reset
  device_.writeReg(CTRL_REG2, 0x84);
  usleep(50000);  // Wait for reset to complete
  
  // Don't configure here - let node apply parameters
  return true;
}

bool LPS25HDriver::read_pressure(float& pressure) {
  uint8_t buf[3];
  if (!device_.readMultiReg(PRESS_OUT_XL, buf, 3, true)) return false;  // Enable auto-increment
  
  int32_t p_raw = (buf[2] << 16) | (buf[1] << 8) | buf[0];
  if (p_raw & 0x800000) {
    p_raw |= 0xFF000000;  // Sign extend 24-bit to 32-bit
  }
  
  pressure = (p_raw / 4096.0f) * 100.0f;  // hPa to Pa
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

void LPS25HDriver::set_avg_samples(uint8_t press_avg, uint8_t temp_avg) {
  uint8_t val = ((temp_avg & 0x03) << 2) | (press_avg & 0x03);
  device_.writeReg(RES_CONF, val);
}

void LPS25HDriver::set_fifo_mean(bool enable, uint8_t samples) {
  if (enable) {
    // Set FIFO to mean mode (110) with sample count
    uint8_t fifo_ctrl = 0xC0 | (samples & 0x1F);  // Mode=110, WTM=samples
    device_.writeReg(FIFO_CTRL, fifo_ctrl);
    
    // Enable FIFO and mean decimator
    uint8_t ctrl2;
    device_.readReg(CTRL_REG2, ctrl2);
    ctrl2 |= 0x50;  // FIFO_EN=1, FIFO_MEAN_DEC=1
    device_.writeReg(CTRL_REG2, ctrl2);
  } else {
    // Disable FIFO
    uint8_t ctrl2;
    device_.readReg(CTRL_REG2, ctrl2);
    ctrl2 &= ~0x40;  // FIFO_EN=0
    device_.writeReg(CTRL_REG2, ctrl2);
    
    // Set FIFO to bypass mode
    device_.writeReg(FIFO_CTRL, 0x00);
  }
}

void LPS25HDriver::enable() {
  uint8_t ctrl;
  device_.readReg(CTRL_REG1, ctrl);
  ctrl |= 0x84;  // PD=1 (power on), BDU=1
  device_.writeReg(CTRL_REG1, ctrl);
}
