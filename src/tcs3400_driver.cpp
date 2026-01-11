#include "ros2_pi_sense_hat/tcs3400_driver.hpp"
#include <unistd.h>

static const uint8_t CMD_BIT = 0x80;
static const uint8_t ENABLE = 0x00;
static const uint8_t ATIME = 0x01;
static const uint8_t CONTROL = 0x0F;
static const uint8_t ID = 0x12;
static const uint8_t STATUS = 0x13;
static const uint8_t CDATAL = 0x14;
static const uint8_t RDATAL = 0x16;
static const uint8_t GDATAL = 0x18;
static const uint8_t BDATAL = 0x1A;

TCS3400Driver::TCS3400Driver() 
  : device_("/dev/i2c-1", 0x29), detected_addr_(0) {}

TCS3400Driver::~TCS3400Driver() {
  device_.close();
}

bool TCS3400Driver::init() {
  if (!device_.open()) return false;
  
  uint8_t id;
  if (!device_.readReg(CMD_BIT | ID, id)) return false;
  
  // TCS34725 ID is 0x44
  if (id != 0x44) return false;
  
  // Power on
  device_.writeReg(CMD_BIT | ENABLE, 0x01);
  usleep(3000);
  
  // Enable ADC with default integration time and gain
  device_.writeReg(CMD_BIT | ATIME, 0xF6);  // 24ms
  device_.writeReg(CMD_BIT | CONTROL, 0x02);  // 16x gain
  device_.writeReg(CMD_BIT | ENABLE, 0x03);  // PON + AEN
  usleep(30000);
  
  return true;
}

bool TCS3400Driver::read_rgbc(uint16_t& red, uint16_t& green, uint16_t& blue, uint16_t& clear) {
  uint8_t buf[2];
  
  if (!device_.readMultiReg(CMD_BIT | CDATAL, buf, 2)) return false;
  clear = buf[0] | (buf[1] << 8);
  
  if (!device_.readMultiReg(CMD_BIT | RDATAL, buf, 2)) return false;
  red = buf[0] | (buf[1] << 8);
  
  if (!device_.readMultiReg(CMD_BIT | GDATAL, buf, 2)) return false;
  green = buf[0] | (buf[1] << 8);
  
  if (!device_.readMultiReg(CMD_BIT | BDATAL, buf, 2)) return false;
  blue = buf[0] | (buf[1] << 8);
  
  return true;
}

bool TCS3400Driver::read_ir(uint16_t& ir) {
  // TCS34725 doesn't have separate IR channel
  ir = 0;
  return true;
}

void TCS3400Driver::set_integration_time(uint8_t atime) {
  device_.writeReg(CMD_BIT | ATIME, atime);
}

void TCS3400Driver::set_gain(uint8_t gain) {
  device_.writeReg(CMD_BIT | CONTROL, gain & 0x03);
}
