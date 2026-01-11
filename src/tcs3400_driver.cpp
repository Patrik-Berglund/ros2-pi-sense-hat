#include "ros2_pi_sense_hat/tcs3400_driver.hpp"
#include <unistd.h>

static const uint8_t ENABLE = 0x80;
static const uint8_t ATIME = 0x81;
static const uint8_t CONTROL = 0x8F;
static const uint8_t ID = 0x92;
static const uint8_t STATUS = 0x93;
static const uint8_t CDATAL = 0x94;
static const uint8_t RDATAL = 0x96;
static const uint8_t GDATAL = 0x98;
static const uint8_t BDATAL = 0x9A;
static const uint8_t IR_REG = 0xC0;

TCS3400Driver::TCS3400Driver() 
  : device_("/dev/i2c-1", 0x29), detected_addr_(0) {}

TCS3400Driver::~TCS3400Driver() {
  device_.close();
}

bool TCS3400Driver::init() {
  // Try both addresses: 0x29 and 0x39 (per official python-sense-hat code)
  uint8_t addresses[] = {0x29, 0x39};
  
  for (uint8_t addr : addresses) {
    I2CDevice test_device("/dev/i2c-1", addr);
    if (!test_device.open()) continue;
    
    uint8_t id;
    if (!test_device.readReg(ID, id)) {
      test_device.close();
      continue;
    }
    
    // Check for TCS340x: (id & 0xF8) == 0x90
    // Valid IDs: 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97
    if ((id & 0xF8) == 0x90) {
      detected_addr_ = addr;
      test_device.close();
      
      // Re-open with detected address
      device_ = I2CDevice("/dev/i2c-1", detected_addr_);
      if (!device_.open()) return false;
      
      // Power on
      device_.writeReg(ENABLE, 0x01);
      usleep(3000);
      
      // Enable ADC, default integration time and gain
      device_.writeReg(ATIME, 0xF6);  // 27.8 ms
      device_.writeReg(CONTROL, 0x02);  // 16x gain
      device_.writeReg(ENABLE, 0x03);  // PON + AEN
      usleep(30000);
      
      return true;
    }
    
    test_device.close();
  }
  
  return false;  // Not found at either address
}

bool TCS3400Driver::read_rgbc(uint16_t& red, uint16_t& green, uint16_t& blue, uint16_t& clear) {
  uint8_t buf[2];
  
  if (!device_.readMultiReg(CDATAL, buf, 2)) return false;
  clear = buf[0] | (buf[1] << 8);
  
  if (!device_.readMultiReg(RDATAL, buf, 2)) return false;
  red = buf[0] | (buf[1] << 8);
  
  if (!device_.readMultiReg(GDATAL, buf, 2)) return false;
  green = buf[0] | (buf[1] << 8);
  
  if (!device_.readMultiReg(BDATAL, buf, 2)) return false;
  blue = buf[0] | (buf[1] << 8);
  
  return true;
}

bool TCS3400Driver::read_ir(uint16_t& ir) {
  device_.writeReg(IR_REG, 0x80);
  usleep(30000);
  
  uint8_t buf[2];
  if (!device_.readMultiReg(CDATAL, buf, 2)) return false;
  ir = buf[0] | (buf[1] << 8);
  
  device_.writeReg(IR_REG, 0x00);
  return true;
}

void TCS3400Driver::set_integration_time(uint8_t atime) {
  device_.writeReg(ATIME, atime);
}

void TCS3400Driver::set_gain(uint8_t gain) {
  device_.writeReg(CONTROL, gain & 0x03);
}
