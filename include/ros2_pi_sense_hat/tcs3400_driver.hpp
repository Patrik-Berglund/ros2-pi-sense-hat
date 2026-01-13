#pragma once

#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <cstdint>

class TCS3400Driver {
public:
  TCS3400Driver();
  ~TCS3400Driver();

  bool init();
  bool read_rgbc(uint16_t& red, uint16_t& green, uint16_t& blue, uint16_t& clear);
  bool read_ir(uint16_t& ir);
  
  void set_integration_time(uint8_t atime);
  void set_gain(uint8_t gain);
  void set_wait_time(bool enable, uint8_t wtime, bool long_wait);
  void enable();

private:
  I2CDevice device_;
  uint8_t detected_addr_;
};
