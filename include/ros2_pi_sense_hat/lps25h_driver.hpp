#pragma once

#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <cstdint>

class LPS25HDriver {
public:
  LPS25HDriver();
  ~LPS25HDriver();

  bool init();
  bool read_pressure(float& pressure);
  bool read_temperature(float& temperature);
  
  void set_odr(uint8_t odr);

private:
  I2CDevice device_;
};
