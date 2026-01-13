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
  void set_avg_samples(uint8_t press_avg, uint8_t temp_avg);
  void set_fifo_mean(bool enable, uint8_t samples);

private:
  I2CDevice device_;
};
