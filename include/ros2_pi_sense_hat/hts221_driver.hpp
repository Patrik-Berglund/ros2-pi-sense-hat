#pragma once

#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <cstdint>

class HTS221Driver {
public:
  HTS221Driver();
  ~HTS221Driver();

  bool init();
  bool read_humidity(float& humidity);
  bool read_temperature(float& temperature);
  
  void set_odr(uint8_t odr);
  void set_avg_samples(uint8_t temp_avg, uint8_t hum_avg);

private:
  I2CDevice device_;
  
  // Calibration coefficients
  int16_t h0_t0_out_, h1_t0_out_;
  int16_t t0_out_, t1_out_;
  float h0_rh_, h1_rh_;
  float t0_degc_, t1_degc_;
  
  bool read_calibration();
};
