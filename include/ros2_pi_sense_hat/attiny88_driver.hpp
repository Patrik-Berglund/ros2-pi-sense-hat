#pragma once

#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <array>

class ATTiny88Driver {
public:
  ATTiny88Driver();
  
  bool init();
  void clear();
  void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
  void setAll(const uint8_t* rgb_data, size_t length);
  void update();

private:
  I2CDevice i2c_;
  std::array<uint8_t, 192> framebuffer_;
};
