#pragma once

#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <array>
#include <gpiod.h>

class ATTiny88Driver {
public:
  ATTiny88Driver();
  ~ATTiny88Driver();
  
  bool initI2C();
  bool initFrameSync();
  bool initJoystickInterrupt();
  void clear();
  void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
  void setAll(const uint8_t* rgb_data, size_t length);
  
  // Joystick functions
  uint8_t readJoystick();
  bool waitForJoystickEvent(int timeout_ms = 100);

private:
  void waitForFrameSync();
  void cleanupGPIO();
  
  I2CDevice i2c_;
  std::array<uint8_t, 192> framebuffer_;
  struct gpiod_chip* gpio_chip_;
  struct gpiod_line* frame_int_line_;
  struct gpiod_line* keys_int_line_;
  bool frame_sync_enabled_;
  bool joystick_int_enabled_;
  bool owns_gpio_chip_;  // Track if this instance owns the chip
};
